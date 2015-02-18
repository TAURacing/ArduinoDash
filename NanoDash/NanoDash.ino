/*
// Include libraries
// NeoPixel https://github.com/adafruit/Adafruit_NeoPixel
// MCP2515 https://github.com/coryjfowler/MCP2515_lib
// MPU6050 https://github.com/jrowberg/i2cdevlib/tree/master/Arduino/MPU6050
*/

#include <Adafruit_NeoPixel.h>
#include <mcp_can.h>
#include <SPI.h>

// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif

// Enable or disable serial, needed for testMode!
#define SERIAL_ENABLED

/*
// Defines for LED strip
*/

#define NUM_LED 16
#define CTRL_PIN 7
#define REFRESH_FREQUENCY 50
#define REFRESH_PERIOD (1 / REFRESH_FREQUENCY) * 1000

#define DEFALT_BRIGHTNESS 25

#define ONE_LED 1000
#define TWO_LED 2000
#define THREE_LED 3000
#define FOUR_LED 4000
#define FIVE_LED 5000
#define SIX_LED 6000
#define SEVEN_LED 7000
#define EIGHT_LED 8000
#define NINE_LED 9000
#define TEN_LED 10000
#define ELEVEN_LED 11000
#define TWELVE_LED 12000
#define THIRTEEN_LED 13000
#define FOURTEEN_LED 14000
#define FIFTEEN_LED 15000
#define SIXTEEN_LED 16000

/*
// Defines for MCP2515 CAN controller
*/

#define MCP_CHIP_SELECT 10
#define MCP_INTERRUPT 3

/*
// Defines for MPU6050 accelerometer
*/

// display initial world-frame acceleration, adjusted to remove gravity
// and rotated based on known orientation from quaternion
// #define OUTPUT_READABLE_WORLDACCEL
// Uncomment if you want the data sent over CAN.
// Note that the data format is signed int with 8192 = 1g
#define ACCELERATION_TO_CAN
#define OUTPUT_READABLE_WORLDACCEL

/*
// Setup for LED strip
*/

Adafruit_NeoPixel strip
    = Adafruit_NeoPixel(NUM_LED, CTRL_PIN, NEO_GRB + NEO_KHZ800);

/*
// Setup for MPU6050
*/

MPU6050 mpu;

/*
// Variables for MCP2515
*/
long unsigned int rxId;
unsigned char len = 0;
unsigned char rxBuf[8];
MCP_CAN CAN0(MCP_CHIP_SELECT); // Set chip select pin

/*
// Variables for MPU6050 accelerometer
*/

// MPU control/status vars
bool dmpReady = false; // set true if DMP init was successful
uint8_t mpuIntStatus;  // holds actual interrupt status byte from MPU
uint8_t devStatus; // return status after each device operation (0 = success, !0
                   // = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;        // [w, x, y, z] quaternion container
VectorInt16 aa;      // [x, y, z] accel sensor measurements
VectorInt16 aaReal;  // [x, y, z] gravity-free accel sensor measurements
VectorInt16 aaWorld; // [x, y, z] world-frame accel sensor measurements
VectorFloat gravity; // [x, y, z] gravity vector
float euler[3];      // [psi, theta, phi] Euler angle container
float ypr[3]; // [yaw, pitch, roll] yaw/pitch/roll container and gravity vector

// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================

// indicates whether MPU interrupt pin has gone high
volatile bool mpuInterrupt = false;

/*
// Function definitions
*/

void dmpDataReady() { mpuInterrupt = true; }
void refreshLedStrip(int rpm);

void setup()
{

    /*
    // Initialize the LED strip and clear it
    */
    strip.begin();
    strip.show();

    // Delay needed during startup - Not sure why, but should be OK for now
    delay(500);

/*
// Initialize I2C and the MPU6050
*/
// join I2C bus (I2Cdev library doesn't do this automatically)
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    Wire.begin();
    TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz)
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
    Fastwire::setup(400, true);
#endif

// initialize serial communication
// (115200 chosen because it is required for Teapot Demo output, but it's
// really up to you depending on your project)
#ifdef SERIAL_ENABLED
    Serial.begin(115200);
#endif

// initialize device
#ifdef SERIAL_ENABLED
    Serial.println(F("Initializing I2C devices..."));
#endif
    mpu.initialize();

// verify connection
#ifdef SERIAL_ENABLED
    Serial.println(F("Testing device connections..."));
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful")
                                        : F("MPU6050 connection failed"));
#endif

// load and configure the DMP
#ifdef SERIAL_ENABLED
    Serial.println(F("Initializing DMP..."));
#endif
    devStatus = mpu.dmpInitialize();

    // Supply your own gyro offsets here, scaled for min sensitivity
    // Note that for every MPU you need to find these values - easiest done
    // using the raw test sketch and an iterative process until everything is
    // zero.
    mpu.setXAccelOffset(-5488);
    mpu.setYAccelOffset(-1464);
    mpu.setZAccelOffset(1315);

    mpu.setXGyroOffset(0);
    mpu.setYGyroOffset(-75);
    mpu.setZGyroOffset(40);

    // make sure it worked (returns 0 if so)
    if (devStatus == 0)
    {
// turn on the DMP, now that it's ready
#ifdef SERIAL_ENABLED
        Serial.println(F("Enabling DMP..."));
#endif
        mpu.setDMPEnabled(true);

// enable Arduino interrupt detection
#ifdef SERIAL_ENABLED
        Serial.println(F(
            "Enabling interrupt detection (Arduino external interrupt 0)..."));
#endif
        attachInterrupt(0, dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

// set our DMP Ready flag so the main loop() function knows it's okay to use it
#ifdef SERIAL_ENABLED
        Serial.println(F("DMP ready! Waiting for first interrupt..."));
#endif
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
    }
    else
    {
// ERROR!
// 1 = initial memory load failed
// 2 = DMP configuration updates failed
// (if it's going to break, usually the code will be 1)
#ifdef SERIAL_ENABLED
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
#endif
    }

    /*
    // Initializing the MCP2515
    */
    delay(500); // Delay needed during startup - Not sure why, but should be OK
                // for now
    CAN0.begin(CAN_1000KBPS);      // Initiate CAN to 1000KBPS
    pinMode(MCP_INTERRUPT, INPUT); // Setting pin for MCP interrupt
}

void loop()
{

    static unsigned int rpm = 0;
    static unsigned long lastRefresh
        = 0; // Store the last time the LEDs were updated
    static unsigned long lastRpmUpdate
        = 0;                     // Used with testMode to generate mock RPM
    static boolean upDown = 1;   // Used with testMode to generate mock RPM
    static boolean testMode = 0; // Use to test the system, generate a mock RPM

#ifdef ACCELERATION_TO_CAN
    static int realAccelerationX = 0;
    static int realAccelerationY = 0;
    static int realAccelerationZ = 0;
    static boolean accelerationToCan = 0;
    unsigned char canBuffer[8] = { 0, 0, 0, 0, 0, 0, 0, 0 };
#endif

    // if programming failed, don't try to do anything
    if (!dmpReady)
        return;

    /*
    All actions not related directly to data gathering from the MPU6050 goes
    here
    */

    // wait for MPU interrupt or extra packet(s) available
    while (!mpuInterrupt && fifoCount < packetSize)
    {
        // If MCP pin is low read data from recieved over CAN bus
        if (!digitalRead(MCP_INTERRUPT))
        {
            CAN0.readMsgBuf(
                &len,
                rxBuf); // Read data: len = data length, buf = data byte(s)
            rxId = CAN0.getCanId(); // Get message ID

            // When we are in testMode the CAN data is sent out over serial
            if (testMode)
            {
                Serial.print("ID: ");
                Serial.print(rxId, HEX);
                Serial.print("  Data: ");
                for (int i = 0; i < len; i++) // Print each byte of the data
                {
                    if (rxBuf[i] < 0x10) // If data byte is less than 0x10, add
                                         // a leading zero
                    {
                        Serial.print("0");
                    }
                    Serial.print(rxBuf[i], HEX);
                    Serial.print(" ");
                }
                Serial.println();
            }
            // When not in test mode we are only interested in ID 0x600 where
            // the two first data slots contain the RPM
            else
            {
                if (rxId == 0x600)
                {
                    rpm = (int)((rxBuf[0] << 8) + rxBuf[1]); // Create a 16 bit
                                                             // number from two
                                                             // 8 bit data
                                                             // slots, store RPM
                }
            }
        }

        // If we are in testMode we want to generate a mock RPM
        if (testMode && (millis() - lastRpmUpdate > 10))
        {
            lastRpmUpdate = millis();
            // Count RPM up
            if (upDown)
            {
                rpm = rpm + random(50, 1000);
                if (rpm >= 20000)
                {
                    upDown = 0;
                    rpm = 20000;
                }
            }
            // Count RPM down
            else
            {
                rpm = rpm - random(50, 1000);
                if (rpm <= 0)
                {
                    upDown = 1;
                    rpm = 0;
                }
            }
        }

        // Update the LED strip
        if (millis() - lastRefresh > REFRESH_PERIOD)
        {
            lastRefresh = millis();
            refreshLedStrip(rpm);
        }
    }

    /*
    Anything following this point is related to data gathering from the
    MPU6050 and outputting the data over serial if enabled
    */

    // reset interrupt flag and get INT_STATUS byte
    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();

    // get current FIFO count
    fifoCount = mpu.getFIFOCount();

    // check for overflow (this should never happen unless our code is too
    // inefficient)
    if ((mpuIntStatus & 0x10) || fifoCount == 1024)
    {
        // reset so we can continue cleanly
        mpu.resetFIFO();
#ifdef SERIAL_ENABLED
        Serial.println(F("FIFO overflow!"));
#endif
    }
    // otherwise, check for DMP data ready interrupt (this should happen
    // frequently)
    else if (mpuIntStatus & 0x02)
    {
        // wait for correct available data length, should be a VERY short wait
        while (fifoCount < packetSize)
            fifoCount = mpu.getFIFOCount();

        // read a packet from FIFO
        mpu.getFIFOBytes(fifoBuffer, packetSize);

        // track FIFO count here in case there is > 1 packet available
        // (this lets us immediately read more without waiting for an interrupt)
        fifoCount -= packetSize;

#ifdef OUTPUT_READABLE_WORLDACCEL
        // display initial world-frame acceleration, adjusted to remove gravity
        // and rotated based on known orientation from quaternion
        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetAccel(&aa, fifoBuffer);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
        mpu.dmpGetLinearAccelInWorld(&aaWorld, &aaReal, &q);
        Serial.print("aworld\t");
        Serial.print(aaWorld.x);
        Serial.print("\t");
        Serial.print(aaWorld.y);
        Serial.print("\t");
        Serial.println(aaWorld.z);
#endif

#ifdef ACCELERATION_TO_CAN
        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetAccel(&aa, fifoBuffer);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
        mpu.dmpGetLinearAccelInWorld(&aaWorld, &aaReal, &q);
        realAccelerationX = map(aaWorld.x, -16384, 16384, 0, 32768);
        realAccelerationY = map(aaWorld.y, -16384, 16384, 0, 32768);
        realAccelerationZ = map(aaWorld.z, -16384, 16384, 0, 32768);
        canBuffer[0] = (realAccelerationX >> 8);
        canBuffer[1] = realAccelerationX;
        canBuffer[2] = (realAccelerationY >> 8);
        canBuffer[3] = realAccelerationY;
        canBuffer[4] = (realAccelerationZ >> 8);
        canBuffer[5] = realAccelerationZ;
        CAN0.sendMsgBuf(0x100, 1, 8, (byte*)canBuffer);
#endif
    }
}

/*
// Update the LED strip
// Note that if you want to turn off one LED you have to update all of the LEDs
// then call strip.show()
// Currently we set all LED packets to 0,0,0 (R,G,B) which means they are all
// off, before we set the individual LED values. This is possibly inefficient
// but for now it works up to an update frequency of 50kHz, more than needed for
// our use.
*/
void refreshLedStrip(unsigned int rpm)
{
    int n;
    int ledNumbers;

    for (n = 0; n < strip.numPixels(); n++)
    {
        strip.setPixelColor(n, strip.Color(0, 0, 0));
    }
    if (rpm < ONE_LED)
    {
        ledNumbers = 1;
        for (n = 0; n < ledNumbers; n++)
        {
            strip.setPixelColor(n, strip.Color(0, DEFALT_BRIGHTNESS, 0));
        }
    }
    else if (rpm < TWO_LED)
    {
        ledNumbers = 2;
        for (n = 0; n < ledNumbers; n++)
        {
            strip.setPixelColor(n, strip.Color(0, DEFALT_BRIGHTNESS, 0));
        }
    }
    else if (rpm < THREE_LED)
    {
        ledNumbers = 3;
        for (n = 0; n < ledNumbers; n++)
        {
            strip.setPixelColor(n, strip.Color(0, DEFALT_BRIGHTNESS, 0));
        }
    }
    else if (rpm < FOUR_LED)
    {
        ledNumbers = 4;
        for (n = 0; n < ledNumbers; n++)
        {
            if (n > 2)
            {
                strip.setPixelColor(n, strip.Color(DEFALT_BRIGHTNESS, 0, 0));
            }
            else
            {
                strip.setPixelColor(n, strip.Color(0, DEFALT_BRIGHTNESS, 0));
            }
        }
    }
    else if (rpm < FIVE_LED)
    {
        ledNumbers = 5;
        for (n = 0; n < ledNumbers; n++)
        {
            if (n > 2)
            {
                strip.setPixelColor(n, strip.Color(DEFALT_BRIGHTNESS, 0, 0));
            }
            else
            {
                strip.setPixelColor(n, strip.Color(0, DEFALT_BRIGHTNESS, 0));
            }
        }
    }
    else if (rpm < SIX_LED)
    {
        ledNumbers = 6;
        for (n = 0; n < ledNumbers; n++)
        {
            if (n > 2)
            {
                strip.setPixelColor(n, strip.Color(DEFALT_BRIGHTNESS, 0, 0));
            }
            else
            {
                strip.setPixelColor(n, strip.Color(0, DEFALT_BRIGHTNESS, 0));
            }
        }
    }
    else if (rpm < SEVEN_LED)
    {
        ledNumbers = 7;
        for (n = 0; n < ledNumbers; n++)
        {
            if (n > 2)
            {
                strip.setPixelColor(n, strip.Color(DEFALT_BRIGHTNESS, 0, 0));
            }
            else
            {
                strip.setPixelColor(n, strip.Color(0, DEFALT_BRIGHTNESS, 0));
            }
        }
    }
    else if (rpm < EIGHT_LED)
    {
        ledNumbers = 8;
        for (n = 0; n < ledNumbers; n++)
        {
            if (n > 2)
            {
                strip.setPixelColor(n, strip.Color(DEFALT_BRIGHTNESS, 0, 0));
            }
            else
            {
                strip.setPixelColor(n, strip.Color(0, DEFALT_BRIGHTNESS, 0));
            }
        }
    }
    else if (rpm < NINE_LED)
    {
        ledNumbers = 9;
        for (n = 0; n < ledNumbers; n++)
        {
            if (n > 2)
            {
                strip.setPixelColor(n, strip.Color(DEFALT_BRIGHTNESS, 0, 0));
            }
            else
            {
                strip.setPixelColor(n, strip.Color(0, DEFALT_BRIGHTNESS, 0));
            }
        }
    }
    else if (rpm < TEN_LED)
    {
        ledNumbers = 10;
        for (n = 0; n < ledNumbers; n++)
        {
            if (n > 2)
            {
                strip.setPixelColor(n, strip.Color(DEFALT_BRIGHTNESS, 0, 0));
            }
            else
            {
                strip.setPixelColor(n, strip.Color(0, DEFALT_BRIGHTNESS, 0));
            }
        }
    }
    else if (rpm < ELEVEN_LED)
    {
        ledNumbers = 11;
        for (n = 0; n < ledNumbers; n++)
        {
            if (n > 2)
            {
                strip.setPixelColor(n, strip.Color(DEFALT_BRIGHTNESS, 0, 0));
            }
            else
            {
                strip.setPixelColor(n, strip.Color(0, DEFALT_BRIGHTNESS, 0));
            }
        }
    }
    else if (rpm < TWELVE_LED)
    {
        ledNumbers = 12;
        for (n = 0; n < ledNumbers; n++)
        {
            if (n > 2)
            {
                strip.setPixelColor(n, strip.Color(DEFALT_BRIGHTNESS, 0, 0));
            }
            else
            {
                strip.setPixelColor(n, strip.Color(0, DEFALT_BRIGHTNESS, 0));
            }
        }
    }
    else if (rpm < THIRTEEN_LED)
    {
        ledNumbers = 13;
        for (n = 0; n < ledNumbers; n++)
        {
            if (n > 12)
            {
                strip.setPixelColor(n, strip.Color(0, 0, DEFALT_BRIGHTNESS));
            }
            else if (n > 2)
            {
                strip.setPixelColor(n, strip.Color(DEFALT_BRIGHTNESS, 0, 0));
            }
            else
            {
                strip.setPixelColor(n, strip.Color(0, DEFALT_BRIGHTNESS, 0));
            }
        }
    }
    else if (rpm < FOURTEEN_LED)
    {
        ledNumbers = 14;
        for (n = 0; n < ledNumbers; n++)
        {
            if (n > 12)
            {
                strip.setPixelColor(n, strip.Color(0, 0, DEFALT_BRIGHTNESS));
            }
            else if (n > 2)
            {
                strip.setPixelColor(n, strip.Color(DEFALT_BRIGHTNESS, 0, 0));
            }
            else
            {
                strip.setPixelColor(n, strip.Color(0, DEFALT_BRIGHTNESS, 0));
            }
        }
    }
    else if (rpm < FIFTEEN_LED)
    {
        ledNumbers = 15;
        for (n = 0; n < ledNumbers; n++)
        {
            if (n > 12)
            {
                strip.setPixelColor(n, strip.Color(0, 0, DEFALT_BRIGHTNESS));
            }
            else if (n > 2)
            {
                strip.setPixelColor(n, strip.Color(DEFALT_BRIGHTNESS, 0, 0));
            }
            else
            {
                strip.setPixelColor(n, strip.Color(0, DEFALT_BRIGHTNESS, 0));
            }
        }
    }
    // For 16 LEDs we have a special case where we want all the LEDs to turn
    // blue in order to alert the driver of a need to shift as we are way
    // outside the power band
    else
    {
        ledNumbers = 16;
        for (n = 0; n < ledNumbers; n++)
        {
            strip.setPixelColor(n, strip.Color(0, 0, DEFALT_BRIGHTNESS));
        }
    }
    strip.show();
}