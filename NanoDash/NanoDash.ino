/*
// Include libraries
// NeoPixel https://github.com/adafruit/Adafruit_NeoPixel
// MCP2515 https://github.com/coryjfowler/MCP2515_lib
// MPU6050 https://github.com/jrowberg/i2cdevlib/tree/master/Arduino/MPU6050
*/

#include <Adafruit_NeoPixel.h>
#include <mcp_can.h>
#include <SPI.h>
//#include "I2Cdev.h"
//#include "MPU6050_6Axis_MotionApps20.h"
//#include "Wire.h"
#include "led.h"

// Uncomment this to enable test-mode where CAN is disabled and errLEDs and RPM
// LEDs blink
// Comment this to disable test mode where CAN is used to update the parameters
// for the LEDs
// #define IN_TESTMODE

Colour colBlue(0, 0, 255);
Colour colRed(255, 0, 0);
Colour colGreen(0, 255, 0);
Colour colYellow(255, 255, 0);
Colour colOff(0, 0, 0);

// Enable or disable serial, needed for testMode!
#define SERIAL_ENABLED

/*
// Defines for LED strip
*/
static const int errorLEDs = 5;
static const int rpmLEDs = 23;
static const int totalLEDs = errorLEDs + rpmLEDs;
#define CTRL_PIN 7
#define REFRESH_FREQUENCY 50
#define REFRESH_PERIOD (1 / REFRESH_FREQUENCY) * 1000
#define DEFALT_BRIGHTNESS 25

Colour ledColArray[totalLEDs];

// Sensor error
// val != 0 -> red
// errorFlagsL(U)
int16_t sensorErr1Ref = 0;
static const int16_t err1Off = 0;
static const int16_t err1Low = 0;
static const int16_t err1High = 1;
LED err1(ledColArray, &sensorErr1Ref, err1Off, err1Low, err1High, &colOff,
         &colOff, &colOff, &colRed);

// Check engine
// Use sync state flag
// 0, 1, 2 -> Yellow
// 3 -> off
int16_t engineErr2Ref = 0;
static const int16_t err2Off = 0;
static const int16_t err2Low = 0;
static const int16_t err2High = 3;
LED err2(ledColArray + 1, &engineErr2Ref, err2Off, err2Low, err2High,
         &colYellow, &colYellow, &colYellow, &colOff);

// Coolant temp
// ect1
// Blue < 60, Green < 100, Red > 100
int16_t coolantErr3Ref = 0;
static const int16_t err3Off = -1000;
static const int16_t err3Low = 60*10;
static const int16_t err3High = 100*10;
LED err3(ledColArray + 2, &coolantErr3Ref, err3Off, err3Low, err3High, &colOff,
         &colGreen, &colBlue, &colRed);

// Battery voltage
// vbat
// Blue < 10, Red < 16, Off in between
int16_t voltageErr4Ref = 0;
static const int16_t err4Off = 0;
static const int16_t err4Low = 10 * 1000;
static const int16_t err4High = 16 * 1000;
LED err4(ledColArray + 3, &voltageErr4Ref, err4Off, err4Low, err4High, &colOff,
         &colOff, &colBlue, &colRed);

// Launch control
// != 0 -> Green
// Else off
int16_t launchErr5Ref = 0;
static const int16_t err5Off = 0;
static const int16_t err5Low = 1;
static const int16_t err5High = 1;
LED err5(ledColArray + 4, &launchErr5Ref, err5Off, err5Low, err5High, &colOff,
         &colOff, &colGreen, &colGreen);

int16_t rpmRef = 0;
static const int16_t rpmHigh = 16000;
LED rpm[rpmLEDs];

Adafruit_NeoPixel strip
    = Adafruit_NeoPixel(totalLEDs, CTRL_PIN, NEO_GRB + NEO_KHZ800);

/*
// Defines and variables for MCP2515 CAN controller
*/
#define MCP_CHIP_SELECT 10
#define MCP_INTERRUPT 3

long unsigned int rxId;
byte len = 0;
byte rxBuf[8];
byte txBuf[8];

#ifdef IN_TESTMODE
int16_t val1 = 0;
int16_t val2 = 16384;
int16_t val3 = -16384;
int16_t val4 = 8192;
#endif

MCP_CAN CAN0(MCP_CHIP_SELECT);

/*
// Function definitions
*/

void updateStrip()
{
    for (size_t i = 0; i < totalLEDs; i++)
    {
        strip.setPixelColor(i, ledColArray[i].r, ledColArray[i].g,
                            ledColArray[i].b);
        strip.show();
    }
}

void setup()
{
    /*
    // Initialize the LED strip and clear it
    */
    strip.begin();
    strip.show();

    // Delay needed during startup - Not sure why, but should be OK for now
    delay(500);

// initialize serial communication
#ifdef SERIAL_ENABLED
    Serial.begin(115200);
#endif

    for (int i = 0; i < rpmLEDs; ++i)
    {
        if (i == 0)
        {
            rpm[i] = LED(ledColArray + errorLEDs + i, &rpmRef, -1, -1, rpmHigh,
                         &colOff, &colGreen, &colOff, &colBlue);
        }
        else if (i < 4)
        {
            rpm[i] = LED(ledColArray + errorLEDs + i, &rpmRef,
                         i * (rpmHigh / rpmLEDs), i * (rpmHigh / rpmLEDs),
                         rpmHigh, &colOff, &colGreen, &colOff, &colBlue);
        }
        else if (i < 19)
        {
            rpm[i] = LED(ledColArray + errorLEDs + i, &rpmRef,
                         i * (rpmHigh / rpmLEDs), i * (rpmHigh / rpmLEDs),
                         rpmHigh, &colOff, &colRed, &colOff, &colBlue);
        }
        else
        {
            rpm[i] = LED(ledColArray + errorLEDs + i, &rpmRef,
                         i * (rpmHigh / rpmLEDs), i * (rpmHigh / rpmLEDs),
                         rpmHigh, &colOff, &colBlue, &colOff, &colBlue);
        }
        rpm[i].setDeltaTime(0);
    }

#ifndef IN_TESTMODE
#ifdef SERIAL_ENABLED
    Serial.println("Init CAN");
#endif // SERIAL_ENABLED

    CAN0.begin(CAN_1000KBPS);      // Initiate CAN to 1000KBPS
    pinMode(MCP_INTERRUPT, INPUT); // Setting pin for MCP interrupt

#ifdef SERIAL_ENABLED
    Serial.println("Done init CAN");
#endif // SERIAL_ENABLED

#endif // IN_TESTMODE
}

void loop()
{
#ifdef IN_TESTMODE
    static int upDown = 1;
    static unsigned int lastUpdate = 0;
    if ((millis() - lastUpdate) > 1)
    {
        rpmRef = rpmRef + upDown * random(1, 300);
        lastUpdate = millis();
        if (rpmRef > 20000)
        {
            upDown = upDown * -1;
            rpmRef = 20000;
        }
        else if (rpmRef <= 0)
        {
            upDown = upDown * -1;
            rpmRef = 0;
        }
    }
    sensorErr1Ref = random(-2000, 2000);
    engineErr2Ref = random(-2000, 2000);
    coolantErr3Ref = random(-2000, 2000);
    voltageErr4Ref = random(-2000, 2000);
    launchErr5Ref = random(-2000, 2000);
#endif // IN_TESTMODE

#ifndef IN_TESTMODE

    // If MCP pin is low read data from recieved over CAN bus
    if (!digitalRead(MCP_INTERRUPT))
    {
        // Serial.println("Got an interrupt.");
        CAN0.readMsgBuf(&len, rxBuf);
        rxId = CAN0.getCanId(); // Get message ID
        if (rxId == 0x600)
        {
            // Create a 16 bit number from two 8 bit data slots, store RPM
            rpmRef = (int16_t)((rxBuf[0] << 8) + rxBuf[1]);
            // Store the vbat
            voltageErr4Ref = (int16_t)((rxBuf[4] << 8) + rxBuf[5]);
        }
        else if (rxId == 0x601)
        {
            // Store the coolant temp
            coolantErr3Ref = (int16_t)((rxBuf[2] << 8) + rxBuf[3]);
        }
        else if (rxId == 0x602)
        {
          // Store the sensor error flags
          sensorErr1Ref = (int16_t)((rxBuf[0] << 8) + rxBuf[1]);
          // Store the sync state
          engineErr2Ref = (int16_t)((rxBuf[2] << 8) + rxBuf[3]);
          // Store the launch state
          launchErr5Ref = (int16_t)((rxBuf[4] << 8) + rxBuf[5]);
        }
    }

#endif // NOT IN_TESTMODE

    for (int i = 0; i < rpmLEDs; ++i)
    {
        rpm[i].checkAndUpdate();
    }
    err1.checkAndUpdate();
    err2.checkAndUpdate();
    err3.checkAndUpdate();
    err4.checkAndUpdate();
    err5.checkAndUpdate();
    updateStrip();
}
