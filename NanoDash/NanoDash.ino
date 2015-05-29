/*
// Include libraries
// NeoPixel https://github.com/adafruit/Adafruit_NeoPixel
// MCP2515 https://github.com/coryjfowler/MCP2515_lib
// MPU6050 https://github.com/jrowberg/i2cdevlib/tree/master/Arduino/MPU6050
*/

#include <Adafruit_NeoPixel.h>
//#include <mcp_can.h>
//#include <SPI.h>
//#include "I2Cdev.h"
//#include "MPU6050_6Axis_MotionApps20.h"
//#include "Wire.h"
#include "led.h"

Colour colBlue(0, 0, 255);
Colour colRed(255, 0, 0);
Colour colGreen(0, 255, 0);
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

int16_t err1Ref = 0;
int16_t err2Ref = 0;
int16_t err3Ref = 0;
int16_t err4Ref = 0;
int16_t err5Ref = 0;
int16_t rpmRef = 0;

static const int16_t err1Off = -1000;
static const int16_t err1Low = 0;
static const int16_t err1High = 1000;
static const int16_t err2Off = -1000;
static const int16_t err2Low = 0;
static const int16_t err2High = 1000;
static const int16_t err3Off = -1000;
static const int16_t err3Low = 0;
static const int16_t err3High = 1000;
static const int16_t err4Off = -1000;
static const int16_t err4Low = 0;
static const int16_t err4High = 1000;
static const int16_t err5Off = -1000;
static const int16_t err5Low = 0;
static const int16_t err5High = 1000;

static const int16_t rpmHigh = 16000;

LED err1(ledColArray, &err1Ref, err1Off, err1Low, err1High, &colOff, &colBlue,
         &colGreen, &colRed);
LED err2(ledColArray + 1, &err2Ref, err2Off, err2Low, err2High, &colOff,
         &colBlue, &colGreen, &colRed);
LED err3(ledColArray + 2, &err3Ref, err3Off, err3Low, err3High, &colOff,
         &colBlue, &colGreen, &colRed);
LED err4(ledColArray + 3, &err4Ref, err4Off, err4Low, err4High, &colOff,
         &colBlue, &colGreen, &colRed);
LED err5(ledColArray + 4, &err5Ref, err5Off, err5Low, err5High, &colOff,
         &colBlue, &colGreen, &colRed);

LED rpm[rpmLEDs];

Adafruit_NeoPixel strip
    = Adafruit_NeoPixel(totalLEDs, CTRL_PIN, NEO_GRB + NEO_KHZ800);

/*
// Function definitions
*/

void refreshLedStrip(int rpm);
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
      rpm[i] = LED(ledColArray + 5 + i, &rpmRef, -1, -1, rpmHigh, &colOff,
                   &colGreen, &colOff, &colBlue);
    }
    else if (i < 4)
    {
      rpm[i] = LED(ledColArray + 5 + i, &rpmRef, i * (rpmHigh / rpmLEDs),
                   i * (rpmHigh / rpmLEDs), rpmHigh, &colOff, &colGreen,
                   &colOff, &colBlue);
    }
    else if (i < 19)
    {
      rpm[i] = LED(ledColArray + 5 + i, &rpmRef, i * (rpmHigh / rpmLEDs),
                   i * (rpmHigh / rpmLEDs), rpmHigh, &colOff, &colRed, &colOff,
                   &colBlue);
    }
    else
    {
      rpm[i] = LED(ledColArray + 5 + i, &rpmRef, i * (rpmHigh / rpmLEDs),
                   i * (rpmHigh / rpmLEDs), rpmHigh, &colOff, &colBlue, &colOff,
                   &colBlue);
    }
    rpm[i].setDeltaTime(0);
  }
}

void loop()
{
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
  err1Ref = random(-2000, 2000);
  err2Ref = random(-2000, 2000);
  err3Ref = random(-2000, 2000);
  err4Ref = random(-2000, 2000);
  err5Ref = random(-2000, 2000);
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
