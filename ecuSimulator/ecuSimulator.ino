// demo: CAN-BUS Shield, send data
#include <mcp_can.h>
#include <SPI.h>

MCP_CAN CAN0(10); // Set CS to pin 10

int n = 0;
void setup()
{
  delay(500);
  Serial.begin(115200);
  // init can bus, baudrate: 500k
  if (CAN0.begin(CAN_1000KBPS) == CAN_OK)
    Serial.print("can init ok!!\r\n");
  else
    Serial.print("Can init fail!!\r\n");
}

void generateRPMAndRandomFrames();
void generateStaticFrames();

void loop()
{
  // Generate 20 frames where only the first has RPM and gear in a sensible
  // fashion.
  generateRPMAndRandomFrames();

  // Generate frame with only 0x600 in order to debug if needed
  // generateStaticFrames();
}

void generateRPMAndRandomFrames()
{
  static uint8_t temp8bit;
  static int count;
  static unsigned int temp16bit = 0;
  static unsigned int rpm = 0;
  static unsigned int gear = 0;
  static unsigned long lastRpmUpdate = 0;
  static unsigned long lastPacket = 0;
  static unsigned long deltaT = 2;
  static boolean upDown = 1;
  byte stmp[8]
      = { random(0, 255), random(0, 255), random(0, 255), random(0, 255),
          random(0, 255), random(0, 255), random(0, 255), random(0, 255) };

  if (millis() - lastPacket > deltaT)
  {
    lastPacket = millis();
    if (n == 0)
    {
      temp8bit = ((rpm >> 8));
      stmp[0] = temp8bit;
      temp8bit = (rpm);
      stmp[1] = temp8bit;
    }
    else if (n == 1)
    {
      temp8bit = ((gear >> 8));
      stmp[4] = temp8bit;
      temp8bit = (gear);
      stmp[5] = temp8bit;
    }
    else
    {
      for (count = 0; count < 8; count++)
      {
        stmp[count] = random(0, 255);
      }
    }
    CAN0.sendMsgBuf((0x600 + n), 1, 8, stmp);
  }

  if (millis() - lastRpmUpdate > 10)
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
      if (rpm >= 50000 || rpm <= 0)
      {
        upDown = 1;
        rpm = 0;
      }
    }
    gear = random(1, 8);
  }
  if (n >= 20)
  {
    // Serial.println("20 cycles");
    n = 0;
  }
  else
  {
    n++;
  }
}

void generateStaticFrames()
{
  byte stmp[8] = { 0 };

  uint16_t staticRPM = 12000;
  uint16_t staticEOT = 100;
  uint16_t staticVBAT = 13000;
  int16_t staticMAP1 = 20;
  static unsigned long lastPacket = 0;
  static const unsigned long deltaT = 2000;
  static byte temp8bit = 0;

  if (millis() - lastPacket > deltaT)
  {
    lastPacket = millis();
    temp8bit = ((staticRPM >> 8));
    stmp[0] = temp8bit;
    temp8bit = (staticRPM);
    stmp[1] = temp8bit;

    temp8bit = ((staticEOT >> 8));
    stmp[2] = temp8bit;
    temp8bit = (staticEOT);
    stmp[3] = temp8bit;

    temp8bit = ((staticVBAT >> 8));
    stmp[4] = temp8bit;
    temp8bit = (staticVBAT);
    stmp[5] = temp8bit;

    temp8bit = ((staticMAP1 >> 8));
    stmp[6] = temp8bit;
    temp8bit = (staticMAP1);
    stmp[7] = temp8bit;

    CAN0.sendMsgBuf(0x600, 1, 8, stmp);
  }
}