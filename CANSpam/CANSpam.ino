/*
// Include libraries
// MCP2515 https://github.com/coryjfowler/MCP2515_lib
*/

#include <mcp_can.h>
#include <SPI.h>

/*
// Defines for MCP2515 CAN controller
*/
#define MCP_CHIP_SELECT 10
#define MCP_INTERRUPT 3

/*
// Variables for MCP2515
*/
long unsigned int rxId;
byte len = 0;
byte txBuf1[8] = { 0 };
byte txBuf2[8] = { 0 };
byte txBuf3[8] = { 0 };
byte txBuf4[8] = { 0 };

int16_t val1 = 0;
int16_t val2 = 16384;
int16_t val3 = -16384;
int16_t val4 = 8192;

// 10 ms delay between packets
const int packetDelay = 1000;

// Set chip select pin
MCP_CAN CAN0(MCP_CHIP_SELECT);

void setup()
{
  Serial.begin(115200);
  /*
  // Initializing the MCP2515
  */
  // Delay needed during startup - Not sure why, but should be OK for now
  delay(500);
  // Initiate CAN to 1000KBPS
  CAN0.begin(CAN_1000KBPS);
  pinMode(MCP_INTERRUPT, INPUT);
}

void loop()
{
  // Goal here is to saturate the bus with 4 frame IDs
  txBuf1[0] = val1 >> 8;
  txBuf1[1] = val1;
  txBuf1[2] = val2 >> 8;
  txBuf1[3] = val2;
  txBuf1[4] = val3 >> 8;
  txBuf1[5] = val3;
  txBuf1[6] = val4 >> 8;
  txBuf1[7] = val4;

  txBuf2[0] = val2 >> 8;
  txBuf2[1] = val2;
  txBuf2[2] = val3 >> 8;
  txBuf2[3] = val3;
  txBuf2[4] = val4 >> 8;
  txBuf2[5] = val4;
  txBuf2[6] = val1 >> 8;
  txBuf2[7] = val1;

  txBuf3[0] = val3 >> 8;
  txBuf3[1] = val3;
  txBuf3[2] = val4 >> 8;
  txBuf3[3] = val4;
  txBuf3[4] = val1 >> 8;
  txBuf3[5] = val1;
  txBuf3[6] = val2 >> 8;
  txBuf3[7] = val2;

  txBuf4[0] = val4 >> 8;
  txBuf4[1] = val4;
  txBuf4[2] = val1 >> 8;
  txBuf4[3] = val1;
  txBuf4[4] = val2 >> 8;
  txBuf4[5] = val2;
  txBuf4[6] = val3 >> 8;
  txBuf4[7] = val3;

  //CAN0.sendMsgBuf(0x100, 1, 8, txBuf1);
  //delay(packetDelay);

  //CAN0.sendMsgBuf(0x101, 1, 8, txBuf2);
  //delay(packetDelay);
  CAN0.sendMsgBuf(0x600, 1, 8, txBuf3);
  Serial.println("Sent 0x100");
  delay(packetDelay);
  //CAN0.sendMsgBuf(0x601, 1, 8, txBuf4);
  //delay(packetDelay);

  ++val1;
  ++val2;
  ++val3;
  ++val4;
}
