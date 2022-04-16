#include <SPI.h>
#include <mcp2515.h>
#include <EEPROM.h>

#define DEBUG_MODE 1
// GPIO Inits
#define POWER_EN_PIN 4

// CANBUS Inits
struct can_frame canMsg;
MCP2515 mcp2515(10); // SPI CS Pin D10

// CAN MESSAGES DEFINES
#define CAN_DIRECT_ADDR 0x999
#define CAN_DIRECT_MSG_ON 0x10
#define CAN_DIRECT_MSG_OFF 0x00
#define CAN_SPEED CAN_500KBPS

#define CAN_OFF_PROXY_ADDR_USED 0x00
#define CAN_OFF_PROXY_ADDR_START 0x10
#define CAN_OFF_PROXY_ADDR_END 0x100

#define CAN_ON_PROXY_ADDR_USED 0x100
#define CAN_ON_PROXY_ADDR_START 0x110
#define CAN_ON_PROXY_ADDR_END 0x200

#define LAST_POWER_STATE_ARRD 0x300
#define EEPROM_CAN_DIRECT_ADDR_ADDR 0x310
#define EEPROM_CAN_DIRECT_MSG_ON_ADDR 0x320
#define EEPROM_CAN_DIRECT_MSG_OFF_ADDR 0x330

int lastPowerState = 0;

int NUM_OF_ON_PROXY_MESSAGES = 0;
int NUM_OF_OFF_PROXY_MESSAGES = 0;

uint8_t ON_PROXY_MESSAGES[256] = {};
uint8_t OFF_PROXY_MESSAGES[256] = {};

char buffer[40];

int32_t addr = 0xffffffff;

void updatePowerState(int newPowerState)
{
  if (newPowerState != lastPowerState)
  {
    lastPowerState = newPowerState;
    Serial.println("Setting and saving power rail to " + String(lastPowerState));

    EEPROM.write(LAST_POWER_STATE_ARRD, lastPowerState);
    digitalWrite(POWER_EN_PIN, lastPowerState);
  }
}

void readProxyMessages(int proxyStartAddr, int proxyEndAddr, int proxyMessagesCount, uint8_t proxyMessagesArr[])
{
  proxyMessagesCount = EEPROM.read(proxyStartAddr - 0x10);
  Serial.println("Loaded " + String(proxyMessagesCount) + " proxy messages!");

  for (int i[2] = {proxyStartAddr, 0}; i[0] < proxyEndAddr; i[0] += 0x10, i[1]++)
  {
    if (DEBUG_MODE)
    {
      sprintf(buffer, "Loading from 0x%02X: ", i[0]);
      Serial.print(buffer);
    }
    for (int j = 0; j < 16; j++)
    {
      proxyMessagesArr[i[1] * 10 + j] = EEPROM.read(i[0] + j);
      delay(5);
      if (DEBUG_MODE)
      {
        sprintf(buffer, "0x%02X ", proxyMessagesArr[i[1] * 10 + j]);
        Serial.print(buffer);
      }
    }
    if (DEBUG_MODE)
      Serial.println();
  }
}

uint32_t getProxyMessageAddr(uint8_t messageArr[], int address)
{
  uint32_t addr = 0;
  addr = messageArr[address];
  addr <<= 8;
  addr += messageArr[address + 1];
  addr <<= 8;
  addr += messageArr[address + 2];
  addr <<= 8;
  addr += messageArr[address + 3];

  return addr;
}

void setup()
{
  Serial.begin(115200);
  if (DEBUG_MODE)
    Serial.println("DEBUG MODE TRUE");

  // Setup pin to output, read the state from eeprom and write it to the pin;
  pinMode(POWER_EN_PIN, OUTPUT);
  lastPowerState = EEPROM.read(LAST_POWER_STATE_ARRD);
  digitalWrite(POWER_EN_PIN, lastPowerState);

  // Loop through the EEPROM and read the messages from the ON and OFF proxy;
  // readProxyMessages(CAN_ON_PROXY_ADDR_START, CAN_ON_PROXY_ADDR_END, NUM_OF_ON_PROXY_MESSAGES, ON_PROXY_MESSAGES);
  // readProxyMessages(CAN_OFF_PROXY_ADDR_START, CAN_OFF_PROXY_ADDR_END, NUM_OF_OFF_PROXY_MESSAGES, OFF_PROXY_MESSAGES);

  Serial.println(getProxyMessageAddr(ON_PROXY_MESSAGES, 10), HEX);
  SPI.begin();

  // sprintf(buffer, "ON_PROXY_MESSAGES[0]: %d;", ON_PROXY_MESSAGES[0]);
  // Serial.println(buffer);

  // Serial.println("Hello!");
  // Serial.println("I'm the Power Rail Controller!");

  Serial.print("Last power state: ");
  Serial.println(lastPowerState);

  // // char buffer[40];
  // sprintf(buffer, "I'm reachable at 0x%02X with ON 0x%02X and OFF 0x%02X;\nMy CANBUS Speed is %d;", CAN_DIRECT_ADDR, CAN_DIRECT_MSG_ON, CAN_DIRECT_MSG_OFF, CAN_SPEED);
  // Serial.println(buffer);

  // Serial.println("");
  // Serial.println("https://github.com/xedsvg/451CANPowerRailControl");
  // Serial.println("");

  // Serial.println("Available commands:");
  // Serial.println("");
  // Serial.println("Main commands:");
  // Serial.println("  modifyMainAddress <0x900 - 0x999> - Modify the main address on CAN");
  // Serial.println("  modifyMainCommandPosition <0x01 - 0x08> - Modify the main command position in the CAN message");
  // Serial.println("  modifyMainOnCommand <0x00 - 0xFF> - Modify the main ON command in the CAN message");
  // Serial.println("  modifyMainOffCommand <0x00 - 0xFF> - Modify the main OFF command in the CAN message");
  // Serial.println("");
  // Serial.println("Proxy commands:");
  // Serial.println("  addproxyONmessage <238#DEADBEEF> - Add a proxy ON address to the controller");
  // Serial.println("  addproxyOFFmessage <238#00000000> - Add a proxy OFF message to the controller");

  mcp2515.reset();
  mcp2515.setBitrate(CAN_SPEED, MCP_8MHZ);
  mcp2515.setNormalMode();
}

void loop()
{

  // if (Serial.available() > 0)
  // {
  //   int inByte = Serial.read();

  //   switch (inByte)
  //   {
  //   case 'w':
  //     digitalWrite(2, LOW);
  //     Serial.println("Turned off");
  //     delay(1000);
  //     break;
  //   case 'q':
  //     digitalWrite(2, HIGH);
  //     Serial.println("Turned on");
  //     delay(1000);
  //     break;
  //   default:
  //     Serial.println("No command recognized.");
  //     break;
  //   }
  // }

  if ((mcp2515.readMessage(&canMsg) == MCP2515::ERROR_OK))
  {
    if (canMsg.can_id == CAN_DIRECT_ADDR)
    {
      Serial.println("Direct command recieved");
      if (canMsg.data[0] == CAN_DIRECT_MSG_OFF)
        updatePowerState(0);
      if (canMsg.data[0] == CAN_DIRECT_MSG_ON)
        updatePowerState(1);
    }
    else
    {
      for (int i = 0; i < NUM_OF_ON_PROXY_MESSAGES; i++)
      {
        if (canMsg.can_id == getProxyMessageAddr(ON_PROXY_MESSAGES, i * 10))
        {
          Serial.println("Proxy ON command recieved");
          updatePowerState(1);
        }
      }
      for (int i = 0; i < NUM_OF_OFF_PROXY_MESSAGES; i++)
      {
        if (canMsg.can_id == getProxyMessageAddr(OFF_PROXY_MESSAGES, i * 10))
        {
          Serial.println("Proxy OFF command recieved");
          updatePowerState(0);
        }
      }
    }
  }