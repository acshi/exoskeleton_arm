#include "controller53.h"

void flushTwi(int address) {
  // Clear out buffers on both ends of any nonsense...
  while (true) {
    while (Wire.available()) {
      int b = Wire.read();
      if (b != 0xff) {
        //Serial << address << ":1: " << b << endl;
      }
    }
    Wire.requestFrom(address, 2, true);
    int b;
    if (!Wire.available() || (b = Wire.read()) == 0xff) {
      break;
    }
    //Serial << address << ":2: " << b << endl;
    if (!Wire.available() || (b = Wire.read()) == 0xff) {
      break;
    }
    //Serial << address << ":3: " << b << endl;
  }
}

uint16_t send53ReadMessage(uint8_t command, int address) {
  flushTwi(address);

  Wire.beginTransmission(address);
  Wire.write(command);
  Wire.write(command);
  Wire.write(MSG_END_BYTE);
  Wire.endTransmission();
  delayMicroseconds(1000);
  Wire.requestFrom(address, 2, true);
  return (uint16_t)(Wire.read() << 8) | Wire.read();
}

static void singleSendMessage(uint8_t command, int16_t value, int address) {
  Wire.beginTransmission(address);
  Wire.write(command);
  Wire.write(command);
  Wire.write(MSG_END_BYTE);
  Wire.write((value >> 8) & 0xff);
  Wire.write(value & 0xff);
  Wire.write((value >> 8) & 0xff);
  Wire.write(value & 0xff);
  Wire.write(VAL_END_BYTE);
  Wire.endTransmission();
}

void send53MessageValue(uint8_t command, int16_t value, int address) {
  // send several times because noise may interfere
  singleSendMessage(command, value, address);
  singleSendMessage(command, value, address);
  singleSendMessage(command, value, address);
}

