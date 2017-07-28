#include "controller53.h"

bool flushTwi(int address) {
  bool hasDiscarded = false;
  // Clear out buffers on both ends from any nonsense...
  while (true) {
    while (Wire.available()) {
      int b = Wire.read();
      if (b != 0xff) {
        Serial << address << ":1: " << b << endl;
        hasDiscarded = true;
      }
    }
    Wire.requestFrom(address, 3);
    for (uint8_t i = 0; i < 3; i++) {
      if (!Wire.available()) {
        return hasDiscarded;
      }
      int b = Wire.read();
      if (b == 0xff) {
        return hasDiscarded;
      }
      Serial << address << ":" << (i + 2) << ": " << b << endl;
      hasDiscarded = true;
    }
  }
}

uint16_t send53ReadMessage(uint8_t command, int address) {
  flushTwi(address);

  Wire.beginTransmission(address);
  Wire.write(command);
  Wire.write(command);
  Wire.write(MSG_END_BYTE);
  Wire.endTransmission();

  // Look for the message mark to know the message has been received.
  Wire.requestFrom(address, 1);
  for (uint8_t i = 0; i < 10; i++) {
    int16_t b = Wire.read();
    if (b == MSG_MARK_BYTE) {
      Wire.requestFrom(address, 2);
      break;
    } else {
      delayMicroseconds(500);
      Wire.requestFrom(address, 1);
    }
  }
  
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

