#include <Streaming.h>

#include <Wire.h>

#define GEAR_BOX_ADDRESS_1 16
#define GEAR_BOX_ADDRESS_2 22

#define SET_RAW_MOTOR_MSG 1
#define READ_RAW_MOTOR_MSG 2
#define SET_CURRENT_LIMIT_MSG 3
#define READ_CURRENT_LIMIT_MSG 4
#define READ_CURRENT_MSG 5
#define DIAGNOSTIC_MSG 11
#define ADDRESS_MSG 12

void setup() {
  // put your setup code here, to run once:
  Wire.begin();
  Serial.begin(115200);

  Serial << "Ready!\n";
}

uint16_t sendMessage(uint8_t command, int address) {
  Wire.beginTransmission(address);
  Wire.write(command);
  Wire.endTransmission();
  delayMicroseconds(100);
  Wire.requestFrom(address, 2, true);
  return (uint16_t)(Wire.read() << 8) | Wire.read();
}

void sendMessageValue(uint8_t command, int16_t value, int address) {
  Wire.beginTransmission(address);
  Wire.write(command);
  Wire.write((value >> 8) & 0xff);
  Wire.write(value & 0xff);
  Wire.endTransmission();
}

bool performAction(int16_t action, int address) {
    uint16_t value;
    switch (action) {
      case 'sr':
        value = (uint16_t)Serial.parseInt();
        Serial << "Setting raw motor output to: " << (int16_t)value << endl;
        sendMessageValue(SET_RAW_MOTOR_MSG, value, address);
        break;
      case 'gr':
        value = sendMessage(READ_RAW_MOTOR_MSG, address);
        Serial << "Raw motor output: " << (int16_t)value << endl;
        break;
      case 'sl':
        value = (int16_t)Serial.parseInt();
        Serial << "Setting current limit to: " << value << endl;
        sendMessageValue(SET_CURRENT_LIMIT_MSG, value, address);
        break;
      case 'gl':
        value = sendMessage(READ_CURRENT_LIMIT_MSG, address);
        Serial << "Current limit: " << value << endl;
        break;
      case 'cr':
        value = sendMessage(READ_CURRENT_MSG, address);
        Serial << "Current: " << value << endl;
        break;
      case 'dg':
        value = sendMessage(DIAGNOSTIC_MSG, address);
        Serial << "Diagnostic: " << (char)((value >> 8) & 0xff) << (char)(value & 0xff) << endl;
        break;
      case 'tw':
        value = sendMessage(ADDRESS_MSG, address);
        Serial << "TWI Address: " << value << endl;
        break;
      default:
        //Serial << "Command not recognized: " << (char)(action >> 8) << ',' << (char)action << endl;
        return false;
    }
    return true;
}

void loop() {
  static byte byte0 = 0;
  static byte byte1 = 0;
  if (Serial.available()) {
    byte b = Serial.read();

    byte0 = byte1;
    byte1 = b;

    if (byte0 != 0 && byte1 != 0) {
      if (performAction((byte0 << 8) | byte1, GEAR_BOX_ADDRESS_1)) {
        byte0 = 0;
        byte1 = 0;
      }
    }
  }
}
