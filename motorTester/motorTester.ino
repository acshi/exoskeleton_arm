#include <Streaming.h>

#include <Wire.h>

#define GEAR_BOX_ADDRESS_1 23
#define GEAR_BOX_ADDRESS_2 22

#define SET_RAW_MOTOR_MSG 1
#define READ_RAW_MOTOR_MSG 2
#define SET_CURRENT_LIMIT_MSG 3
#define READ_CURRENT_LIMIT_MSG 4
#define READ_CURRENT_MSG 5
#define DIAGNOSTIC_MSG 11
#define ADDRESS_MSG 12
#define VALUE_TEST_MSG 13

void setup() {
  // put your setup code here, to run once:
  Wire.begin();
  Serial.begin(115200);

  Serial << "Ready!\n";
}

int16_t sendMessage(uint8_t command, int address) {
  Wire.beginTransmission(address);
  Wire.write(command);
  Wire.endTransmission();
  delay(5);
  Wire.requestFrom(address, 2, true);
  return (int16_t)(Wire.read() << 8) | Wire.read();
}

void sendMessageValue(uint8_t command, int16_t value, int address) {
  Wire.beginTransmission(address);
  Wire.write(command);
  Wire.write((value >> 8) & 0xff);
  Wire.write(value & 0xff);
  Wire.endTransmission();
}

void performAction(int16_t action, int address) {
    int16_t value;
    switch (action) {
      case 'sr':
        value = (int16_t)Serial.parseInt();
        Serial << "Setting raw motor output to: " << value << endl;
        sendMessageValue(SET_RAW_MOTOR_MSG, value, address);
        break;
      case 'gr':
        value = sendMessage(READ_RAW_MOTOR_MSG, address);
        Serial << "Raw motor output: " << value << endl;
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
      case 'ts':
        value = sendMessage(VALUE_TEST_MSG, address);
        Serial << "Test Value: " << value << endl;
        break;
      default:
        Serial << "Command not recognized: " << (char)(action >> 8) << (char)action << endl;
        break;
    }
}

void loop() {
  byte byte0 = 0;
  byte byte1 = 0;
  if (Serial.available()) {
    byte b = Serial.read();

    if (b == '\n' || b == '\r') {
      performAction((byte0 << 8) & byte1, GEAR_BOX_ADDRESS_1);
    } else {
      byte0 = byte1;
      byte1 = b;
    }
  }
}
