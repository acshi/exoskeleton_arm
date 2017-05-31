#include <Streaming.h>

#include <Wire.h>

#define GEAR_BOX_ADDRESS_1 23
#define GEAR_BOX_ADDRESS_2 22

#define READ_ANGLE_MSG 1
#define SET_ANGLE_MSG 2
#define READ_SET_ANGLE_MSG 3
#define READ_ENCODER_MSG 4
#define READ_HOMING_MSG 5
#define DIAGNOSTIC_MSG 6
#define ADDRESS_MSG 7
#define VALUE_TEST_MSG 8
#define READ_ADC0_MSG 9
#define READ_ADC1_MSG 10
#define READ_ADC2_MSG 11
#define READ_ADC3_MSG 12
#define READ_ADC4_MSG 13
#define READ_ADC5_MSG 14
#define READ_ADC6_MSG 15
#define READ_ADC7_MSG 16
#define SET_RAW_MOTOR_MSG 17
#define READ_RAW_MOTOR_MSG 18
#define SET_TARGET_SPEED_MSG 19
#define READ_TARGET_SPEED_MSG 20
#define READ_MEASURED_SPEED_MSG 21
#define DO_HOMING_MSG 22

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

void actOnByte(int b, int address) {
    int16_t value;
    switch (b) {
      case 'h':
        value = sendMessage(READ_HOMING_MSG, address);
        Serial << "Homing value: " << value << endl;
        break;
      case 'e':
        value = sendMessage(READ_ENCODER_MSG, address);
        //Serial << "Encoder value: ";
        Serial << value << endl;
        break;
      case 's':
        value = (int16_t)Serial.parseInt();
        Serial << "Setting angle to: " << value / 100.0f << endl;
        sendMessageValue(SET_ANGLE_MSG, value, address);
        break;
      case 'a':
        value = sendMessage(READ_ANGLE_MSG, address);
        if (value == -1) {
          Serial << "Cannot read angle: homing not finished" << endl;
        } else {
          Serial << "Current angle: " << value / 100.0f << endl;
        }
        break;
      case 'c':
        value = sendMessage(READ_SET_ANGLE_MSG, address);
        Serial << "Set angle: " << value << endl;
        break;
      case 'd':
        value = sendMessage(DIAGNOSTIC_MSG, address);
        Serial << "Diagnostic: " << (char)((value >> 8) & 0xff) << (char)(value & 0xff) << endl;
        break;
      case 'w':
        value = sendMessage(ADDRESS_MSG, address);
        Serial << "TWI Address: " << value << endl;
        break;
      case 't':
        value = sendMessage(VALUE_TEST_MSG, address);
        Serial << "Test Value: " << value << endl;
        break;
      case 'r':
        value = (int16_t)Serial.parseInt();
        Serial << "Setting raw motor output to: " << value << endl;
        sendMessageValue(SET_RAW_MOTOR_MSG, value, address);
        break;
      case 'm':
        value = sendMessage(READ_RAW_MOTOR_MSG, address);
        Serial << "Raw motor output value: " << value << endl;
        break;
      case 'p':
        value = (int16_t)Serial.parseInt();
        Serial << "Setting target speed to (100 = 1 degree per second): " << value << endl;
        sendMessageValue(SET_TARGET_SPEED_MSG, value, address);
        break;
    }
    if (b >= '0' && b <= '7') {
        value = sendMessage(b - '0' + READ_ADC0_MSG, address);
        Serial << "ADC " << (char)b << " Value: " << value << endl;
    }
}

void loop() {
 if (Serial.available()) {
    byte b = Serial.read();
    if (b == '\n' || b == '\r') {
      return;
    }
    actOnByte(b, GEAR_BOX_ADDRESS_1);
    //actOnByte(b, GEAR_BOX_ADDRESS_2);
  }
  //actOnByte('e', GEAR_BOX_ADDRESS_1);
}
