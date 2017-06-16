#include <Streaming.h>

#include <Wire.h>

#define SET_RAW_MOTOR_MSG 1
#define READ_RAW_MOTOR_MSG 2
#define SET_CURRENT_LIMIT_MSG 3
#define READ_CURRENT_LIMIT_MSG 4
#define READ_CURRENT_MSG 5
#define DIAGNOSTIC_MSG 11
#define ADDRESS_MSG 12
#define MSG_END_BYTE 0xed
#define VAL_END_BYTE 0xec

uint8_t selectedAddress = 255; // none selected yet

bool availableAddrs[128];

void flushTwi(int address) {
  // Clear out buffers on both ends from any nonsense...
  while (true) {
    while (Wire.available()) {
      int b = Wire.read();
      if (b != 0xff) {
        Serial << address << ":1: " << b << endl;
      }
    }
    Wire.requestFrom(address, 2, true);
    int b;
    if (!Wire.available() || (b = Wire.read()) == 0xff) {
      break;
    }
    Serial << address << ":2: " << b << endl;
    if (!Wire.available() || (b = Wire.read()) == 0xff) {
      break;
    }
    Serial << address << ":3: " << b << endl;
  }
}

uint16_t sendReadMessage(uint8_t command, int address) {
  if (address == 255) {
    Serial << "Cannot send a message without first selecting a TWI device\n";
    return 0xffff;
  }
  
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

void singleSendMessage(uint8_t command, int16_t value, int address) {
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

void sendMessageValue(uint8_t command, int16_t value, int address) {
  if (address == 255) {
    Serial << "Cannot send a message without first selecting a TWI device\n";
    return;
  }

  // send several times because noise may interfere
  singleSendMessage(command, value, address);
  singleSendMessage(command, value, address);
  singleSendMessage(command, value, address);
}

void scanAddrs() {
  bool foundAny = false;
  for (uint8_t i = 0; i < 128; i++) {
    bool foundAddr = sendReadMessage(DIAGNOSTIC_MSG, i) == 'HI';
    availableAddrs[i] = foundAddr;
    if (foundAddr) {
      foundAny = true;
    }
  }
  if (foundAny) {
    Serial << "Available TWI devices:\n";
    for (uint8_t i = 0; i < 128; i++) {
      if (availableAddrs[i]) {
        Serial << '\t' << i << endl;
        if (selectedAddress == 255) {
          selectedAddress = i;
          Serial << "Selected address " << i << " as default\n";
        }
      }
    }
    Serial << endl;
  } else {
    Serial << "No TWI devices found. Check your wiring and scan again with 'sc'\n";
  }
}

void setup() {
  // put your setup code here, to run once:
  Wire.begin();
  // slow clock from 100kHz to 40kHz
  // hopefully help with noise
  Wire.setClock(40000); 
  Serial.begin(115200);

  Serial << "Ready!\n";
  scanAddrs();
}

bool performAction(int16_t action, int address) {
    uint16_t value;
    switch (action) {
      case 'sm':
        value = (uint16_t)Serial.parseInt();
        Serial << "Setting motor output to: " << (int16_t)value << endl;
        sendMessageValue(SET_RAW_MOTOR_MSG, value, address);
        break;
      case 'gm':
        value = sendReadMessage(READ_RAW_MOTOR_MSG, address);
        Serial << "Motor output: " << (int16_t)value << endl;
        break;
      case 'sl':
        value = (int16_t)Serial.parseInt();
        Serial << "Setting current limit to: " << value << endl;
        sendMessageValue(SET_CURRENT_LIMIT_MSG, value, address);
        break;
      case 'gl':
        value = sendReadMessage(READ_CURRENT_LIMIT_MSG, address);
        Serial << "Current limit: " << value << endl;
        break;
      case 'cr':
        value = sendReadMessage(READ_CURRENT_MSG, address);
        Serial << "Current: " << value << endl; 
        break;
      case 'dg':
        value = sendReadMessage(DIAGNOSTIC_MSG, address);
        Serial << "Diagnostic: " << (char)((value >> 8) & 0xff) << (char)(value & 0xff) << endl;
        break;
      case 'tw':
        value = sendReadMessage(ADDRESS_MSG, address);
        Serial << "TWI Address: " << value << endl;
        break;
      case 'sc':
        scanAddrs();
        break;
      case 'se':
        value = Serial.parseInt();
        if (value > 127) {
          Serial << "Address " << value << " is invalid. TWI addresses may range from 0 to 127.\n";
          break;
        }
        if (true || availableAddrs[value]) {
          selectedAddress = value;
          Serial << "Selected address " << selectedAddress << endl;
        } else {
          Serial << "Address " << value << " is not available. Check your wiring and then rescan with 'sc'\n";
        }
        break;
      case 'fl':
        value = Serial.parseInt();
        if (value > 127) {
          Serial << "Address " << value << " is invalid. TWI addresses may range from 0 to 127.\n";
          break;
        }
        Serial << "Flushing TWI interface at " << value << endl;
        flushTwi(value);
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
      if (performAction((byte0 << 8) | byte1, selectedAddress)) {
        byte0 = 0;
        byte1 = 0;
      }
    }
  }
}
