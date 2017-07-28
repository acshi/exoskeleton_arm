#include <AltSoftSerial.h>
#include <Streaming.h>

#define FRAME_START_BYTE 0xed
#define FRAME_END_BYTE 0xec

#define READ_TIMEOUT_MICROS 100000

AltSoftSerial bluetooth; // Rx (8), Tx (9) (connect to bluetooth's Tx-O, Rx-I)

#define N_JOINTS 4
uint16_t jointVals[N_JOINTS];

uint16_t potMins[N_JOINTS] = {930, 2, 2, 2};
uint16_t potMaxs[N_JOINTS] = {2050, 5000, 5000, 5000};

uint16_t remoteMapMins[N_JOINTS] = {810, 0, 0, 0};
uint16_t remoteMapMaxs[N_JOINTS] = {500, 0, 0, 0};

void setup() {
  Serial.begin(9600);
  bluetooth.begin(9600);

  Serial << "Ready!\n";
}

void sendValFrame(Stream &stream, uint16_t *vals, uint8_t numVals) {
  stream.write(FRAME_START_BYTE);
  for (uint8_t i = 0; i < numVals; i++) {
    stream.write((vals[i] >> 8) & 0xff);
    stream.write(vals[i] & 0xff);
    stream.write((vals[i] >> 8) & 0xff);
    stream.write(vals[i] & 0xff);
  }
  stream.write(FRAME_END_BYTE);
}

int16_t timedRead(Stream &stream) {
  uint32_t startMicros = micros();
  while (!stream.available() && (micros() - startMicros) < READ_TIMEOUT_MICROS) { }
  if (stream.available()) {
    return stream.read();
  }
  Serial << "Timed Out!!\n";
  return -1;
}

void attemptBluetoothRead() {
  uint16_t receivedVals[N_JOINTS];
  
  byte start = bluetooth.read();
  if (start != FRAME_START_BYTE) {
    if (start != 0) {
      Serial << "ignore bluetooth val: " << (char)start << " or " << (byte)start << endl;
    }
    return;
  }

  for (uint8_t i = 0; i < N_JOINTS; i++) {
    uint16_t val1 = (timedRead(bluetooth) << 8) | timedRead(bluetooth);
    uint16_t val2 = (timedRead(bluetooth) << 8) | timedRead(bluetooth);
    if (val1 != val2) {
      Serial << "bluetooth vals did not match: " << val1 << " != " << val2 << endl;
      return;
    }
    receivedVals[i] = val1;
  }
  
  byte endByte = timedRead(bluetooth);
  if (endByte != FRAME_END_BYTE) {
    Serial << "bluetooth frame end failed. instead got: " << endByte << endl;
    return;
  }

  Serial << "V";
  for (int8_t i = N_JOINTS - 1; i >= 0; i--) {
    float f = ((float)receivedVals[i] - potMins[i]) / (potMaxs[i] - potMins[i]);
    Serial.print(f, 3);
    Serial << " " ;
    //Serial << "(" << receivedVals[i] << ") ";
  }
  Serial << endl;
}

// Expect values in the form of 0.0 to 1.0 in terms of the total range of the given angle
void attemptSerialRead() {
  byte start = Serial.read();
  if (start != 'V') {
    if (start != 0) {
      //Serial << "ignore serial val: " << (char)start << " or " << (byte)start << endl;
    }
    return;
  }

  for (int8_t i = N_JOINTS - 1; i >= 0; i--) {
    float f = Serial.parseFloat();
    uint16_t val = (uint16_t)(f * ((float)remoteMapMaxs[i] - remoteMapMins[i]) + remoteMapMins[i] + 0.5f);
    jointVals[i] = val;
  }

  sendValFrame(bluetooth, jointVals, N_JOINTS);
  Serial << "sent: ";
   for (uint8_t i = 0; i < N_JOINTS; i++) {
    Serial << jointVals[i] << " ";
  }
  Serial << endl;
}

void loop() {
  while (bluetooth.available()) {
    attemptBluetoothRead();
  }

  while (Serial.available()) {
    attemptSerialRead();
  }
}

