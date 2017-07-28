#include <Metro.h>
#include <AltSoftSerial.h>
#include <Streaming.h>

#define FRAME_START_BYTE 0xed
#define FRAME_END_BYTE 0xec

Metro reportMetro = Metro(50);
AltSoftSerial bluetooth; // Rx (8), Tx (9) (connect to bluetooth's Tx-O, Rx-I)


#define N_JOINTS 4
uint16_t jointVals[N_JOINTS];

// ADC prescalers
// from http://www.microsmart.co.za/technical/2014/03/01/advanced-arduino-adc/
const unsigned char PS_16 = (1 << ADPS2);
const unsigned char PS_32 = (1 << ADPS2) | (1 << ADPS0);
const unsigned char PS_64 = (1 << ADPS2) | (1 << ADPS1);
const unsigned char PS_128 = (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);

void setup() {
  Serial.begin(9600);
  
  // Arduino ADC default prescaler is PS_128. Unset that and set our own.
  ADCSRA &= ~PS_128;
  ADCSRA |= PS_64;
  
  bluetooth.begin(9600);

  for (uint8_t i = 0; i < N_JOINTS; i++) {
    jointVals[i] = 0xffff;
  }

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

void loop() {
  for (uint8_t i = 0; i < N_JOINTS; i++) {
    if (jointVals[i] == 0xffff) {
      jointVals[i] = analogRead(i);
    } else {
      jointVals[i] = (jointVals[i] * 20 + (analogRead(i) * 12)) >> 5;
    }
  }

  if (reportMetro.check()) {
    sendValFrame(bluetooth, jointVals, N_JOINTS);
    for (uint8_t i = 0; i < N_JOINTS; i++) {
      Serial << jointVals[i] << " ";
    }
    Serial << endl;
  }

  while (bluetooth.available()) {
    Serial << (char)bluetooth.read();
  }

  while (Serial.available()) {
    bluetooth << (char)Serial.read();
  }
}

