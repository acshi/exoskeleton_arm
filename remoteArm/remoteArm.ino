#include <Metro.h>
#include <AltSoftSerial.h>
#include <Streaming.h>

#define FRAME_START_BYTE 0xed
#define FRAME_END_BYTE 0xec

Metro reportMetro = Metro(50);
AltSoftSerial bluetooth; // Rx (8), Tx (9) (connect to bluetooth's Tx-O, Rx-I)

// ADC prescalers
// from http://www.microsmart.co.za/technical/2014/03/01/advanced-arduino-adc/
const unsigned char PS_16 = (1 << ADPS2);
const unsigned char PS_32 = (1 << ADPS2) | (1 << ADPS0);
const unsigned char PS_64 = (1 << ADPS2) | (1 << ADPS1);
const unsigned char PS_128 = (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);

/*void waitForBluetooth() {
  while (!bluetooth.available()) {
    if (Serial.available()) {
      bluetooth << (char)Serial.read();
    }
  }
  while (bluetooth.available()) {
    Serial << (char)bluetooth.read();
    delay(2);
  }
}*/

void setup() {
  Serial.begin(9600);
  
  // Arduino ADC default prescaler is PS_128. Unset that and set our own.
  ADCSRA &= ~PS_128;
  ADCSRA |= PS_64;
  
  bluetooth.begin(9600);

  Serial << "Ready!\n";
}

void sendValFrame(Stream &stream, uint16_t val) {
  stream.write(FRAME_START_BYTE);
  stream.write((val >> 8) & 0xff);
  stream.write(val & 0xff);
  stream.write((val >> 8) & 0xff);
  stream.write(val & 0xff);
  stream.write(FRAME_END_BYTE);
}

void loop() {
  static uint16_t jointVal = 0xffff;
  if (jointVal == 0xffff) {
    jointVal = analogRead(0);
  } else {
    jointVal = (jointVal * 20 + (analogRead(0) * 12)) >> 5;
  }

  if (reportMetro.check()) {
    sendValFrame(bluetooth, jointVal);
    Serial << jointVal << endl;
  }

  while (bluetooth.available()) {
    Serial << (char)bluetooth.read();
  }

  while (Serial.available()) {
    bluetooth << (char)Serial.read();
  }
}

