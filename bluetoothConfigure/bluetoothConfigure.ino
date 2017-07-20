#include <SoftwareSerial.h>
#include <Streaming.h>

SoftwareSerial bluetooth(8, 9); // Rx, Tx (connect to bluetooth's Tx-O, Rx-I)
//#define bluetooth Serial2

bool onBluetoothA = true;

void setup() {
  Serial.begin(9600);

  bluetooth.begin(9600);
  bluetooth << "$$$";
  delay(100);
  
  Serial << "Ready!\n";
}

void loop() {
  if (Serial.available()) {
    char c = Serial.read();
    bluetooth << c;
  }

  if (bluetooth.available()) {
    Serial << (char)bluetooth.read();
  }
}

