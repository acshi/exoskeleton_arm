#include <SoftwareSerial.h>
#include <Streaming.h>

#define bluetooth Serial2
SoftwareSerial bluetoothA(8, 9); // Rx, Tx (connect to bluetooth's Tx-O, Rx-I)
SoftwareSerial bluetoothB(10, 11); // Rx, Tx (connect to bluetooth's Tx-O, Rx-I)

bool onBluetoothA = true;

void setup() {
  Serial.begin(9600);

  bluetooth.begin(9600);
  bluetooth << "$$$";
  delay(100);

  // configure bluetooth
  /*bluetoothA.begin(115200);
  bluetoothA << "$$$";
  delay(100);
  bluetoothA << "U,9600,N\n"; // temporary change of baud rate
  bluetoothA.begin(9600);

  bluetoothB.begin(115200);
  bluetoothB << "$$$";
  delay(100);
  bluetoothB << "U,9600,N\n"; // temporary change of baud rate
  bluetoothB.begin(9600);
  
  bluetoothA.listen();
  Serial << "Starting with bluetooth A\n";*/
  
  Serial << "Ready!\n";
}

void loop() {
  if (Serial.available()) {
    char c = Serial.read();
    /*if (c == '*') {
      onBluetoothA = !onBluetoothA;
      Serial << "Switched to bluetooth " << (onBluetoothA ? 'A' : 'B') << endl;
      if (onBluetoothA) {
        bluetoothA.listen();
      } else {
        bluetoothB.listen();
      }
    } else {
      if (onBluetoothA) {
        bluetoothA << c;
      } else {
        bluetoothB << c;
      }
    }*/
    bluetooth << c;
  }

  if (bluetooth.available()) {
    Serial << (char)bluetooth.read();
  }
  
  /*if (onBluetoothA) {
    if (bluetoothA.available()) {
      Serial << (char)bluetoothA.read();
    }
  } else {
    if (bluetoothB.available()) {
      Serial << (char)bluetoothB.read();
    }
  }*/
}
