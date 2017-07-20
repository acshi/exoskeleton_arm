#include <SoftwareSerial.h>
#include <AltSoftSerial.h>
#include <RoboClaw.h>
#include <Servo.h>
#include <Streaming.h>
#include <Wire.h>
#include "controller53.h"

#define FRAME_START_BYTE 0xed
#define FRAME_END_BYTE 0xec
#define READ_TIMEOUT_MICROS 100000

#define N_JOINTS 4

// motor power calculated as max(0, force f - FORCE_THRESHOLD) * FORCE_MULT >> FORCE_SHIFT
#define FORCE_THRESHOLD 120
#define FORCE_MULT 3
#define FORCE_SHIFT 6

#define LOWER_ARM_I 0
#define UPPER_ARM_I 1
#define SHOLDER_SIDE_I 2
#define SHOLDER_ROTATE_I 3

byte motorMinMove[N_JOINTS] = {22, 18, 18, 18};
byte motorProportion[N_JOINTS] = {22, 1, 1, 1}; // in 128ths
int16_t motorDerivative[N_JOINTS] = {220, 1, 1, 1}; // in 128ths
int16_t lastErrors[N_JOINTS];

byte motorAddrs[N_JOINTS] = {0x80, 17, 18, 19};
byte ratchetServoPinsF[N_JOINTS] = {38, 40, 42, 44};
byte ratchetServoPinsB[N_JOINTS] = {39, 41, 43, 45};

byte servoClosedF[N_JOINTS] = {40, 45, 35, 35};
byte servoOpenF[N_JOINTS] = {80, 65, 50, 50};
byte servoClosedB[N_JOINTS] = {130, 60, 45, 45};
byte servoOpenB[N_JOINTS] = {96, 40, 35, 35};

byte servoStatusPins[N_JOINTS * 2] = {22, 24, 26, 28, 23, 25, 27, 29};
bool servoStatusOpen[N_JOINTS * 2] = {HIGH, LOW, LOW, HIGH, LOW, HIGH, LOW, HIGH};

byte potPins[N_JOINTS] = {4, 5, 6, 7};
uint16_t potMins[N_JOINTS] = {930, 2, 2, 2}; // 40 40
uint16_t potMaxs[N_JOINTS] = {2050, 5000, 5000, 5000};

#define REMOTE_LOWER_ARM_MAP_MIN 810
#define REMOTE_LOWER_ARM_MAP_MAX 500

#define POT_DEADBAND_PERCENT 10
#define POT_DEADBAND_MIN 50
#define POT_DEADBAND_MAX 40

// how much beyond the pot limits the pot may be and we still attempt a return
// beyond this, we just disable the motor
#define POT_OK_MIN_EXT 110
#define POT_OK_MAX_EXT 80

#define WIGGLE_MS 250

#define MAX_RETRACT 22

#define BOUND_HOLD_N_INCREMENTS 10
#define BOUND_HOLD_INCREMENT_MS 1000
#define BOUND_HOLD_INCREMENT 2

// forwards (F) and backwards (B) ratchet servos
Servo lowerArmF;
Servo lowerArmB;
Servo upperArmF;
Servo upperArmB;
Servo sholderSideF;
Servo sholderSideB;
Servo sholderRotateF;
Servo sholderRotateB;
Servo ratchetServosF[] = {lowerArmF, upperArmF, sholderSideF, sholderRotateF};
Servo ratchetServosB[] = {lowerArmB, upperArmB, sholderSideB, sholderRotateB};

#define RATCHET_WAIT_MS 400
#define RATCHET_POWER_N_INCREMENTS 3
#define RATCHET_POWER_INCREMENT 1

RoboClaw roboclaw(&Serial1, 10000); // timeout=10ms

Servo lowerArmMotor1;
Servo lowerArmMotor2;
Servo upperArmMotor1;
Servo upperArmMotor2;
Servo sholderSideMotor1;
Servo sholderSideMotor2;
Servo sholderRotateMotor1;
Servo sholderRotateMotor2;

Servo motorServos[] = {lowerArmMotor1, lowerArmMotor2, upperArmMotor1, upperArmMotor2, sholderSideMotor1, sholderSideMotor2, sholderRotateMotor1, sholderRotateMotor2};
byte motorServoPins[] = {30, 31, 32, 33, 34, 35, 36, 37};

//SoftwareSerial bluetooth(48, 46); // Rx(48), Tx(46)
#define bluetooth Serial2

bool useRemoteMode = false;

uint16_t remoteLowerArmVal = 0;

#define N_ANALOG_VALS 13
uint16_t analogVals[N_ANALOG_VALS];
uint16_t rawAnalogVals[N_ANALOG_VALS];

int16_t lastMotorVals[4];
int16_t motorVals[4];

// ADC prescalers
// from http://www.microsmart.co.za/technical/2014/03/01/advanced-arduino-adc/
#define PS_16 (char)(1 << ADPS2)
#define PS_32 (char)((1 << ADPS2) | (1 << ADPS0))
#define PS_64 (char)((1 << ADPS2) | (1 << ADPS1))
#define PS_128 (char)((1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0))

void setup() {
  bluetooth.begin(9600);

  Wire.begin();
  // slow clock from 100kHz to 40kHz
  // hopefully help with noise
  Wire.setClock(40000);
  Serial.begin(115200);
  roboclaw.begin(115200);

  for (uint8_t i = 0; i < N_JOINTS; i++) {
    ratchetServosF[i].attach(ratchetServoPinsF[i]);
    ratchetServosB[i].attach(ratchetServoPinsB[i]);
    ratchetServosF[i].write(servoClosedF[i]);
    ratchetServosB[i].write(servoClosedB[i]);
  }

  for (uint8_t i = 0; i < N_JOINTS * 2; i++) {
    motorServos[i].attach(motorServoPins[i]);
  }

  // Arduino ADC default prescaler is PS_128. Unset that and set our own.
  ADCSRA &= ~PS_128;
  ADCSRA |= PS_64;

  Serial << "Ready!\n";
}

void setMotorTo(byte motorI, int16_t motorVal) {
  bool useRoboclaw = false;
  bool usePwmControl = true;
  if (useRoboclaw) {
    motorVal = constrain(motorVal, -128, 128);
    if (motorVal < 0) {
      roboclaw.ForwardM1(motorAddrs[motorI], -motorVal);
      roboclaw.ForwardM2(motorAddrs[motorI], -motorVal);
    } else {
      roboclaw.BackwardM1(motorAddrs[motorI], motorVal);
      roboclaw.BackwardM2(motorAddrs[motorI], motorVal);
    }
  } else if (usePwmControl) {
    motorVal = constrain(motorVal, -128, 128);
    uint16_t pulseWidthMicrosA = 1500 + ((int32_t)motorVal * 500 >> 7);
    uint16_t pulseWidthMicrosB = 1500 - ((int32_t)motorVal * 500 >> 7);
    //Serial << "Writing " << pulseWidthMicros << "us to motors with val " << motorVal << " and analog vals " << analogVals[2] << " and " << analogVals[3] << endl;
    motorServos[motorI * 2].write(pulseWidthMicrosA);
    motorServos[motorI * 2 + 1].write(pulseWidthMicrosB);
  } else {
    motorVal = constrain(motorVal, -255, 255);
    set53Motor(motorVal, motorI);
  }
}

// only valid if digitalRead(servoPin) == HIGH.
bool readRatchetOpenNonblock(uint8_t statusI) {
  return digitalRead(servoStatusPins[statusI]) == servoStatusOpen[statusI];
}

// The status pin originating from the servo's pot voltage only holds
// a reliable signal when the motor is not on. The motor turns on after reading
// the PWM pulse width, so while the pulse is high, the value is stable.
bool readRatchetOpen(uint8_t statusI, uint8_t servoPin) {
  while (digitalRead(servoPin) == LOW) { }
  return readRatchetOpenNonblock(statusI);
}

// Open one ratchet so we can move in the direction we want to, based on motorVal
void manageRatchetFor(byte motorI) {
  Servo &servoF = ratchetServosF[motorI];
  Servo &servoB = ratchetServosB[motorI];
  
  const int16_t motorVal = motorVals[motorI];
  
  if (motorVal == 0) {
    servoF.write(servoClosedF[motorI]);
    servoB.write(servoClosedB[motorI]);
    Serial << motorI << ": Closing the ratchets.\n";
    return;
  }
  
  if ((motorVal < 0 && servoF.read() == servoOpenF[motorI]) ||
      (motorVal > 0 && servoB.read() == servoOpenB[motorI])) {
    return;
  }

  if (motorVal < 0) {
    servoF.write(servoOpenF[motorI]);
  } else {
    servoB.write(servoOpenB[motorI]);
  }
  
  byte forwardStatusI = motorI * 2 + 1;
  byte backwardStatusI = motorI * 2;
  bool forwardOpen = false;
  bool backwardOpen = false;

  uint32_t startMillis = millis();
  while (millis() - startMillis <= RATCHET_WAIT_MS) {
    forwardOpen = readRatchetOpen(forwardStatusI, ratchetServoPinsF[motorI]);//robustDigitalRead(;
    backwardOpen = readRatchetOpen(backwardStatusI, ratchetServoPinsB[motorI]);//robustDigitalRead(servoStatusPins[backwardStatusI]) == servoStatusOpen[backwardStatusI];
    //Serial << motorI << "F:" << (forwardOpen ? "open" : "closed") << (servoStatusOpen[forwardStatusI] == LOW ? "(inverted)" : "") << ", ";
    //Serial << "B:" << (backwardOpen ? "open" : "closed") << (servoStatusOpen[backwardStatusI] == LOW ? "(inverted)" : "") << endl;
    if ((motorVal < 0 && forwardOpen) ||
        (motorVal > 0 && backwardOpen)) {
      Serial << motorI << ": Opened " << ((motorVal > 0) ? "backward" : "forward") << " ratchet without resistance.\n";
      return;
    }
  }

  int16_t ratchetMotorVal = 0;
  if (motorVal < 0) {
    ratchetMotorVal = motorMinMove[motorI];
    Serial << motorI << ": Moving motor backward to open forward ratchet.\n";
  } else {
    ratchetMotorVal = -motorMinMove[motorI];
    Serial << motorI << ": Moving motor forward to open backward ratchet.\n";
  }
  setMotorTo(motorI, ratchetMotorVal);
  // disable all other motors since we are blocking here...
  for (uint8_t i = 0; i < N_JOINTS; i++) {
    if (i != motorI) {
      setMotorTo(i, 0);
    }
  }

  bool firstLoop = true;
  byte incrementCount = 0;
  while(true) {
    if (!firstLoop && incrementCount < RATCHET_POWER_N_INCREMENTS) {
      incrementCount++;
      if (motorVal > 0) {
        ratchetMotorVal -= RATCHET_POWER_INCREMENT;
      } else {
        ratchetMotorVal += RATCHET_POWER_INCREMENT;
      }
      setMotorTo(motorI, ratchetMotorVal);
      Serial << motorI << ": ratchet not yet released... upping motor to: " << ratchetMotorVal << endl;
    }
    
    startMillis = millis();
    while (millis() - startMillis <= RATCHET_WAIT_MS) {
      forwardOpen = readRatchetOpen(forwardStatusI, ratchetServoPinsF[motorI]);
      backwardOpen = readRatchetOpen(backwardStatusI, ratchetServoPinsB[motorI]);
      if ((motorVal < 0 && forwardOpen) ||
          (motorVal > 0 && backwardOpen)) {
        goto exit_loops;
      }
    }
    firstLoop = false;
  }
exit_loops:
  
  if (motorVal < 0) {
    Serial << motorI << ": finished opening forward ratchet; closing the backward ratchet.\n";
    servoB.write(servoClosedB[motorI]);
  } else {
    Serial << motorI << ": finished opening backward ratchet; closing the forward ratchet.\n";
    servoF.write(servoClosedF[motorI]);
  }

  setMotorTo(motorI, 0);
}

int16_t boundsManageFor(byte motorI, int16_t motorVal, uint16_t potVal) {
  static int8_t holdOn[N_JOINTS];
  static uint32_t holdStartMs[N_JOINTS];
  
  uint16_t potMin = potMins[motorI];
  uint16_t potMax = potMaxs[motorI];
  
  if (potVal < potMin - POT_OK_MIN_EXT || potVal > potMax + POT_OK_MAX_EXT) {
    return 0; // disable for safety
  }

  uint16_t potMinH = potMin + POT_DEADBAND_MIN;//POT_DEADBAND_PERCENT * potMin / 100;
  uint16_t potMaxL = potMax - POT_DEADBAND_MAX;//POT_DEADBAND_PERCENT * potMax / 100;
  uint16_t additionalPower = 0;

  if (holdOn[motorI]) {
    uint16_t incrementsPassed = (millis() - holdStartMs[motorI]) / BOUND_HOLD_INCREMENT_MS;
    additionalPower = min(BOUND_HOLD_N_INCREMENTS, incrementsPassed) * BOUND_HOLD_INCREMENT;
  }
  
  if (potVal < potMin || (holdOn[motorI] > 0 && potVal < potMinH)) {
    // attempt a slow return to within the correct bounds
    if (!holdOn[motorI]) {
      holdOn[motorI] = 1;
      holdStartMs[motorI] = millis();
      Serial << motorI << ": Placing a bounds hold until we get back up to " << potMinH << " from " << potVal << endl;
    }
    motorVal = motorMinMove[motorI] + additionalPower;
  } else if (potVal > potMax || (holdOn[motorI] < 0 && potVal > potMaxL)) {
    if (!holdOn[motorI]) {
      holdOn[motorI] = -1;
      holdStartMs[motorI] = millis();
      Serial << motorI << ": Placing a bounds hold until we get back down to " << potMaxL << " from " << potVal << endl;
    }
    motorVal = -motorMinMove[motorI] - additionalPower;
  } else if ((motorVal < 0 && potVal < potMinH) || (motorVal > 0 && potVal > potMaxL)) {
    motorVal = 0;
    if (holdOn[motorI]) {
      Serial << motorI << ": removed hold\n";
    }
    holdOn[motorI] = 0;
  } else {
    if (holdOn[motorI]) {
      Serial << motorI << ": removed hold\n";
    }
    holdOn[motorI] = 0;
  }
  return motorVal;
}

uint16_t readVirtual11BitAdc(uint8_t pin) {
    analogRead(pin);
    uint16_t sum = 0;
    for (uint8_t i = 0; i < 4; i++) {
        sum += analogRead(pin);
    }
    return sum >> 1;
}

void bulkAnalogRead() {
  static bool initialized = false;
  static uint32_t lastReadMicros;
  uint32_t nowMicros = micros();

  uint16_t tareValue = readVirtual11BitAdc(N_ANALOG_VALS - 1);
  analogVals[N_ANALOG_VALS - 1] = tareValue;

  if (!initialized) {
    initialized = true;
    for (uint8_t j = 0; j < N_ANALOG_VALS - 1; j++) {
      uint16_t newVal = readVirtual11BitAdc(j);
      if (newVal >= tareValue) {
        newVal -= tareValue;
      } else {
        newVal = 0;
      }
      analogVals[j] = newVal;
      rawAnalogVals[j] = newVal;
    }
  } else {
    for (uint8_t j = 0; j < N_ANALOG_VALS - 1; j++) {
      uint16_t newVal = readVirtual11BitAdc(j);
      if (newVal >= tareValue) {
        newVal -= tareValue;
      } else {
        newVal = 0;
      }
      rawAnalogVals[j] = newVal;
      
      int32_t microsPassed = (int32_t)nowMicros - lastReadMicros;
      if (microsPassed >= 769) {
        analogVals[j] = ((uint32_t)analogVals[j] * 26 + (newVal * 6)) >> 5;
      } else if (microsPassed >= 500) {
        analogVals[j] = ((uint32_t)analogVals[j] * 28 + (newVal * 4)) >> 5;
      } else {
        analogVals[j] = ((uint32_t)analogVals[j] * 30 + (newVal * 2)) >> 5;
      }
    }
  }

  lastReadMicros = nowMicros;
}

void forceBasedControl() {
  /*int lowerLeft = analogVals[0];
  int lowerRight = analogVals[1];*/
  int lowerTop = analogVals[2];
  int lowerBottom = analogVals[3];

  /*int upperLeft = analogVals[8];
  int upperRight = analogVals[9];
  int upperTop = analogVals[10];
  int upperBottom = analogVals[11];*/

  /*int lowerArmMotorVal = 0;
  int upperArmMotorVal = 0;
  int sholderSideMotorVal = 0;
  int sholderRotateMotorVal = 0;*/

  int16_t motorVal;
  int lowerTopBottomDiff = lowerTop - lowerBottom;
  if (lowerTopBottomDiff > FORCE_THRESHOLD) {
    motorVal = (lowerTopBottomDiff - FORCE_THRESHOLD) * FORCE_MULT >> FORCE_SHIFT;
  } else if (lowerTopBottomDiff < -FORCE_THRESHOLD) {
    motorVal = (lowerTopBottomDiff + FORCE_THRESHOLD) * FORCE_MULT >> FORCE_SHIFT;
  } else {
    motorVals[LOWER_ARM_I] = 0;
    return;
  }

  uint16_t potRange = potMaxs[LOWER_ARM_I] - potMins[LOWER_ARM_I];
  uint16_t potVal = analogVals[potPins[LOWER_ARM_I]];
  
  uint16_t distFromEnd;
  if (lowerTopBottomDiff < 0) {
    distFromEnd = potVal - potMins[LOWER_ARM_I];
  } else {
    distFromEnd = potMaxs[LOWER_ARM_I] - potVal;
  }
  
  if (distFromEnd < potRange / 3) {
    motorVal = (int32_t)motorVal * 2 * (int32_t)distFromEnd / potRange;
  }
  if (lowerTopBottomDiff < 0) {
    motorVal = min(-motorMinMove[LOWER_ARM_I], motorVal);
  } else {
    motorVal = max(motorMinMove[LOWER_ARM_I], motorVal);
  }
  motorVals[LOWER_ARM_I] = motorVal;
}

void remoteBasedControl(bool shouldReport) {
  static bool inMotion = false;
  static int8_t motionDirection;
  static uint32_t lastMotorUpdateMicros;
  static uint16_t lastRawPotVal;

  uint16_t constrainedRemoteVal;
  if (REMOTE_LOWER_ARM_MAP_MIN < REMOTE_LOWER_ARM_MAP_MAX) {
    constrainedRemoteVal = constrain(remoteLowerArmVal, REMOTE_LOWER_ARM_MAP_MIN, REMOTE_LOWER_ARM_MAP_MAX);
  } else {
    constrainedRemoteVal = constrain(remoteLowerArmVal, REMOTE_LOWER_ARM_MAP_MAX, REMOTE_LOWER_ARM_MAP_MIN);
  }
  
  uint16_t desiredPotVal = map(constrainedRemoteVal, REMOTE_LOWER_ARM_MAP_MIN, REMOTE_LOWER_ARM_MAP_MAX,
                               potMins[LOWER_ARM_I] + POT_DEADBAND_MIN, potMaxs[LOWER_ARM_I] - POT_DEADBAND_MAX);

  uint16_t currentPotVal = analogVals[potPins[LOWER_ARM_I]];

  bool closeEnough = abs((int16_t)currentPotVal - (int16_t)desiredPotVal) <= (POT_DEADBAND_PERCENT * desiredPotVal) / 100;

  if (!inMotion && closeEnough) {
    return;
  }
  
  if (!inMotion) {
    if (desiredPotVal > currentPotVal) {
      motionDirection = 1;
    } else {
      motionDirection = -1;
    }
    inMotion = true;
    lastRawPotVal = rawAnalogVals[potPins[LOWER_ARM_I]];
    Serial << "Starting action from " << currentPotVal << " to " << desiredPotVal << "\n";
  }
  
  uint32_t nowMicros = micros();
  if (nowMicros - lastMotorUpdateMicros > 5e3) {
    int16_t newMotorVal = 0;
    int16_t error = (int16_t)desiredPotVal - (int16_t)currentPotVal;//(int16_t)rawAnalogVals[potPins[LOWER_ARM_I]];
    if (!closeEnough) {
      newMotorVal = error * motorProportion[LOWER_ARM_I] >> 7;
      if (newMotorVal < 0) {
        newMotorVal -= motorMinMove[LOWER_ARM_I];
        newMotorVal = max(-128, newMotorVal);
      } else {
        newMotorVal += motorMinMove[LOWER_ARM_I];
        newMotorVal = min(128, newMotorVal);
      }

      int16_t derivative = (int32_t)(error - lastErrors[LOWER_ARM_I]) * (int32_t)motorDerivative[LOWER_ARM_I] >> 7;
      if ((newMotorVal > 0 && derivative < -newMotorVal) ||
          (newMotorVal < 0 && derivative > -newMotorVal)) {
        Serial << "limited " << derivative << " to " << -newMotorVal << endl;
        derivative = -newMotorVal;
      }
      newMotorVal += derivative;
      Serial << "goal: " << desiredPotVal << " current: " << currentPotVal << " error: " << error << " delta error: " << (error - lastErrors[LOWER_ARM_I]) << " dTerm: " << derivative  << " total: " << newMotorVal << endl;
    } else {
      Serial << "close w/ goal: " << desiredPotVal << " current: " << currentPotVal << " error: " << error << " delta error: " << (error - lastErrors[LOWER_ARM_I])  << " total: " << newMotorVal << endl;
    }
    lastErrors[LOWER_ARM_I] = error;

    int16_t lastMotorVal = lastMotorVals[LOWER_ARM_I];
    // break through dynamic friction
    if (lastMotorVal == 0) {
      if (newMotorVal > 0) {
        lastMotorVal = motorMinMove[LOWER_ARM_I] >> 1;
      } else {
        lastMotorVal = -motorMinMove[LOWER_ARM_I] >> 1;
      }
    }
    
    int8_t convergenceAdjustment = 0;
    if (newMotorVal > lastMotorVal) {
      convergenceAdjustment = 31;
    } else if (newMotorVal < lastMotorVal) {
      convergenceAdjustment = -31;
    }
    motorVals[LOWER_ARM_I] = (lastMotorVal * 30 + (newMotorVal << 1) + convergenceAdjustment) >> 5;
    lastMotorUpdateMicros = nowMicros;
  } else {
    motorVals[LOWER_ARM_I] = lastMotorVals[LOWER_ARM_I];
  }

  if (closeEnough && motorVals[LOWER_ARM_I] == 0) {
    inMotion = false;
    Serial << "Target location reached.\n";
    return;
  }

  if (shouldReport) {
    Serial << "Target pot val: " << desiredPotVal << " Last motor val: " << lastMotorVals[LOWER_ARM_I] << " ";
  }
}

int16_t timedRead(Stream &stream) {
  uint32_t startMicros = micros();
  while (!stream.available() && (micros() - startMicros) < READ_TIMEOUT_MICROS) { }
  if (stream.available()) {
    //Serial << "got byte after " << (micros() - startMicros) << endl;
    return stream.read();
  }
  Serial << "Timed Out!!\n";
  return -1;
}

void handleBluetooth() {
  while (Serial.available()) {
    char c = Serial.read();
    bluetooth << c;
  }
  while (bluetooth.available()) {
    byte start = bluetooth.read();
    if (start != FRAME_START_BYTE) {
      if (start != 0) {
        Serial << "ignore bluetooth val: " << (char)start << " or " << (byte)start << endl;
      }
      continue;
    }
    
    uint16_t val1 = (timedRead(bluetooth) << 8) | timedRead(bluetooth);
    uint16_t val2 = (timedRead(bluetooth) << 8) | timedRead(bluetooth);
    if (val1 != val2) {
      Serial << "bluetooth vals did not match: " << val1 << " != " << val2 << endl;
      continue;
    }
    byte endByte = timedRead(bluetooth);
    if (endByte != FRAME_END_BYTE) {
      Serial << "bluetooth frame end failed. instead got: " << endByte << endl;
      Serial << "had just gotten vals: " << val1 << " and " << val2 << endl;
      continue;
    }

    remoteLowerArmVal = val1;
    //Serial << remoteLowerArmVal << endl;
    if (!useRemoteMode && remoteLowerArmVal != 0) {
      useRemoteMode = true;
      Serial << "Remote control mode enabled.\n";
    }
  }
}

void loop() {
  // to control Serial output
  static uint16_t loopIter = 0;
  
  static uint32_t lastReportStartMicros = 0;
  uint32_t startMicros = micros();

  bool shouldReport = startMicros - lastReportStartMicros > 100e3L;

  handleBluetooth();
  
  bulkAnalogRead();

  /*int lowerLeft = analogVals[0];
  int lowerRight = analogVals[1];*/
  int lowerTop = analogVals[2];
  int lowerBottom = analogVals[3];

  for (uint8_t i = 0; i < N_JOINTS; i++) {
    motorVals[i] = 0;
  }
  
  if (useRemoteMode) {
    remoteBasedControl(shouldReport);
  } else {
    forceBasedControl();
  }

  if (shouldReport) {
    Serial << "From control: " << motorVals[0] << " ";
  }

  for (uint8_t i = 0; i < N_JOINTS; i++) {
    motorVals[i] = boundsManageFor(i, motorVals[i], analogVals[potPins[i]]);
    if (shouldReport && i == 0) {
      Serial << "Bounds managed: " << motorVals[i] << " ";
    }
    
    if (motorVals[i] != lastMotorVals[i]) {
      lastMotorVals[i] = motorVals[i];
      
      manageRatchetFor(i);
      setMotorTo(i, motorVals[i]);
    }
  }

  static bool fRatchetOpen[N_JOINTS];
  static bool bRatchetOpen[N_JOINTS];
  for (uint8_t i = 0; i < N_JOINTS; i++) {
    if (digitalRead(ratchetServoPinsF[i]) == HIGH) {
      fRatchetOpen[i] = readRatchetOpenNonblock(i * 2 + 1);
    }
    if (digitalRead(ratchetServoPinsB[i]) == HIGH) {
      bRatchetOpen[i] = readRatchetOpenNonblock(i * 2);
    }
  }

  if (shouldReport) {
    Serial << "Remote val: " << remoteLowerArmVal << " ";
    Serial << "Top Force: " << lowerTop << " Bottom Force: " << lowerBottom;// << " Pot: " << lowerArmPot;
    Serial << " Pots: ";
    for (uint8_t i = 0; i < N_JOINTS; i++) {
      Serial << analogVals[potPins[i]] << " ";
    }
    Serial << "Tare: " << analogVals[12] << " ";
    Serial << " Motors: ";
    for (uint8_t i = 0; i < N_JOINTS; i++) {
      Serial << motorVals[i] << " ";
    }
    Serial << " Ratchets: ";
    for (uint8_t i = 0; i < 1; i++) {
      Serial << i << "F:" << (fRatchetOpen[i] ? "open" : "closed") << (servoStatusOpen[i * 2 + 1] == LOW ? "(inverted)" : "") << ", ";
      Serial << "B:" << (bRatchetOpen[i] ? "open" : "closed") << (servoStatusOpen[i * 2] == LOW ? "(inverted)" : "") << ", ";
    }
    Serial << " ... at " << loopIter * 1000000 / (startMicros - lastReportStartMicros) << "hz\n";
    lastReportStartMicros = startMicros;
    loopIter = 0;
  }
  loopIter++;
}
