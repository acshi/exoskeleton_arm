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
#define FORCE_THRESHOLD 60
#define FORCE_MULT 3
#define FORCE_SHIFT 5

#define LOWER_ARM_I 0
#define UPPER_ARM_I 1
#define SHOLDER_SIDE_I 2
#define SHOLDER_ROTATE_I 3

#define MIN_MOTOR_VAL 30

byte motorMinMove[N_JOINTS] = {38, 18, 18, 18};
byte motorSectionP[N_JOINTS] = {0, 0, 0, 0};

byte motorAddrs[N_JOINTS] = {0x80, 17, 18, 19};
byte servoPinsF[N_JOINTS] = {38, 40, 42, 44};
byte servoPinsB[N_JOINTS] = {39, 41, 43, 45};
byte potPins[N_JOINTS] = {4, 5, 6, 7};
byte servoClosedF[N_JOINTS] = {145, 45, 35, 35};
byte servoOpenF[N_JOINTS] = {165, 65, 50, 50};
byte servoClosedB[N_JOINTS] = {85, 60, 45, 45};
byte servoOpenB[N_JOINTS] = {65, 40, 35, 35};

byte servoStatusPins[N_JOINTS * 2] = {24, 26, 28, 30, 25, 27, 29, 31};
bool servoStatusInverted[N_JOINTS * 2] = {true, false, true, false, true, false, true, false};

// 213, 241, 
uint16_t lowerArmPotSections[] = {190, 200, 222, 257, 267, 320, 325, 438, 445, 555, 562, 671, 672, 780, 782, 905};
uint16_t upperArmPotSections[] = {39, 42, 58, 60, 74, 77, 91, 96, 108, 112, 125, 131, 143, 146, 164, 171, 182, 192, 198, 204, 216, 224, 242, 252, 286, 315, 393, 437};

uint16_t *potSections[N_JOINTS] = {lowerArmPotSections, upperArmPotSections, 0, 0};
byte numPotSections[N_JOINTS] = {7, 14, 0, 0};

/*#define LOWER_ARM_POT_MIN_L 39
#define LOWER_ARM_POT_MIN_H 58
#define LOWER_ARM_POT_MAX_L 382
#define LOWER_ARM_POT_MAX_H 520*/

#define REMOTE_LOWER_ARM_MAP_MIN 810
#define REMOTE_LOWER_ARM_MAP_MAX 412

// how much beyond the pot limits the pot may be and we still attempt a return
// beyond this, we just disable the motor
#define POT_OK_MIN_EXT 20
#define POT_OK_MAX_EXT 120

#define WIGGLE_MS 250

#define MAX_RETRACT 22

// forwards (F) and backwards (B) ratchet servos
Servo lowerArmServoF;
Servo lowerArmServoB;

//SoftwareSerial roboclawSerial(10, 11); // Rx=10, Tx=11
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

int16_t lastMotorVals[4];
int16_t motorVals[4];

// ADC prescalers
// from http://www.microsmart.co.za/technical/2014/03/01/advanced-arduino-adc/
const unsigned char PS_16 = (1 << ADPS2);
const unsigned char PS_32 = (1 << ADPS2) | (1 << ADPS0);
const unsigned char PS_64 = (1 << ADPS2) | (1 << ADPS1);
const unsigned char PS_128 = (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);

void setup() {
  /*bluetooth.begin(115200);
  bluetooth << "$$$"; // command mode
  bluetooth << "U,9600,N\n"; // temporary change of baud rate for reliability
  bluetooth.begin(9600);
  delay(100);
  bluetooth << "C\n"; // connect to paired device!
  delay(100);
  bluetooth << "---\n"; // in case we were already connected, exit*/
  bluetooth.begin(9600);

  Wire.begin();
  // slow clock from 100kHz to 40kHz
  // hopefully help with noise
  Wire.setClock(40000);
  Serial.begin(115200);
  roboclaw.begin(115200);

  lowerArmServoF.attach(servoPinsF[LOWER_ARM_I]);
  lowerArmServoB.attach(servoPinsB[LOWER_ARM_I]);
  
  lowerArmServoF.write(servoClosedF[LOWER_ARM_I]);
  lowerArmServoB.write(servoClosedB[LOWER_ARM_I]);

  for (uint8_t i = 0; i < N_JOINTS * 2; i++) {
    motorServos[i].attach(motorServoPins[i]);
  }

  // Arduino ADC default prescaler is PS_128. Unset that and set our own.
  ADCSRA &= ~PS_128;
  ADCSRA |= PS_64;

  Serial << "Ready!\n";
}

uint8_t getSection(byte motorI, uint16_t potVal) {
  int nSections = numPotSections[motorI];
  uint16_t *sections = potSections[motorI];
  
  for (uint8_t i = 0; i < nSections; i++) {
    if (potVal >= sections[i * 2] && potVal <= sections[i * 2 + 1]) {
      return i;
    } else if (potVal < sections[i * 2]) {
      if (i > 0 && abs(potVal - sections[i * 2 - 1]) < abs(potVal - sections[i * 2])) {
        return i - 1;
      } else {
        return i;
      }
    }
  }

  return 0;
}

void setMotorTo(byte motorI, int16_t motorVal) {
  bool useRoboclaw = false;
  bool usePwmControl = true;
  if (useRoboclaw) {
    if (motorVal < 0) {
      roboclaw.ForwardM1(motorAddrs[motorI], -motorVal);
      roboclaw.ForwardM2(motorAddrs[motorI], -motorVal);
    } else {
      roboclaw.BackwardM1(motorAddrs[motorI], motorVal);
      roboclaw.BackwardM2(motorAddrs[motorI], motorVal);
    }
  } else if (usePwmControl) {
    uint16_t pulseWidthMicros = 1500 + ((int32_t)motorVal * 500 / 128);
    //Serial << "Writing " << pulseWidthMicros << "us to motors with val " << motorVal << " and analog vals " << analogVals[2] << " and " << analogVals[3] << endl;
    motorServos[motorI * 2].write(pulseWidthMicros);
    motorServos[motorI * 2 + 1].write(pulseWidthMicros);
  } else {
    set53Motor(motorVal, motorI);
  }
}

// Open one ratchet so we can move in the direction we want to, based on motorVal
// And then actually give a little power in the opposite direction to take stress off
// of the servo we are trying to open. We block until either a small period of time
// has passed or we notice the shaft has moved a certain amount.
void manageRatchetFor(byte motorI) {
  Servo &servoF = lowerArmServoF;
  Servo &servoB = lowerArmServoB;
  switch (motorI) {
    case 0:
      servoF = lowerArmServoF;
      servoB = lowerArmServoB;
      break;
    default:
      return;
  }
  
  const int16_t motorVal = motorVals[motorI];
  
  if (motorVal == 0) {
    servoF.write(servoClosedF[motorI]);
    servoB.write(servoClosedB[motorI]);
    Serial << motorI << ": Closing the ratchets.\n";
    return;
  }

  uint16_t startingPotVal = analogRead(potPins[motorI]);
  uint8_t startingSection = getSection(motorI, startingPotVal);
  uint16_t sectionLength = potSections[motorI][startingSection * 2 + 1] - potSections[motorI][startingSection * 2];
  uint16_t retractVal = min(MAX_RETRACT, sectionLength / 2);
  uint16_t targetVal;
  int16_t tempMotorVal;
  
  if (motorVal > 0 && servoF.read() == servoClosedF[motorI]) {
    servoB.write(servoClosedB[motorI]);
    servoF.write(servoOpenF[motorI]);
    tempMotorVal = -MIN_MOTOR_VAL;
    targetVal = potSections[motorI][startingSection * 2 + 1] - retractVal;
    Serial << motorI << ": Opening forward ratchet. In segment " << startingSection << " with target " << targetVal << endl;
  } else if (motorVal < 0 && servoB.read() == servoClosedB[motorI]) {
    servoF.write(servoClosedF[motorI]);
    servoB.write(servoOpenB[motorI]);
    tempMotorVal = MIN_MOTOR_VAL;
    targetVal = potSections[motorI][startingSection * 2] + retractVal;
    Serial << motorI << ": Opening backward ratchet. In segment " << startingSection << " with target " << targetVal << endl;
  } else {
    return;
  }
  bool hasSetMotor = false;

  uint32_t startMillis = millis();
  while (true || millis() - startMillis <= WIGGLE_MS) {
    bulkAnalogRead();
    uint16_t potVal = analogVals[potPins[motorI]];
    if ((millis() - startMillis) % 200 == 0) {
      Serial << motorI << ": For " << motorVal << " started opening ratchet at " << startingPotVal << " with motor at " << tempMotorVal << " waiting for it to get to " << targetVal << " right now it is at " << potVal << endl;
    }
    if (motorVal > 0 && potVal <= targetVal) {
      Serial << motorI << ": Rachet released as we reached a value of " << potVal << " out of our goal of " << targetVal << endl;
      break;
    } else if (motorVal < 0 && potVal >= targetVal) {
      Serial << motorI << ": Rachet released as we reached a value of " << potVal << " out of our goal of " << targetVal << endl;
      break;
    }
    if (!hasSetMotor) {
      hasSetMotor = true;
      setMotorTo(motorI, tempMotorVal);
    }
    //Serial << "r";
  }
  setMotorTo(motorI, 0);

  // because we block, flush the receive buffers of any errors that accumulated
  bluetooth.flush();
}

int16_t boundsManageFor(byte motorI, int16_t motorVal, uint16_t potVal) {
  static int8_t holdOn[N_JOINTS];

  uint16_t potMinL = potSections[motorI][0];
  uint16_t potMinH = potSections[motorI][1];
  uint16_t potMaxL = potSections[motorI][2 * numPotSections[motorI] - 2];
  uint16_t potMaxH = potSections[motorI][2 * numPotSections[motorI] - 1];

  uint16_t potMin = (potMinL + potMinH) >> 1;
  uint16_t potMax = (potMaxL + potMaxH) >> 1;
  
  if (potVal < potMinL - POT_OK_MIN_EXT || potVal > potMaxH + POT_OK_MAX_EXT) {
    return 0; // disable for safety
  }
  
  if (potVal < potMinL || (holdOn[motorI] > 0 && potVal < potMin)) {
    // attempt a slow return to within the correct bounds
    motorVal = MIN_MOTOR_VAL;
    holdOn[motorI] = 1;
  } else if (potVal > potMaxH || (holdOn[motorI] < 0 && potVal > potMax)) {
    motorVal = -MIN_MOTOR_VAL;
    holdOn[motorI] = -1;
  } else if ((motorVal < 0 && potVal < potMinH) || (motorVal > 0 && potVal > potMaxL)) {
    motorVal = 0;
    holdOn[motorI] = 0;
  } else {
    holdOn[motorI] = 0;
  }
  return motorVal;
}

void bulkAnalogRead() {
  static bool initialized = false;
  static uint32_t lastReadMicros;
  uint32_t nowMicros = micros();

  uint16_t tareValue = analogRead(N_ANALOG_VALS - 1);
  analogVals[N_ANALOG_VALS - 1] = tareValue;

  if (!initialized) {
    initialized = true;
    for (uint8_t j = 0; j < N_ANALOG_VALS - 1; j++) {
      uint16_t newVal = analogRead(j);
      if (newVal >= tareValue) {
        newVal -= tareValue;
      } else {
        newVal = 0;
      }
      analogVals[j] = newVal;
    }
  } else {
    for (uint8_t j = 0; j < N_ANALOG_VALS - 1; j++) {
      uint16_t newVal = analogRead(j);
      if (newVal >= tareValue) {
        newVal -= tareValue;
      } else {
        newVal = 0;
      }
      int32_t microsPassed = (int32_t)nowMicros - lastReadMicros;
      if (microsPassed >= 769) {
        analogVals[j] = (analogVals[j] * 26 + (newVal * 6)) >> 5;
      } else if (microsPassed >= 500) {
        analogVals[j] = (analogVals[j] * 28 + (newVal * 4)) >> 5;
      } else {
        analogVals[j] = (analogVals[j] * 30 + (newVal * 2)) >> 5;
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

  int lowerTopBottomDiff = lowerTop - lowerBottom;
  if (lowerTopBottomDiff > FORCE_THRESHOLD) {
    motorVals[LOWER_ARM_I] = (lowerTopBottomDiff - FORCE_THRESHOLD) * FORCE_MULT >> FORCE_SHIFT;
  } else if (lowerTopBottomDiff < -FORCE_THRESHOLD) {
    motorVals[LOWER_ARM_I] = (lowerTopBottomDiff + FORCE_THRESHOLD) * FORCE_MULT >> FORCE_SHIFT;
  } else {
    motorVals[LOWER_ARM_I] = 0;
  }
}

void remoteBasedControl(bool shouldReport) {
  static bool inMotion = false;
  static int8_t motionDirection;
  static uint8_t motionTargetSection;
  static uint16_t motionTargetVal;
  static uint32_t lastMotorUpdateMicros;
  
  int nSections = numPotSections[LOWER_ARM_I];
  uint16_t *sections = potSections[LOWER_ARM_I];

  uint16_t constrainedRemoteVal;
  if (REMOTE_LOWER_ARM_MAP_MIN < REMOTE_LOWER_ARM_MAP_MAX) {
    constrainedRemoteVal = constrain(remoteLowerArmVal, REMOTE_LOWER_ARM_MAP_MIN, REMOTE_LOWER_ARM_MAP_MAX);
  } else {
    constrainedRemoteVal = constrain(remoteLowerArmVal, REMOTE_LOWER_ARM_MAP_MAX, REMOTE_LOWER_ARM_MAP_MIN);
  }
  
  uint16_t desiredPotVal = map(constrainedRemoteVal, REMOTE_LOWER_ARM_MAP_MIN, REMOTE_LOWER_ARM_MAP_MAX,
                               sections[0], sections[nSections * 2 - 1]);

  uint16_t currentPotVal = analogVals[potPins[LOWER_ARM_I]];
  uint8_t targetSection = getSection(LOWER_ARM_I, desiredPotVal);

  uint16_t sectionLow = sections[targetSection * 2];
  uint16_t sectionHigh = sections[targetSection * 2 + 1];
  bool inSection = currentPotVal >= sectionLow && currentPotVal <= sectionHigh;

  if (!inMotion && inSection) {
    return;
  }
  
  bool madeTargetF = motionDirection > 0 && currentPotVal >= motionTargetVal;
  bool madeTargetB = motionDirection < 0 && currentPotVal <= motionTargetVal;
  if (inMotion && inSection && (targetSection == motionTargetSection) && (madeTargetF || madeTargetB)) {
    inMotion = false;
    Serial << "Target section reached.\n";
    return;
  }

  if (!inMotion) {
    if (desiredPotVal > currentPotVal) {
      motionDirection = 1;
    } else {
      motionDirection = -1;
    }
    inMotion = true;
    Serial << "Starting action from " << currentPotVal << " to " << desiredPotVal << " in section " << targetSection << "\n";
  }

  uint16_t sectionLength = sectionHigh - sectionLow;
  if (motionDirection > 0) {
    motionTargetVal = sections[targetSection * 2] + sectionLength / 4;
  } else {
    motionTargetVal = sections[targetSection * 2 + 1] - sectionLength / 4;
  }
  motionTargetSection = targetSection;

  uint8_t currentSection = getSection(LOWER_ARM_I, currentPotVal);

  int16_t newMotorVal = ((int16_t)targetSection - (int16_t)currentSection) * motorSectionP[LOWER_ARM_I];
  if (motionDirection < 0) {
    newMotorVal -= motorMinMove[LOWER_ARM_I];
    newMotorVal = max(-128, newMotorVal);
  } else {
    newMotorVal += motorMinMove[LOWER_ARM_I];
    newMotorVal = min(128, newMotorVal);
  }
  
  uint32_t nowMicros = micros();
  if (nowMicros - lastMotorUpdateMicros > 5000) {
    motorVals[LOWER_ARM_I] = (lastMotorVals[LOWER_ARM_I] * 22 + (newMotorVal * 10)) / 32;
    lastMotorUpdateMicros = nowMicros;
  } else {
    motorVals[LOWER_ARM_I] = lastMotorVals[LOWER_ARM_I];
  }

  if (shouldReport) {
    Serial << "Current section: " << currentSection << " Target section: " << targetSection;
    Serial << " Target pot val: " << motionTargetVal << " New Motor Val: " << newMotorVal << " ";
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
      Serial << "ignore bluetooth val: " << (char)start << " or " << (byte)start << endl;
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

  bool shouldReport = startMicros - lastReportStartMicros > 100L * 1000L;

  handleBluetooth();
  
  bulkAnalogRead();

  /*int lowerLeft = analogVals[0];
  int lowerRight = analogVals[1];*/
  int lowerTop = analogVals[2];
  int lowerBottom = analogVals[3];

  /*int lowerArmPot = analogVals[potPins[LOWER_ARM_I]];
  int upperArmPot = analogVals[5];
  int sholderSidePot = analogVals[6];
  int sholderRotatePot = analogVals[7];*/

  for (uint8_t i = 0; i < N_JOINTS; i++) {
    motorVals[i] = 0;
  }
  
  if (useRemoteMode) {
    remoteBasedControl(shouldReport);
  } else {
    forceBasedControl();
  }

  for (uint8_t i = 0; i < N_JOINTS; i++) {
    motorVals[i] = boundsManageFor(i, motorVals[i], analogVals[potPins[i]]);
    
    if (motorVals[i] != lastMotorVals[i]) {
      lastMotorVals[i] = motorVals[i];
      
      manageRatchetFor(i);
      setMotorTo(i, motorVals[i]);
    }
  }

  if (shouldReport) {
    Serial << "Remote val: " << remoteLowerArmVal << " ";
    Serial << "From control: " << motorVals[LOWER_ARM_I] << " ";
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
    Serial << " ... at " << loopIter * 1000000 / (startMicros - lastReportStartMicros) << "hz\n";
    lastReportStartMicros = startMicros;
    loopIter = 0;
  }
  loopIter++;
}
