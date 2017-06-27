#include <SoftwareSerial.h>
#include <RoboClaw.h>
#include <Servo.h>
#include <Streaming.h>
#include <Wire.h>
#include "controller53.h"

#define N_JOINTS 4

#define FORCE_THRESHOLD 60

// motor power calculated as (force f - FORCE_THRESHOLD) * FORCE_MULT >> FORCE_SHIFT
#define FORCE_MULT 3
#define FORCE_SHIFT 5

#define LOWER_ARM 16
#define SHOLDER_LIFT 17
#define SHOLDER_SIDE 18
#define SHOLDER_ROTATE 19

#define LOWER_ARM_SERVO_F 46
#define LOWER_ARM_SERVO_B 47
#define SHOLDER_LIFT_SERVO_F 48
#define SHOLDER_LIFT_SERVO_B 49
#define SHOLDER_SIDE_SERVO_F 50
#define SHOLDER_SIDE_SERVO_B 51
#define SHOLDER_ROTATE_SERVO_F 52
#define SHOLDER_ROTATE_SERVO_B 53

#define LOWER_ARM_POT_PIN 4
#define LOWER_ARM_POT_MIN_L 39
#define LOWER_ARM_POT_MIN_H 58
#define LOWER_ARM_POT_MAX_L 382
#define LOWER_ARM_POT_MAX_H 494

#define MIN_MOTOR_VAL 30

// how much beyond the pot limits the pot may be and we still attempt a return
// beyond this, we just disable the motor
#define POT_OK_MIN_EXT 20
#define POT_OK_MAX_EXT 120

// how much the pot should change in value when attempting to wiggle it open
#define POT_WIGGLE 6
#define WIGGLE_MS 250

// forwards (F) and backwards (B) ratchet servos
Servo lowerArmServoF;
Servo lowerArmServoB;

#define LOWER_ARM_SERVO_F_CLOSED 35
#define LOWER_ARM_SERVO_F_OPEN 50
#define LOWER_ARM_SERVO_B_CLOSED 45
#define LOWER_ARM_SERVO_B_OPEN 35

#define manageRatchetLowerArm(motorVal) manageRatchet(lowerArmServoF, lowerArmServoB, \
                                                      LOWER_ARM_SERVO_F_CLOSED, LOWER_ARM_SERVO_F_OPEN, \
                                                      LOWER_ARM_SERVO_B_CLOSED, LOWER_ARM_SERVO_B_OPEN, \
                                                      LOWER_ARM_POT_PIN, LOWER_ARM, motorVal)

#define getBoundsManagedMotorValLowerArm(motorVal, potVal) getBoundsManagedMotorVal(motorVal, potVal, LOWER_ARM_POT_PIN, LOWER_ARM_POT_MIN_L, LOWER_ARM_POT_MIN_H, LOWER_ARM_POT_MAX_L, LOWER_ARM_POT_MAX_H, 0)

// ADC prescalers
// from http://www.microsmart.co.za/technical/2014/03/01/advanced-arduino-adc/
const unsigned char PS_16 = (1 << ADPS2);
const unsigned char PS_32 = (1 << ADPS2) | (1 << ADPS0);
const unsigned char PS_64 = (1 << ADPS2) | (1 << ADPS1);
const unsigned char PS_128 = (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);

void setup() {
  // put your setup code here, to run once:
  Wire.begin();
  // slow clock from 100kHz to 40kHz
  // hopefully help with noise
  Wire.setClock(40000);
  Serial.begin(115200);

  lowerArmServoF.attach(LOWER_ARM_SERVO_F);
  lowerArmServoB.attach(LOWER_ARM_SERVO_B);
  
  lowerArmServoF.write(LOWER_ARM_SERVO_F_CLOSED);
  lowerArmServoB.write(LOWER_ARM_SERVO_B_CLOSED);

  // Arduino ADC default prescaler is PS_128. Unset that and set our own.
  ADCSRA &= ~PS_128;
  ADCSRA |= PS_64;

  Serial << "Ready!\n";
  Serial << "Currently doing a test of the Lower Top/Bottom (Lift) joint only\n";
}

// Open one ratchet so we can move in the direction we want to, based on motorVal
// And then actually give a little power in the opposite direction to take stress off
// of the servo we are trying to open. We block until either a small period of time
// has passed or we notice the shaft has moved a certain amount.
void manageRatchet(Servo &servoF, Servo &servoB, byte closedValF, byte openValF, byte closedValB, byte openValB, byte pot, byte motor, int16_t motorVal) {
  if (motorVal == 0) {
    servoF.write(closedValF);
    servoB.write(closedValB);
    return;
  }

  int startingPotVal = analogRead(pot);
  
  if (motorVal > 0 && servoF.read() == closedValF) {
    servoB.write(closedValB);
    servoF.write(openValF);
    setMotor(-MIN_MOTOR_VAL, motor);
  } else if (motorVal < 0 && servoB.read() == closedValB) {
    servoF.write(closedValF);
    servoB.write(openValB);
    setMotor(MIN_MOTOR_VAL, motor);
  } else {
    return;
  }

  uint32_t startMillis = millis();
  while (millis() - startMillis <= WIGGLE_MS) {
    uint16_t *analogVals = bulkAnalogRead();
    int potVal = analogVals[pot];
    //Serial << "For " << motorVal << " started opening ratchet at " << startingPotVal << " waiting for it to get to " << ((motorVal > 0) ? startingPotVal - POT_WIGGLE : startingPotVal + POT_WIGGLE) << " right now it is at " << potVal << endl;
    if (motorVal > 0 && startingPotVal - POT_WIGGLE >= potVal) {
      break;
    } else if (motorVal < 0 && startingPotVal + POT_WIGGLE <= potVal) {
      break;
    }
  }
}

int16_t getBoundsManagedMotorVal(int16_t motorVal, int16_t potVal, byte pot, int16_t potMinL, int16_t potMinH, int16_t potMaxL, int16_t potMaxH, byte index) {
  static int8_t holdOn[N_JOINTS];

  uint16_t potMin = (potMinL + potMinH) >> 1;
  uint16_t potMax = (potMaxL + potMaxH) >> 1;
  
  if (potVal < potMinL - POT_OK_MIN_EXT || potVal > potMaxH + POT_OK_MAX_EXT) {
    return 0; // disable for safety
  }
  
  if (potVal < potMinL || (holdOn[index] > 0 && potVal < potMin)) {
    // attempt a slow return to within the correct bounds
    motorVal = MIN_MOTOR_VAL;
    holdOn[index] = 1;
  } else if (potVal > potMaxH || (holdOn[index] < 0 && potVal > potMax)) {
    motorVal = -MIN_MOTOR_VAL;
    holdOn[index] = -1;
  } else if ((motorVal < 0 && potVal < potMinH) || (motorVal > 0 && potVal > potMaxL)) {
    motorVal = 0;
    holdOn[index] = 0;
  } else {
    holdOn[index] = 0;
  }
  return motorVal;
}


// median of three
int16_t med3(uint16_t a, uint16_t b, uint16_t c) {
  uint16_t r;
  if ((a <= b && a >= c) || (a <= c && a >= b)) {
    r = a;
  } else if ((b <= c && b >= a) || (b <= a && b >= c)) {
    r = b;
  } else {
    r = c;
  }
  //Serial << "median of three of " << a << ", " << b << ", " << c << " is " << r << endl;
  return r;
}

uint16_t * bulkAnalogRead() {
  static bool initialized = false;
  static uint16_t vals[12 * 3];
  static uint16_t outVals[12];
  
  /*for (uint8_t i = 0; i < 3; i++) {
    for (uint8_t j = 0; j < 12; j++) {
      vals[j * 3 + i] = analogRead(j);
    }
  }*/

  if (!initialized) {
    initialized = true;
    for (uint8_t j = 0; j < 12; j++) {
      outVals[j] = analogRead(j);//med3(vals[j * 3], vals[j * 3 + 1], vals[j * 3 + 2]);
    }
  } else {
    for (uint8_t j = 0; j < 12; j++) {
      uint16_t newVal = analogRead(j);//med3(vals[j * 3], vals[j * 3 + 1], vals[j * 3 + 2]);
      outVals[j] = (outVals[j] * 20 + (newVal * 12)) >> 5;
    }
  }

  return outVals;
}

void loop() {
  // to control Serial output
  static uint8_t loopIter = 0;
  
  static uint32_t lastReportStartMicros = 0;
  uint32_t startMicros = micros();
  
  uint16_t *analogVals = bulkAnalogRead();
  
  int lowerLeft = analogVals[0];
  int lowerRight = analogVals[1];
  int lowerTop = analogVals[2];
  int lowerBottom = analogVals[3];

  int upperLeft = analogVals[8];
  int upperRight = analogVals[9];
  int upperTop = analogVals[10];
  int upperBottom = analogVals[11];

  int lowerArmPot = analogVals[LOWER_ARM_POT_PIN];
  int sholderLiftPot = analogVals[5];
  int sholderSidePot = analogVals[6];
  int sholderRotatePot = analogVals[7];

  int lowerArmMotorVal = 0;
  int sholderLiftMotorVal = 0;
  int sholderSideMotorVal = 0;
  int sholderRotateMotorVal = 0;

  int lowerTopBottomDiff = lowerTop - lowerBottom;
  if (lowerTopBottomDiff > FORCE_THRESHOLD) {
      lowerArmMotorVal = (lowerTopBottomDiff - FORCE_THRESHOLD) * FORCE_MULT >> FORCE_SHIFT;
  } else if (lowerTopBottomDiff < -FORCE_THRESHOLD) {
      lowerArmMotorVal = (lowerTopBottomDiff + FORCE_THRESHOLD) * FORCE_MULT >> FORCE_SHIFT;
  }

  lowerArmMotorVal = getBoundsManagedMotorValLowerArm(lowerArmMotorVal, lowerArmPot);
  manageRatchetLowerArm(lowerArmMotorVal);
  
  setMotor(lowerArmMotorVal, LOWER_ARM);

  if (loopIter % 32 == 0) {
    Serial << "Top Force: " << lowerTop << " Bottom Force: " << lowerBottom << " Pot: " << lowerArmPot;
    Serial << " Motor: " << lowerArmMotorVal << " ... at " << 32 * 1000000 / (startMicros - lastReportStartMicros) << "hz\n";
    lastReportStartMicros = startMicros;
  }
  loopIter++;
}
