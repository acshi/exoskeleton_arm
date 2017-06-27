#include <Arduino.h>
#include <Wire.h>

#define SET_MOTOR_MSG 1
#define READ_MOTOR_MSG 2
#define SET_CURRENT_LIMIT_MSG 3
#define READ_CURRENT_LIMIT_MSG 4
#define READ_CURRENT_MSG 5
#define DIAGNOSTIC_MSG 11
#define ADDRESS_MSG 12
#define MSG_END_BYTE 0xed
#define VAL_END_BYTE 0xec


uint16_t sendReadMessage(uint8_t command, int address);
void sendMessageValue(uint8_t command, int16_t value, int address);
void flushTwi(int address);

#define setMotor(VAL, ADDR) sendMessageValue(SET_MOTOR_MSG, VAL, ADDR)
#define readMotor(ADDR) sendReadMessage(READ_MOTOR_MSG, ADDR)
#define setCurrentLimit(VAL, ADDR) sendMessageValue(SET_CURRENT_LIMIT_MSG, VAL, ADDR)
#define readCurrentLimit(ADDR) sendReadMessage(READ_CURRENT_LIMIT_MSG, ADDR)
#define readCurrent(ADDR) sendReadMessage(READ_CURRENT_MSG, ADDR)
#define readDiagnostic(ADDR) sendReadMessage(DIAGNOSTIC_MSG, ADDR)
#define readAddress(ADDR) sendReadMessage(ADDRESS_MSG, ADDR)
