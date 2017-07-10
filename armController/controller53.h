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


uint16_t send53ReadMessage(uint8_t command, int address);
void send53MessageValue(uint8_t command, int16_t value, int address);
void flushTwi(int address);

#define set53Motor(VAL, ADDR) send53MessageValue(SET_MOTOR_MSG, VAL, ADDR)
#define rea53dMotor(ADDR) send53ReadMessage(READ_MOTOR_MSG, ADDR)
#define set53CurrentLimit(VAL, ADDR) send53MessageValue(SET_CURRENT_LIMIT_MSG, VAL, ADDR)
#define read53CurrentLimit(ADDR) send53ReadMessage(READ_CURRENT_LIMIT_MSG, ADDR)
#define read53Current(ADDR) send53ReadMessage(READ_CURRENT_MSG, ADDR)
#define read53Diagnostic(ADDR) send53ReadMessage(DIAGNOSTIC_MSG, ADDR)
#define read53Address(ADDR) send53ReadMessage(ADDRESS_MSG, ADDR)
