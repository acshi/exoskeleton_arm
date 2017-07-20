#include <Arduino.h>
#include <core_timers.h>
#include <core_adc.h>
#include <TinyWireS.h>

#define PHASE_PIN PIN_A3
#define ENABLE_PIN PIN_A7
#define PREDRIVER_STATUS_PIN PIN_B2
#define READ_PREDRIVER_STATUS() (PINB & _BV(PINB2))

#define MOTOR_OFF(); PORTA &= ~_BV(PORTA7);
#define PHASE_ON(); PORTA |= _BV(PORTA3);
#define PHASE_OFF(); PORTA &= ~_BV(PORTA3);

#define SENSE_ADC 1

#define JUMPER_0_PIN PIN_B0
#define JUMPER_1_PIN PIN_B1
#define JUMPER_2_PIN PIN_A2
#define JUMPER_3_PIN PIN_A3

#define READ_JUMPER_0() (PINB & _BV(PINB0))
#define READ_JUMPER_1() (PINB & _BV(PINB1))
#define READ_JUMPER_2() (PINA & _BV(PINA2))
#define READ_JUMPER_3() (PINA & _BV(PINA3))

#define ERR_NONE 0
#define ERR_OVERCURRENT 1
#define ERR_VDS_FAULT 2

uint8_t twiAddress = 0;

uint8_t adcOffsetError = 0;

#define MAX_CURRENT_LIMIT 3723 // maximum ~100A limit
// 1mohm resistance across the sense from ground, using 10x gain and 1.1V reference and virtual 12-bit ADC
uint16_t currentLimit = 1117; // default ~30A limit
uint16_t currentVal = 0;

// we interface with the outside world using 10 milliamps as the unit
#define CURRENT_10MA_TO_VAL(val) ((val * 3 + 4) >> 3)
#define CURRENT_VAL_TO_10MA(val) (((val << 3) + 1) / 3)

// Communication
#define SET_MOTOR_MSG 1
#define READ_MOTOR_MSG 2
#define SET_CURRENT_LIMIT_MSG 3
#define GET_CURRENT_LIMIT_MSG 4
#define READ_CURRENT_MSG 5
#define READ_TEMPERATURE_MSG 6
#define READ_ERROR_MSG 7
#define DIAGNOSTIC_MSG 11
#define ADDRESS_MSG 12
#define MSG_END_BYTE 0xed
#define VAL_END_BYTE 0xec

bool receivingSetRawMotor = false;
bool receivingSetCurrentLimit = false;

// Motor
#define MOTOR_PWM_MAX 255

int16_t lastMotorOutput = (MOTOR_PWM_MAX + 1);
int16_t motorOutput = 0;

// We do not need the full 32-bit resolution of millis, so we save space by doing 16-bit operations
// Rollover is not a problem as the _unsigned_ subtraction will still work out.
uint16_t millis16() {
    return (uint16_t)(millis() & 0xffff);
}

void setupTimerInterrupts() {
    Timer0_SetToPowerup(); // Turn all settings off!

     // These settings should give us   8Mhz / 2 / 1 / 255 = 15686Hz ~= 16khz PWM
    Timer0_SetWaveformGenerationMode(Timer0_Phase_Correct_PWM_FF); // Top is 255, OCR0A is used to modify duty cycle
    Timer0_ClockSelect(Timer0_Prescale_Value_1);

    // Sharing settings with the millisecond timer, but that doesn't use the output compare hardware.
    // It uses a 64x prescaler and Fast_PWM_FF mode (count to 255) ... 8Mhz / 64 / 256 = 488.28Hz
    Timer1_SetCompareOutputModeB(Timer1_Clear);
}

// Our DAC to the A4956's current limiting. Very rough. Each unit should increase the limit by about 1.953A
// since from the A4956 Ipeak = Vref / (10 * Rsense), where Rsense is 0.001ohm.
// And our Timer1 configured as the millis timer is using a top of 255.
// So our smallest output voltage is 5V * 1 / 256.
// as such, this is only for a very rough extra safety measure. We do finer current control ourselves.
void updateCurrentLimit() {
    Timer1_SetOutputCompareMatchB((CURRENT_VAL_TO_10MA(currentLimit) * 131) >> 8);
}

// Does a raw ADC reading with whatever settings are already set
int readAdc() {
    ADC_StartConversion();
    while (ADC_ConversionInProgress()) {
    }
    return ADC_GetDataRegister();
}

uint16_t readVirtual12BitAdc() {
    uint16_t sum = 0;
    for (uint8_t i = 0; i < 16; i++) {
        sum += readAdc();
    }
    return sum >> 2;
}

uint16_t readSense() {
    ADMUX = (0b10000000 | SENSE_ADC);
    // throw away first read
    readAdc();

    uint16_t current = readVirtual12BitAdc();
    
    if (current < adcOffsetError) {
        current = 0;
    } else {
        current -= adcOffsetError;
    }
    return current;
}

int16_t readInternalTemp() {
    ADMUX = 0b10100010; // 1.1V internal reference and ADC8 (internal temperature)
    readAdc(); // throw away result following change to ADMUX

    int16_t temp = readVirtual12BitAdc() - (275 << 2); // approx from datasheet. can be calibrated.
    return temp;
}

void setupAdc() {
    readSense();

    // let the ADC settle...
    uint16_t settleStart = millis16();
    while ((uint8_t)(millis16() - settleStart) < 1) {}

    adcOffsetError = (uint8_t)readSense();
}

void updateMotor() {
    if (motorOutput != lastMotorOutput) {
        lastMotorOutput = motorOutput;
        MOTOR_OFF();
        Timer0_SetCompareOutputModeB(Timer0_Disconnected);
        if (motorOutput < 0) {
            PHASE_ON();
            Timer0_SetOutputCompareMatchB(-motorOutput);
            Timer0_SetCompareOutputModeB(Timer0_Clear);
        } else if (motorOutput > 0) {
            PHASE_OFF();
            Timer0_SetOutputCompareMatchB(motorOutput);
            Timer0_SetCompareOutputModeB(Timer0_Clear);
        }
    }
}

void updateCurrentControl() {
    if (motorOutput == 0) {
        currentVal = 0;
        return;
    }
    
    uint16_t current = readSense();
    currentVal = ((currentVal << 1) + current * 30) >> 5;

    // Only run this limiting part every 10ms to give time for motor/current response
    static uint16_t lastControlRunMs = 0;
    uint16_t nowMs = millis16();
    if (nowMs - lastControlRunMs < 10) {
        return;
    }
    lastControlRunMs = nowMs;

    if (currentVal > currentLimit) {
        motorOutput = (motorOutput * 30) >> 5;
        // multiplying by 30 >> 5 = 0.9375 each stage gives cutback to 52% in .1 seconds
    }
}

void send16bits(uint16_t val) {
    TinyWireS.send((val >> 8) & 0xff);
    TinyWireS.send(val & 0xff);
}

void updateControl() {
    // to help deal with interference from motor noise,
    // we collect the bytes here in this buffer
    // we accept a single "new byte" when the first two byte values are equal
    // and the third byte is the MSG_END_BYTE
    static uint8_t receiveBuffer[5];

    while (TinyWireS.available()) {
        for (uint8_t i = 0; i < 4; i++) {
            receiveBuffer[i] = receiveBuffer[i + 1];
        }
        receiveBuffer[4] = TinyWireS.receive();

        bool gotMsg = (receiveBuffer[2] == receiveBuffer[3]) & (receiveBuffer[4] == MSG_END_BYTE);
        bool gotValue = (receiveBuffer[0] == receiveBuffer[2]) & (receiveBuffer[1] == receiveBuffer[3]) & (receiveBuffer[4] == VAL_END_BYTE);

        if (!gotMsg & !gotValue) {
            /*for (uint8_t i = 0; i < 5; i++) {
                TinyWireS.send(receiveBuffer[i]);
            }
            TinyWireS.send(240);*/
            continue;
        }

        if (gotValue & (receivingSetRawMotor | receivingSetCurrentLimit)) {
            int16_t value = (receiveBuffer[0] << 8) | receiveBuffer[1];
            if (receivingSetRawMotor) {
                if ((value <= MOTOR_PWM_MAX) & (value >= -MOTOR_PWM_MAX)) {
                    motorOutput = value;
                }
                receivingSetRawMotor = false;
            } else {
                // receivingSetCurrentLimit
                value = CURRENT_10MA_TO_VAL(value);
                // simultaneously prevent negative values and too high values
                if ((uint16_t)value <= MAX_CURRENT_LIMIT) {
                    currentLimit = value;
                    updateCurrentLimit();
                }
                receivingSetCurrentLimit = false;
            }
        } else {
            receivingSetRawMotor = false;
            receivingSetCurrentLimit = false;

            uint16_t current10ma;
            uint16_t temperature;

            switch (receiveBuffer[3]) {
                case SET_MOTOR_MSG:
                    receivingSetRawMotor = true;
                    break;
                case READ_MOTOR_MSG:
                    send16bits(motorOutput);
                    break;
                case SET_CURRENT_LIMIT_MSG:
                    receivingSetCurrentLimit = true;
                    break;
                case GET_CURRENT_LIMIT_MSG:
                    current10ma = CURRENT_VAL_TO_10MA(currentLimit);
                    send16bits(current10ma);
                    break;
                case READ_CURRENT_MSG:
                    current10ma = CURRENT_VAL_TO_10MA(currentVal);
                    send16bits(current10ma);
                    break;
                case READ_TEMPERATURE_MSG:
                    temperature = readInternalTemp();
                    send16bits(temperature);
                    break;
                case READ_ERROR_MSG:
                    if (READ_PREDRIVER_STATUS() == 0) {//digitalRead(PREDRIVER_STATUS_PIN) == LOW) {
                        if (readSense() == 0) {
                            send16bits(ERR_VDS_FAULT);
                        } else {
                            send16bits(ERR_OVERCURRENT);
                        }
                    } else {
                        send16bits(ERR_NONE);
                    }
                    break;
                case DIAGNOSTIC_MSG:
                    send16bits('HI');
                    break;
                case ADDRESS_MSG:
                    send16bits(twiAddress);
                    break;
            }
        }
    }
}

void setup() {
    /*pinMode(JUMPER_0_PIN, INPUT_PULLUP);
    pinMode(JUMPER_1_PIN, INPUT_PULLUP);
    pinMode(JUMPER_2_PIN, INPUT_PULLUP);
    pinMode(JUMPER_3_PIN, INPUT_PULLUP);
    pinMode(SENSE_PIN, INPUT);
    pinMode(PREDRIVER_STATUS_PIN, INPUT);
    pinMode(ENABLE_PIN, OUTPUT);*/
    // equivalent but optimized...
    DDRA = 0b11110001;
    DDRB = 0b11111000;
    PORTA = 0b00001100;
    PORTB = 0b00000011;

    // Our address range is 8 to 23 because 0 to 7 are reserved.
    uint8_t addr = 23;
    addr = (uint8_t)(addr - (READ_JUMPER_0() ? 1 : 0));//digitalRead(JUMPER_0_PIN));
    addr = (uint8_t)(addr - (READ_JUMPER_1() ? 2 : 0));//(uint8_t)(digitalRead(JUMPER_1_PIN) << (uint8_t)1));
    addr = (uint8_t)(addr - (READ_JUMPER_2() ? 4 : 0));//(uint8_t)(digitalRead(JUMPER_2_PIN) << (uint8_t)2));
    addr = (uint8_t)(addr - (READ_JUMPER_3() ? 8 : 0));//(uint8_t)(digitalRead(JUMPER_3_PIN) << (uint8_t)3));
    twiAddress = addr;

    TinyWireS.begin(twiAddress);

    // Since it doubles as a jumper at first
    //pinMode(PHASE_PIN, OUTPUT);
    DDRA = 0b11111001;

    setupAdc();
    setupTimerInterrupts();
    updateCurrentLimit();

    motorOutput = 0;
    updateMotor();
}

void loop() {
    updateControl();
    updateCurrentControl();
    updateMotor();
}
