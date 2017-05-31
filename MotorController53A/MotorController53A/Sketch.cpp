#include <Arduino.h>
#include <core_timers.h>
#include <core_adc.h>
//#include <USI_TWI_Slave.h>
#include <TinyWireS.h>

#define MOTOR_FORWARD_PIN PIN_A7
#define MOTOR_BACKWARD_PIN PIN_B2
#define SENSE_FORWARD_PIN PIN_A1
#define SENSE_BACKWARD_PIN PIN_A0
#define SENSE_CMP_GND PIN_A5
#define JUMPER_0_PIN PIN_B0
#define JUMPER_1_PIN PIN_B1
#define JUMPER_2_PIN PIN_A2
#define JUMPER_3_PIN PIN_A3

uint8_t twiAddress = 0;

uint8_t adcGainOffset = 0;

#define MAX_CURRENT_LIMIT 465 // maximum ~100A limit
// 0.25mohm resistance across the sense from ground, using 20x gain and 1.1V reference
uint16_t currentLimit = 140; // default ~30A limit
uint16_t currentVal = 0;

// Communication
#define SET_RAW_MOTOR_MSG 1
#define READ_RAW_MOTOR_MSG 2
#define SET_CURRENT_LIMIT_MSG 3
#define GET_CURRENT_LIMIT_MSG 4
#define READ_CURRENT_MSG 5
#define DIAGNOSTIC_MSG 11
#define ADDRESS_MSG 12
#define VALUE_TEST_MSG 13

bool receivingSetRawMotor = false;
bool receivingSetCurrentLimit = false;
bool hasHighByte = false;
byte setValueHighByte;

// Motor
#define MOTOR_PWM_MAX 1000
#define MOTOR_PWM_MIN 0

int16_t lastMotorOutput = 0;
int16_t motorOutput = 0;

// We do not need the full 32-bit resolution of millis, so we save space by doing 16-bit operations
// Rollover is not a problem as the _unsigned_ subtraction will still work out.
uint16_t millis16() {
    return (uint16_t)(millis() & 0xffff);
}

void setupTimerInterrupts() {
    Timer1_SetToPowerup(); // Turn all settings off!

     // These settings should give us 8Mhz / 2 / 1 / 191 = 20942Hz ~= 21khz PWM
    ICR1 = 191;
    Timer1_SetWaveformGenerationMode(Timer1_Phase_Frequency_PWM_ICR); // Top is ICR1, OCR1A is used to modify duty cycle
    Timer1_ClockSelect(Timer1_Prescale_Value_1);

    Timer1_SetOutputCompareMatchA(0); // Set pulse width

    Timer1_EnableOverflowInterrupt();
    Timer1_EnableOutputCompareInterruptA();
}

// Does a raw ADC reading with whatever settings are already set
int readAdc() {
    ADC_StartConversion();
    while (ADC_ConversionInProgress()) {
    }
    return ADC_GetDataRegister();
}

void setupDifferentialGainAdc() {
    // 1.1V internal analog reference, ADMUX leading 10
    //analogReference(INTERNAL);

    // ADC0-ADC0 20x Gain differential ADC mode = 100011
    ADMUX = (0b10000000 | 0b100011);
    // through away first read
    readAdc();

    // let the ADC settle...
    uint16_t settleStart = millis16();
    while ((uint8_t)(millis16() - settleStart) < 2) {}

    // use second read to get gain offset
    adcGainOffset = (int8_t)readAdc();
}

ISR(TIMER1_OVF_vect) {
    if (motorOutput < 0) {
        digitalWrite(MOTOR_BACKWARD_PIN, HIGH);
    } else if (motorOutput > 0) {
        digitalWrite(MOTOR_FORWARD_PIN, HIGH);
    }
}

ISR(TIMER1_COMPA_vect) {
    if (!Timer1_IsOverflowSet()) {
        if (motorOutput < 0) {
            digitalWrite(MOTOR_BACKWARD_PIN, LOW);
        } else if(motorOutput > 0) {
            digitalWrite(MOTOR_FORWARD_PIN, LOW);
        }
    }
}

void updateMotor() {
    if (motorOutput != lastMotorOutput) {
        lastMotorOutput = motorOutput;
        digitalWrite(MOTOR_FORWARD_PIN, LOW);
        digitalWrite(MOTOR_BACKWARD_PIN, LOW);
        if (motorOutput < 0) {
            Timer1_SetOutputCompareMatchA(-motorOutput);
        } else if (motorOutput > 0) {
            Timer1_SetOutputCompareMatchA(motorOutput);
        }
    }
}

void updateControl() {
    while (TinyWireS.available()) {
        byte newByte = TinyWireS.receive();

        if (receivingSetRawMotor | receivingSetCurrentLimit) {
            if (~hasHighByte) {
                setValueHighByte = newByte;
                hasHighByte = true;
            } else {
                int value = (setValueHighByte << 8) + newByte;
                if (receivingSetRawMotor) {
                    if (value < MOTOR_PWM_MIN) {
                        value = MOTOR_PWM_MIN;
                    } else if (value > MOTOR_PWM_MAX) {
                        value = MOTOR_PWM_MAX;
                    }
                    motorOutput = value;
                    receivingSetRawMotor = false;
                } else {
                    // receivingSetCurrentLimit
                    if (value > MAX_CURRENT_LIMIT) {
                        value = MAX_CURRENT_LIMIT;
                    }
                    currentLimit = value;
                    receivingSetCurrentLimit = false;
                }
                hasHighByte = false;
            }
        } else {
            switch (newByte) {
                case SET_RAW_MOTOR_MSG:
                    receivingSetRawMotor = true;
                    break;
                case READ_RAW_MOTOR_MSG:
                    TinyWireS.send((motorOutput >> 8) & 0xff);
                    TinyWireS.send(motorOutput & 0xff);
                    break;
                case SET_CURRENT_LIMIT_MSG:
                    receivingSetCurrentLimit = true;
                    break;
                case GET_CURRENT_LIMIT_MSG:
                    TinyWireS.send((currentLimit >> 8) & 0xff);
                    TinyWireS.send(currentLimit & 0xff);
                    break;
                case READ_CURRENT_MSG:
                    TinyWireS.send((currentVal >> 8) & 0xff);
                    TinyWireS.send(currentVal & 0xff);
                    break;
                case DIAGNOSTIC_MSG:
                    TinyWireS.send('H');
                    TinyWireS.send('I');
                    break;
                case ADDRESS_MSG:
                    TinyWireS.send(0);
                    TinyWireS.send(twiAddress);
                    break;
                case VALUE_TEST_MSG:
                    TinyWireS.send((1023 >> 8) & 0xff);
                    TinyWireS.send(1023 & 0xff);
                    break;
            }
        }
    }
}

void updateCurrentControl() {
    // Only run this every 10ms to give time for motor/current response
    static uint16_t lastControlRunMs = 0;
    uint16_t nowMs = millis16();
    if (nowMs - lastControlRunMs < 10) {
        return;
    }
    lastControlRunMs = nowMs;

    if (motorOutput < 0) {
        // ADC0-ADC3 20x Gain differential ADC mode = 001011
        ADMUX = (0b10000000 | 0b001011);
    } else if (motorOutput > 0) {
        // ADC1-ADC3 20x Gain differential ADC mode = 001111
        ADMUX = (0b10000000 | 0b001111);
    }
    
    currentVal = readAdc() - adcGainOffset;
    if (currentVal > currentLimit) {
        motorOutput = (motorOutput * 30) >> 5;
        // multiplying by 30 >> 5 = 0.9375 each stage gives cutback to 52% in .1 seconds
    }
}

void setup() {
    /*pinMode(JUMPER_0_PIN, INPUT_PULLUP);
    pinMode(JUMPER_1_PIN, INPUT_PULLUP);
    pinMode(JUMPER_2_PIN, INPUT_PULLUP);
    pinMode(JUMPER_3_PIN, INPUT_PULLUP);
    pinMode(MOTOR_FORWARD_PIN, OUTPUT);
    pinMode(MOTOR_BACKWARD_PIN, OUTPUT);*/
    // equivalent but optimized...
    DDRA = 0b00001100;
    DDRB = 0b00000011;
    PINA |= 0b00001100;
    PINB |= 0b00000011;

    // We start with one high bit, because the low addresses
    // from 0 to 7 or so are reserved.
    // Our address range is 8 to 23
    uint8_t addr = 8;
    addr = (uint8_t)(addr + digitalRead(JUMPER_0_PIN));
    addr = (uint8_t)(addr + (uint8_t)(digitalRead(JUMPER_1_PIN) << (uint8_t)1));
    addr = (uint8_t)(addr + (uint8_t)(digitalRead(JUMPER_2_PIN) << (uint8_t)2));
    addr = (uint8_t)(addr + (uint8_t)(digitalRead(JUMPER_3_PIN) << (uint8_t)3));
    twiAddress = addr;

    TinyWireS.begin(twiAddress);

    setupDifferentialGainAdc();
    setupTimerInterrupts();

    motorOutput = 0;
    digitalWrite(MOTOR_FORWARD_PIN, LOW);
    digitalWrite(MOTOR_BACKWARD_PIN, LOW);
    updateMotor();
}

void loop() {
    updateControl();
    updateCurrentControl();
    updateMotor();
}
