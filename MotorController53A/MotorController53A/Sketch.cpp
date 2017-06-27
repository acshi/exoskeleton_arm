#include <Arduino.h>
#include <core_timers.h>
#include <core_adc.h>
#include <TinyWireS.h>

#define MOTOR_FORWARD_PIN PIN_A7
#define MOTOR_BACKWARD_PIN PIN_B2

#define MOTOR_FORWARD_ON(); PORTA |= _BV(PORTA7);
#define MOTOR_FORWARD_OFF(); PORTA &= ~_BV(PORTA7);
#define MOTOR_FORWARD_TOGGLE(); PINA |= _BV(PORTA7);

#define MOTOR_BACKWARD_ON(); PORTB |= _BV(PORTB2);
#define MOTOR_BACKWARD_OFF(); PORTB &= ~_BV(PORTB2);
#define MOTOR_BACKWARD_TOGGLE(); PINB |= _BV(PORTB2);

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
#define SET_MOTOR_MSG 1
#define READ_MOTOR_MSG 2
#define SET_CURRENT_LIMIT_MSG 3
#define GET_CURRENT_LIMIT_MSG 4
#define READ_CURRENT_MSG 5
#define DIAGNOSTIC_MSG 11
#define ADDRESS_MSG 12
#define MSG_END_BYTE 0xed
#define VAL_END_BYTE 0xec

bool receivingSetRawMotor = false;
bool receivingSetCurrentLimit = false;

// Motor
#define MOTOR_PWM_MAX 248

int16_t lastMotorOutput = (MOTOR_PWM_MAX + 1);
int16_t motorOutput = 0;

// We do not need the full 32-bit resolution of millis, so we save space by doing 16-bit operations
// Rollover is not a problem as the _unsigned_ subtraction will still work out.
uint16_t millis16() {
    return (uint16_t)(millis() & 0xffff);
}

void setupTimerInterrupts() {
    Timer0_SetToPowerup(); // Turn all settings off!

     // These settings should give us   8Mhz / 2 / 1 / 191 = 20942Hz ~= 21khz PWM
     // Or we can have...               8Mhz / 2 / 1 / 255 = 15686Hz ~= 16khz PWM
    //OCR0A = MOTOR_PWM_MAX;
    Timer0_SetWaveformGenerationMode(Timer0_Phase_Correct_PWM_FF); // Top is 255, OCR0A is used to modify duty cycle
    Timer0_ClockSelect(Timer0_Prescale_Value_64); // making it actually 15686Hz / 64 = 245Hz

    //Timer0_EnableOutputCompareInterruptA();
    //Timer0_EnableOutputCompareInterruptB();
    //Timer0_EnableOverflowInterrupt();
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
    while ((uint8_t)(millis16() - settleStart) < 1) {}

    // use second read to get gain offset
    adcGainOffset = (int8_t)readAdc();
}

/*ISR(TIMER1_OVF_vect) {
    if (motorOutput < 0) {
        //digitalWrite(MOTOR_BACKWARD_PIN, HIGH);
        MOTOR_BACKWARD_ON();
    } else if (motorOutput > 0) {
        //digitalWrite(MOTOR_FORWARD_PIN, HIGH);
        MOTOR_FORWARD_ON();
    }
}*/

/*ISR(TIMER0_COMPA_vect) {
    adcGainOffset++;
}

ISR(TIMER0_COMPB_vect) {
    adcGainOffset--;
}

ISR(TIMER0_OVF_vect) {
    adcGainOffset++;
}*/

/*ISR(TIMER1_COMPA_vect) {
    uint8_t TCNT1L_1 = TCNT1L;
    uint8_t TCNT1L_2 = TCNT1L;
    if (TCNT1L_2 < TCNT1L_1) {
        if (motorOutput < 0) {
            MOTOR_BACKWARD_ON();
        } else if(motorOutput > 0) {
            MOTOR_FORWARD_ON();
        }
    } else {
        if (motorOutput < 0) {
            MOTOR_BACKWARD_OFF();
        } else if(motorOutput > 0) {
            MOTOR_FORWARD_OFF();
        }
    }
}*/

void updateMotor() {
    if (motorOutput != lastMotorOutput) {
        lastMotorOutput = motorOutput;
        //digitalWrite(MOTOR_FORWARD_PIN, LOW);
        //digitalWrite(MOTOR_BACKWARD_PIN, LOW);
        Timer0_SetCompareOutputModeA(Timer0_Disconnected);
        Timer0_SetCompareOutputModeB(Timer0_Disconnected);
        MOTOR_FORWARD_OFF();
        MOTOR_BACKWARD_OFF();
        if (motorOutput < 0) {
            Timer0_SetOutputCompareMatchA(-motorOutput);
            Timer0_SetCompareOutputModeA(Timer0_Clear);
        } else if (motorOutput > 0) {
            Timer0_SetOutputCompareMatchB(motorOutput);
            Timer0_SetCompareOutputModeB(Timer0_Clear);
        }
    }
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
                // simultaneously prevent negative values and too high values
                if ((uint16_t)value <= MAX_CURRENT_LIMIT) {
                    currentLimit = value;
                }
                receivingSetCurrentLimit = false;
            }
        } else {
            receivingSetRawMotor = false;
            receivingSetCurrentLimit = false;

            switch (receiveBuffer[3]) {
                case SET_MOTOR_MSG:
                    receivingSetRawMotor = true;
                    break;
                case READ_MOTOR_MSG:
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
            }
        }
    }
}

void updateCurrentControl() {
    if (motorOutput < 0) {
        // ADC0-ADC3 20x Gain differential ADC mode = 001011
        ADMUX = (0b10000000 | 0b001011);
    } else if (motorOutput > 0) {
        // ADC1-ADC3 20x Gain differential ADC mode = 001111
        ADMUX = (0b10000000 | 0b001111);
    } else {
        currentVal = 0;
        return;
    }
    
    uint16_t current = readAdc();
    
    if (current < adcGainOffset) {
        current = 0;
    } else {
        //current -= adcGainOffset;
    }

    //currentVal = ((currentVal << 1) + current * current * 30) >> 5;
    if (current > currentVal) {
        currentVal = current;
    }

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

void setup() {
    /*pinMode(JUMPER_0_PIN, INPUT_PULLUP);
    pinMode(JUMPER_1_PIN, INPUT_PULLUP);
    pinMode(JUMPER_2_PIN, INPUT_PULLUP);
    pinMode(JUMPER_3_PIN, INPUT_PULLUP);
    pinMode(MOTOR_FORWARD_PIN, INPUT);
    pinMode(MOTOR_BACKWARD_PIN, INPUT);*/
    // equivalent but optimized...
    DDRA = 0b11110011;
    DDRB = 0b11111100;
    PORTA = 0b00001100;
    PORTB = 0b00000011;

    // Our address range is 8 to 23 because 0 to 7 are reserved.
    uint8_t addr = 23;
    addr = (uint8_t)(addr - digitalRead(JUMPER_0_PIN));
    addr = (uint8_t)(addr - (uint8_t)(digitalRead(JUMPER_1_PIN) << (uint8_t)1));
    addr = (uint8_t)(addr - (uint8_t)(digitalRead(JUMPER_2_PIN) << (uint8_t)2));
    addr = (uint8_t)(addr - (uint8_t)(digitalRead(JUMPER_3_PIN) << (uint8_t)3));
    twiAddress = addr;

    TinyWireS.begin(twiAddress);

    setupDifferentialGainAdc();
    setupTimerInterrupts();

    motorOutput = 0;
    updateMotor();
}

void loop() {
    updateControl();
    updateCurrentControl();
    updateMotor();
}
