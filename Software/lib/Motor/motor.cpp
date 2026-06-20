#include "motor.h"

Motor::Motor(uint8_t ina, uint8_t inb, uint8_t pwm) { // Constructor for Motor class
    INA = ina;
    INB = inb;
    PWM = pwm;
}

void Motor::init() {
    pinMode(INA, OUTPUT);
    pinMode(INB, OUTPUT);
    pinMode(PWM, OUTPUT);

    digitalWrite(INA, HIGH);
    digitalWrite(INB, HIGH);
    delayMicroseconds(100);
    analogWriteFrequency(PWM, MOTOR_PWM_FREQ);
    analogWrite(PWM, 0);
}

void Motor::move(float speed) {
    uint8_t pwm = fabs((speed / 100.0f)) * (uint8_t)255;
    uint8_t ina = speed > 0.0f ? HIGH : LOW;
    uint8_t inb = speed < 0.0f ? HIGH : LOW;

    digitalWrite(INA, ina);
    digitalWrite(INB, inb);
    analogWrite(PWM, pwm);
}