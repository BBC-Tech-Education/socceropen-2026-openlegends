#include "motors.h"

void Motors::init() {
    for (uint8_t i = 0; i < MOTOR_NUM; i++) { 
        motor[i].init();
    }
}

void Motors::move(float speed, float direction, float rotation, float heading) {

    float highest = -1.0f;

    for (uint8_t i = 0; i < 4; i++) {
        float dir = floatMod(direction - heading, 360.0f);
        motorSpeeds[i] = -speed * cosf((motorAngles[i] - dir) * DEG_TO_RAD_F) - rotation;
        highest = fmaxf(highest, fabsf(motorSpeeds[i]));
    }

    if (highest > 100.0f) {
        for (uint8_t i = 0; i < 4; i++) {
            motorSpeeds[i] *= 100.0f/highest;
        }
    }
    
    // for (uint8_t i = 0; i < 4; i++) {
    //     Serial.print(motorSpeeds[i]);
    //     Serial.print(" ");
    // }
    // Serial.println();

    for (uint8_t i = 0; i < MOTOR_NUM; i++) {
        motor[i].move(constrain(motorSpeeds[i], -100.0f, 100.0f));
    }
}