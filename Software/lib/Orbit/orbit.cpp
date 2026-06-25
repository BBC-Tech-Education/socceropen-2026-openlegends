#include "orbit.h"

float Orbit::speed(bool ball, bool attackGoal, float attackGoalAngle, float ballAngle, float ballDist) {
    // float surgeAngle = attackGoal ? attackGoalAngle : 0.0f;
    // float minorAngle = angleDiff(ballAngle, 0.0f);
    // if (minorAngle < ATTACK_SURGE_ANGLE && ballDist < ATTACK_CLOSE_DISTANCE) { // Surges and accelerates
    //     moveSpeed = constrain(moveSpeed + 0.5f, ATTACK_SLOW_SPEED, ATTACK_SURGE_SPEED);
    //     return moveSpeed;
    // } else {
    //     moveSpeed = -1.0f;
    // }
    // float m = (ATTACK_FAST_SPEED - ATTACK_SLOW_SPEED)/(ATTACK_FAR_DISTANCE - ATTACK_CLOSE_DISTANCE);
    // float line = m * (ballDist - ATTACK_CLOSE_DISTANCE) + ATTACK_SLOW_SPEED;
    // moveSpeed = constrain(line, ATTACK_SLOW_SPEED, ATTACK_FAST_SPEED);
    moveSpeed = ballDist == 0.0f ? 0.0f : 18.0f;
    // Serial.println(moveSpeed);
    return moveSpeed;
}

float Orbit::angle(bool attackGoal, float attackGoalAngle, float ballAngle, float ballDist) {
    float minorAngle = smallestAngleBetween(ballAngle, 0.0f);
    float surgeAngle = attackGoal ? attackGoalAngle : 0.0f;

    if (minorAngle < ATTACK_SURGE_ANGLE && 0.00304002f * ballDist * ballDist < ATTACK_CLOSE_DISTANCE) { // tune values
        return surgeAngle;
    }

    float angleOffset = constrain(4.0f * minorAngle, 0.0f, ORBIT_MAX_OFFSET);

    float pixelToCm = 0.00304002f * ballDist * ballDist;
    float numerator = 1.0f;
    float denominator = expf(0.06f * (pixelToCm - 60.0f)) + 1.0f;
    float distScaleFactor = constrain(numerator/denominator, ORBIT_MIN_MULTIPLIER, ORBIT_MAX_MULTIPLIER);
    float moveOffset = angleOffset * distScaleFactor;

    float moveAngle = ballAngle > 180.0f ? ballAngle - moveOffset : ballAngle + moveOffset;
    return floatMod(moveAngle, 360.0f);
}

void Orbit::update(const CameraData& cameraData) {
    orbitData.orbitSpeed = speed(cameraData.ball, cameraData.attackGoal, cameraData.attackGoalAngle, cameraData.ballAngle, cameraData.ballDist);
    orbitData.orbitAngle = angle(cameraData.attackGoal, cameraData.attackGoalAngle, cameraData.ballAngle, cameraData.ballDist);
}