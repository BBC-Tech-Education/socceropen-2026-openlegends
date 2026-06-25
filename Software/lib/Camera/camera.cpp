#include "camera.h"

void Camera::init() {
    pinMode(camPins[0], INPUT);
    pinMode(camPins[1], OUTPUT);

    CAMERA_SERIAL.begin(115200);
}

float Camera::distance(uint16_t cx, uint16_t cy) {
    int16_t y = 200 - cy;
    int16_t x = 200 - cx;

    return sqrtf((float)(x*x + y*y));
}

float Camera::angle(uint16_t cx, uint16_t cy) {
    int16_t y = 200 - cy;
    int16_t x = 200 - cx;

    float angle = 450.0f - RAD_TO_DEG_F * atan2f((float)y, (float)x);

    if (angle >= 360.0f) {
        return angle - 360.0f;
    }

    return angle;
}

void Camera::update() {
    cameraData.newData = false;

    if (CAMERA_SERIAL.available() >= CAMERA_PACKET_NUMBER) { // Full packet
        uint16_t first = CAMERA_SERIAL.read();
        uint16_t second = CAMERA_SERIAL.peek();

        if (first == CAMERA_START_BYTE && second == CAMERA_START_BYTE) { // Start byte
            CAMERA_SERIAL.read();

            // for (uint8_t byte = 0; byte < CAMERA_PACKET_NUMBER - 2; byte++) {
            //     camData[byte] = CAMERA_SERIAL.read();
            //     Serial.print(camData[byte]);
            //     Serial.print(" ");
            // }
            // Serial.println();

            for (uint8_t byte = 0; byte < CAMERA_PACKET_NUMBER - 2; byte++) {
                receivedPacket[byte] = CAMERA_SERIAL.read();
            }

            for (uint8_t byte = 0; byte < (CAMERA_PACKET_NUMBER - 2) / 2; byte++) {
                camData[byte] = receivedPacket[byte * 2] | (receivedPacket[byte * 2 + 1] << 8);
                // Serial.print(camData[byte]);
                // Serial.print(" ");
            }
            // Serial.println();
            
            cameraData.ballDist = distance(camData[0], camData[1]); // Add pixel conversion
            cameraData.ballAngle = angle(camData[0], camData[1]); // Add pixel conversion???

            float yellowGoalDist = distance(camData[2], camData[3]); // Add pixel conversion
            float yellowGoalAngle = angle(camData[2], camData[3]); // Add pixel conversion???
            float blueGoalDist = distance(camData[4], camData[5]); // Add pixel conversion
            float blueGoalAngle = angle(camData[4], camData[5]); // Add pixel conversion???
            
            // cameraData.attackGoal = BLUE_GOAL_ATTACK ? blueGoal : yellowGoal;
            // cameraData.defendGoal = BLUE_GOAL_ATTACK ? yellowGoal : blueGoal;
            cameraData.attackGoalDist = BLUE_GOAL_ATTACK ? blueGoalDist : yellowGoalDist;
            cameraData.defendGoalDist = BLUE_GOAL_ATTACK ? yellowGoalDist : blueGoalDist;
            cameraData.attackGoalAngle = BLUE_GOAL_ATTACK ? blueGoalAngle : yellowGoalAngle;
            cameraData.defendGoalAngle = BLUE_GOAL_ATTACK ? yellowGoalAngle : blueGoalAngle;
        }
    // } else {
        // cameraData.attackGoal = false;
        // cameraData.defendGoal = false;
        // cameraData.attackGoalDist = -1.0f;
        // cameraData.defendGoalDist = -1.0f;
        // cameraData.attackGoalAngle = -1.0f;
        // cameraData.defendGoalAngle = -1.0f;
    }
}