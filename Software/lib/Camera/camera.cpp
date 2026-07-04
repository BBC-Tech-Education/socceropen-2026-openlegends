#include "camera.h"

void Camera::init() { // Initialises camera UART
    pinMode(camPins[0], INPUT); // TX Input
    pinMode(camPins[1], OUTPUT); // RX Output

    CAMERA_SERIAL.begin(115200); // Sets baud rate
}

float Camera::distance(uint16_t cx, uint16_t cy) { // Finds blob distance
    int16_t y = CY - cy; // y-coordinate of blob
    int16_t x = CX - cx; // x-coordinate of blob

    return sqrtf((float)(x*x + y*y));
}

float Camera::angle(uint16_t cx, uint16_t cy) { // Finds blob angle (true bearing)
    int16_t y = CY - cy; // y-coordinate of blob
    int16_t x = CX - cx; // x-coordinate of blob

    float angle = 450.0f - RAD_TO_DEG_F * atan2f((float)y, (float)x); // Polar to bearing

    return floatMod(angle, 360.0f);
}

void Camera::update() {
    if (CAMERA_SERIAL.available() >= CAMERA_PACKET_NUMBER) { // Full packet
        uint16_t first = CAMERA_SERIAL.read();
        uint16_t second = CAMERA_SERIAL.peek();

        if (first == CAMERA_START_BYTE && second == CAMERA_START_BYTE) { // Start byte
            cameraData.fps = (1000.0f)/(millis() - camTime);
            camTime = millis();
            // Serial.println(cameraData.fps);

            CAMERA_SERIAL.read(); // Reads second start byte

            for (uint8_t byte = 0; byte < CAMERA_PACKET_NUMBER - 2; byte++) {
                receivedPacket[byte] = CAMERA_SERIAL.read();
                // Serial.print(receivedPacket[byte]);
                // Serial.print(" ");
            }
            // Serial.println();

            for (uint8_t byte = 0; byte < (CAMERA_PACKET_NUMBER - 2) / 2; byte++) { // Bitshifts back to 0-400
                camData[byte] = receivedPacket[byte * 2] | (receivedPacket[byte * 2 + 1] << 8);
                // Serial.print(camData[byte]);
                // Serial.print(" ");
            }
            // Serial.println();
            cameraData.ballDist = (camData[0] == 500) ? 0.0f : 0.00304002f * distance(camData[0], camData[1]) * distance(camData[0], camData[1]); 
            cameraData.ballAngle = (camData[0] == 500) ? -1.0f : angle(camData[0], camData[1]);

            // cameraData.ballDist = 0.00304002f * distance(camData[0], camData[1]) * distance(camData[0], camData[1]); // Converts pixel distance to cm
            // cameraData.ballAngle = (cameraData.ballDist == 0.0f) ? -1.0f : angle(camData[0], camData[1]);
            // Serial.println(cameraData.ballAngle);

            float yellowGoalDist = (camData[2] == 500) ? 0.0f : distance(camData[2], camData[3]);
            float yellowGoalAngle = (camData[2] == 500) ? -1.0f : angle(camData[2], camData[3]);

            // float yellowGoalDist = distance(camData[2], camData[3]); // Add pixel conversion
            // float yellowGoalAngle = (yellowGoalDist == 0.0f) ? -1.0f : angle(camData[2], camData[3]);

            float blueGoalDist = (camData[4] == 500) ? 0.0f : distance(camData[4], camData[5]);
            float blueGoalAngle = (camData[4] == 500) ? -1.0f : angle(camData[4], camData[5]);

            // float blueGoalDist = distance(camData[4], camData[5]); // Add pixel conversion
            // float blueGoalAngle = (blueGoalDist == 0.0f) ? -1.0f : angle(camData[4], camData[5]);
            
            cameraData.attackGoalDist = BLUE_GOAL_ATTACK ? blueGoalDist : yellowGoalDist;
            cameraData.defendGoalDist = BLUE_GOAL_ATTACK ? yellowGoalDist : blueGoalDist;
            cameraData.attackGoalAngle = BLUE_GOAL_ATTACK ? blueGoalAngle : yellowGoalAngle;
            cameraData.defendGoalAngle = BLUE_GOAL_ATTACK ? yellowGoalAngle : blueGoalAngle;
        }
    }
}