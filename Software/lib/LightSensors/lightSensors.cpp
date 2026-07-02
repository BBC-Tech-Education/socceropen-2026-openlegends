#include "lightSensors.h"

void LightSensors::init() {
    for (uint8_t i = 0; i < 3; i++) {
        pinMode(lightOutPins[i], INPUT);
    }

    for (uint8_t i = 0; i < 4; i++) {
        pinMode(lightPins[i], OUTPUT);
    }

    read();

    for (uint8_t i = 0; i < LS_NUM; i++) {
        whiteCal[i] = lightValues[i] + LS_THRESH_BUFF;
    }
}

void LightSensors::read() {
    for (uint8_t i = 0; i < 16; i++) {
        for (uint8_t j = 0; j < 4; j++) {
            digitalWrite(lightPins[j], (i >> j) & 0x01);
        }

        delayMicroseconds(10);

        lightValues[muxIndex[i]]      = analogRead(LS_OUT_1);
        lightValues[muxIndex[i + 16]] = analogRead(LS_OUT_2);
        lightValues[muxIndex[i + 32]] = analogRead(LS_OUT_3);
    }
    // for (uint8_t i = 0; i < LS_NUM_INNER; i++) {
    //     Serial.print(lightValues[i]);
    //     Serial.print(" ");
    // }
    // Serial.println();

}

void LightSensors::update(float heading) { //inner ring for now
    read();

    for (uint8_t i = 0; i < LS_NUM; i++) {
        whiteValues[i] = (lightValues[i] > whiteCal[i]);
    }

    // for (uint8_t i = LS_NUM_INNER; i < LS_NUM; i++) {
    //     Serial.print(whiteCal[i]);
    //     Serial.print(" ");
    // }
    // Serial.println();

    // whiteValues[8] = 0;

    for (uint8_t i = 0; i < LS_NUM_INNER; i++) {
        if (!whiteValues[i]) {
        //     whiteValues[i] = whiteValues[mod(i - 1, LS_NUM)]  || whiteValues[mod(i + 1, LS_NUM)];
        // } else {
            whiteValues[i] = (whiteValues[mod(i - 1, LS_NUM)] && whiteValues[mod(i + 1, LS_NUM)]);
        }
    }

    // for (uint8_t i = 0; i < LS_NUM_INNER; i++) { // outer max increase 600 and a bit
    //     Serial.print(lightValues[i]);
    //     Serial.print(" ");
    // }
    // Serial.println();

    clusterNum = 0;
    inCluster = false;
    LightSensors::Cluster ClusterArray[4];
    lineDir = -1.0f;
    lineSize = -1.0f;

    for (uint8_t i = 0; i < LS_NUM_INNER; i++) {
        if (!inCluster) {
            if (whiteValues[i]) {
                ClusterArray[clusterNum].start = i;
                inCluster = true;
            }
        } else {
            if (!whiteValues[i]) {
                ClusterArray[clusterNum].end = i - 1;
                inCluster = false;
                clusterNum++;
            }
        }
    }
    
    if (whiteValues[LS_NUM_INNER - 1]) {
        if (whiteValues[0]) {
            ClusterArray[0].start = ClusterArray[clusterNum].start;
        } else {
            ClusterArray[clusterNum].end = LS_NUM_INNER - 1;
            clusterNum++;
        }
    }


    if (clusterNum > 0) {
        for (uint8_t i = 0; i < 3; i++) {
            ClusterArray[i].midpoint = midAngleBetween(ClusterArray[i].start * 11.25f, ClusterArray[i].end * 11.25f);
        }

        // for (uint8_t i = 0; i < 2; i++) {
        //     Serial.printf("CLUSTER %d: s - %d, e - %d, m - %.2f\t", i + 1, ClusterArray[i].start, ClusterArray[i].end, ClusterArray[i].midpoint);
        // }
        // Serial.println();
        // Serial.printf("CLUSTER 3: s - %d, e - %d, m - %.2f\n", ClusterArray[2].start, ClusterArray[2].end, ClusterArray[2].midpoint);

        // if (clusterNum == 4) {
        //
        // }
        if (clusterNum == 3) {
            float angleDiff12 = angleBetween(ClusterArray[0].midpoint, ClusterArray[1].midpoint);
            float angleDiff23 = angleBetween(ClusterArray[1].midpoint, ClusterArray[2].midpoint);
            float angleDiff31 = angleBetween(ClusterArray[2].midpoint, ClusterArray[0].midpoint);
            float biggestAngle = fmaxf(angleDiff12, fmaxf(angleDiff23, angleDiff31));

            if (biggestAngle == angleDiff12) {
                lineDir = midAngleBetween(ClusterArray[1].midpoint, ClusterArray[0].midpoint);
                lineSize = angleBetween(ClusterArray[1].midpoint, ClusterArray[0].midpoint) <= 180.0f ? 1.0f - cosf(DEG_TO_RAD_F * angleBetween(ClusterArray[1].midpoint, ClusterArray[0].midpoint) / 2.0f) : 1.0f;
            } else if (biggestAngle == angleDiff23) {
                lineDir = midAngleBetween(ClusterArray[2].midpoint, ClusterArray[1].midpoint);
                lineSize = angleBetween(ClusterArray[2].midpoint, ClusterArray[1].midpoint) <= 180.0f ? 1.0f - cosf(DEG_TO_RAD_F * angleBetween(ClusterArray[2].midpoint, ClusterArray[1].midpoint) / 2.0f) : 1.0f;
            } else {
                lineDir = midAngleBetween(ClusterArray[0].midpoint, ClusterArray[2].midpoint);
                lineSize = angleBetween(ClusterArray[0].midpoint, ClusterArray[2].midpoint) <= 180.0f ? 1.0f - cosf(DEG_TO_RAD_F * angleBetween(ClusterArray[0].midpoint, ClusterArray[2].midpoint) / 2.0f) : 1.0f;
            }
        } else if (clusterNum == 2) {
            bool clockwise = angleBetween(ClusterArray[0].midpoint, ClusterArray[1].midpoint) <= 180.0f;
            lineDir = clockwise ? midAngleBetween(ClusterArray[0].midpoint, ClusterArray[1].midpoint) : midAngleBetween(ClusterArray[1].midpoint, ClusterArray[0].midpoint);
            lineSize = 1.0f - cosf(DEG_TO_RAD_F * (clockwise ? angleBetween(ClusterArray[0].midpoint, ClusterArray[1].midpoint) / 2.0f : angleBetween(ClusterArray[1].midpoint, ClusterArray[0].midpoint) / 2.0f));
        } else {
            lineDir = ClusterArray[0].midpoint;
            lineSize = 1.0f - cosf(DEG_TO_RAD_F * angleBetween(ClusterArray[0].start * 11.25f, ClusterArray[0].end * 11.25f) / 2.0f);
        }
    } else { // Check outer ring
        if (whiteValues[32] || whiteValues[33] || whiteValues[34] || whiteValues[35]) {
            ClusterArray[0].midpoint = 0.0f;
            clusterNum ++;
        }

        if (whiteValues[36] || whiteValues[37] || whiteValues[38] || whiteValues[39]) {
            ClusterArray[1].midpoint = 90.0f;
            clusterNum ++;
        }

        if (whiteValues[40] || whiteValues[41] || whiteValues[42] || whiteValues[43]) {
            ClusterArray[2].midpoint = 180.0f;
            clusterNum ++;
        }

        if (whiteValues[44] || whiteValues[45] || whiteValues[46] || whiteValues[47]) {
            ClusterArray[3].midpoint = 270.0f;
            clusterNum ++;
        }

        if (clusterNum == 2) {
            if (ClusterArray[0].midpoint != -1.0f) {
                if (ClusterArray[1].midpoint != -1.0f) {
                    lineDir = 45.0f;
                    lineSize = 0.1f;
                } else if (ClusterArray[3].midpoint != -1.0f) {
                    lineDir = 315.0f;
                    lineSize = 0.1f;
                }
            } else if (ClusterArray[2].midpoint != -1.0f) {
                if (ClusterArray[1].midpoint != -1.0f) {
                    lineDir = 135.0f;
                    lineSize = 0.1f;
                } else if (ClusterArray[3].midpoint != -1.0f) {
                    lineDir = 225.0f;
                    lineSize = 0.1f;
                }
            }
        } else if (clusterNum == 1) {
            for (uint8_t i = 0; i < 4; i++) {
                if (ClusterArray[i].midpoint != -1.0f) {
                    lineDir = ClusterArray[i].midpoint;
                }
            }
            lineSize = 0.1f;
        }

    }

    // Serial.printf("Direction: %.2f\tSize: %.2f\n", lineDir, lineSize);

    // In field: lineSize == -1.0f
    // 
    bool noLine = (lineDir == -1.0f);
    float lineDirection = noLine ? -1.0f : floatMod(450.0f - lineDir + heading, 360.0f);
    if (onField) {
        if (lineDir != -1.0f) { // State 2
            fieldLineAngle = lineDirection;
            fieldLineSize = lineSize;
            onField = false;
            flip = false;
        }
    } else {
        if (fieldLineSize == 3.0f) {
            if (lineDir != -1.0f && smallestAngleBetween(lineDirection, fieldLineAngle) >= 60.0f) { // State 3
                fieldLineAngle = floatMod(lineDirection + 180.0f, 360.0f);
                fieldLineSize = 2.0f - lineSize;
                flip = true; // ?
            }
        } else {  
            if (lineDir == -1.0f) {
                if (fieldLineSize <= 1.0f) { // State 1
                    onField = true;
                    fieldLineAngle = -1.0f;
                    fieldLineSize = -1.0f;
                    flip = false;
                } else { // State 4
                    fieldLineSize = 3.0f;
                    flip = true;
                }
            } else {
                if (smallestAngleBetween(lineDirection, fieldLineAngle) <= 90.0f) {
                    fieldLineAngle = lineDirection;
                    fieldLineSize = lineSize;
                    flip = false;
                } else {
                    fieldLineAngle = floatMod(lineDirection + 180.0f, 360.0f);
                    fieldLineSize = 2.0f - lineSize;
                    flip = true;
                }
            }
        }
    }
    // Serial.print("\t");
    // Serial.print(lineSize);
    // Serial.print(" ");
    // Serial.print(fieldLineSize);
    // Serial.print(" ");
    // Serial.print(lineDir);
    // Serial.print(" ");
    // Serial.print(fieldLineAngle);
    // Serial.print(" ");
    // Serial.print(onField);
    // Serial.print(" ");
    // Serial.print(clusterNum);
    // Serial.println();
    // delay(100);
    // Serial.printf("ABSOLUTE ----- Direction: %.2f\tSize: %.2f\n", fieldLineAngle, fieldLineSize);
}

float LightSensors::getLineAngle() {
    return fieldLineAngle;
}

float LightSensors::getLineSize() {
    return fieldLineSize;
}