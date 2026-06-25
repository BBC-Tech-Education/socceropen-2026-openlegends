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

    
    // Serial.println();
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

    for (uint8_t i = 0; i < LS_NUM_INNER; i++) { // outer max increase 600 and a bit
        Serial.print(lightValues[i]);
        Serial.print(" ");
    }

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
        //     lineDir = fieldLineAngle;
        // }
        if (clusterNum == 3) {
            float angleDiff12 = angleBetween(ClusterArray[0].midpoint, ClusterArray[1].midpoint);
            float angleDiff23 = angleBetween(ClusterArray[1].midpoint, ClusterArray[2].midpoint);
            float angleDiff31 = angleBetween(ClusterArray[2].midpoint, ClusterArray[0].midpoint);
            float biggestAngle = fmaxf(angleDiff12, fmaxf(angleDiff23, angleDiff31));

            if(biggestAngle == angleDiff12) {
                lineDir = midAngleBetween(ClusterArray[1].midpoint, ClusterArray[0].midpoint);
                lineSize = angleBetween(ClusterArray[1].midpoint, ClusterArray[0].midpoint) <= 180.0f ? 1.0f - cosf(DEG_TO_RAD_F * angleBetween(ClusterArray[1].midpoint, ClusterArray[0].midpoint) / 2.0f) : 1.0f;
            } else if(biggestAngle == angleDiff23) {
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
    } else {
        // Check outer ring

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

    bool noLine = (lineDir == -1.0f);
    float lineDirection = noLine ? -1.0f : floatMod(450.0f - lineDir + heading, 360.0f);
    if (onField) {
        if (lineDir != -1.0f) {
            fieldLineAngle = lineDirection;
            fieldLineSize = lineSize;
            onField = false;
        }
    } else {
        if (fieldLineSize == 3.0f) {
            if (lineDir != -1.0f && smallestAngleBetween(lineDirection, fieldLineAngle) >= 60.0f) {
                fieldLineAngle = floatMod(lineDirection + 180.0f, 360.0f);
                fieldLineSize = 2.0f - lineSize;
            }
        } else {  
            if (lineDir == -1.0f) {
                if (fieldLineSize <= 1.0f) {
                    onField = true;
                    fieldLineAngle = -1.0f;
                    fieldLineSize = -1.0f;
                } else {
                    fieldLineSize = 3.0f;
                }
            } else {
                if (smallestAngleBetween(lineDirection, fieldLineAngle) <= 90.0f) {
                    fieldLineAngle = lineDirection;
                    fieldLineSize = lineSize;
                } else {
                    fieldLineAngle = floatMod(lineDirection + 180.0f, 360.0f);
                    fieldLineSize = 2.0f - lineSize;
                }
            }
        }
    }
    Serial.print("\t");
    Serial.print(lineDirection);
    Serial.print("\t");
    Serial.print(clusterNum);
    Serial.println();
    // Serial.printf("ABSOLUTE ----- Direction: %.2f\tSize: %.2f\n", fieldLineAngle, fieldLineSize);
}

float LightSensors::getLineAngle() {
    return fieldLineAngle;
}

float LightSensors::getLineSize() {
    return fieldLineSize;
}

    //     if (whiteValues[i] == 1) {
    //         if (!inCluster) {
    //             inCluster = true;
    //             clusterNum++;
    //             ClusterArray[clusterNum].start = i;
    //         } if (i == 31 && whiteValues[0] == 1) {
    //             ClusterArray[0].start = ClusterArray[clusterNum].start;
    //             clusterNum--;
    //         }
    //     }
    //     else {
    //         if (inCluster) {
    //             inCluster = false;
    //             ClusterArray[clusterNum].end = i - 1;
    //             float diff = angleDiff(ClusterArray[clusterNum].start * 11.25f, ClusterArray[clusterNum].end * 11.25f);
    //             ClusterArray[clusterNum].midpoint = normaliseAngle(ClusterArray[clusterNum].start * 11.25f + 0.5f * diff);
    //             // ClusterArray[clusterNum].midpoint = -1 * (float)(smallMidAngle(ClusterArray[clusterNum].end * (float)(360/LS_NUM_INNER), ClusterArray[clusterNum].start * (float)(360/LS_NUM_INNER))) + 360.0f;
    //         }
    //     }
    // }
    // // Serial.println(ClusterArray[0].midpoint);

    // switch (clusterNum) {
    //     case -1:
    //         whiteAngle = -1.0f;
    //         break;

    //     case 0:
    //         whiteAngle = ClusterArray[0].midpoint;
    //         break;

    //     case 1:
    //         whiteAngle = smallMidAngle(ClusterArray[0].midpoint, ClusterArray[1].midpoint);
    //         break;
        
    //     case 2: {
    //         float angleDiff01 = angleDiff(ClusterArray[0].midpoint, ClusterArray[1].midpoint);
    //         float angleDiff12 = angleDiff(ClusterArray[1].midpoint, ClusterArray[2].midpoint);
    //         float angleDiff20 = angleDiff(ClusterArray[2].midpoint, ClusterArray[0].midpoint);
    //         float biggestAngle = fmaxf(angleDiff01, fmaxf(angleDiff12, angleDiff20));
    //         if (angleDiff01 == biggestAngle) {
    //             whiteAngle = smallMidAngle(ClusterArray[0].midpoint, ClusterArray[1].midpoint);
    //         } else if (angleDiff12 == biggestAngle) {
    //             whiteAngle = smallMidAngle(ClusterArray[1].midpoint, ClusterArray[2].midpoint);
    //         } else {
    //             whiteAngle = smallMidAngle(ClusterArray[2].midpoint, ClusterArray[0].midpoint);
    //         }

    //         Serial.print(clusterNum);
    //         Serial.print(" ");
    //         for (uint8_t i = 0; i < 3; i++) {
    //             Serial.print(ClusterArray[i].midpoint);
    //             Serial.print(" ");
    //         }
    //         Serial.println(whiteAngle);

    //         break;
    //     }
    

    
    
    //check which angles the outer hits    
    // for (uint8_t i = LS_NUM_INNER; i < LS_NUM; i++) {
    //     if (i < 36 && whiteValues[i] == 1) {
    //         outerClusterNum = 0;
    //         outerClusterAngles[0] = 0.0f;
    //     }
    //     else if (i > 35 && i < 40 && whiteValues[i] == 1) {
    //         outerClusterNum = 1;
    //         outerClusterAngles[1] = 90.0f;
    //     }
    //     else if (i > 39 && i < 44 && whiteValues[i] == 1) {
    //         outerClusterNum = 2;
    //         outerClusterAngles[2] = 180.0f;
    //     }
    //     else if (i > 43 && whiteValues[i] == 1) {
    //         outerClusterNum = 3;
    //         outerClusterAngles[3] = 270.0f;
    //     }
    // }
    
    // Serial.println(outerClusterAngles[outerClusterNum]);
    
    
    
    
    
    
    
    
    
    // delay(50);
//     return whiteAngle;
// }

// float LightSensors::direction(float heading) {
//     float correctedAngle = normaliseAngle(whiteAngle + heading);
//     // Serial.printf("Line: %d\tState: %d\tWhiteAngle: %.1f\tCorrectedAngle: %.1f\tLineAngle: %.1f\n", line, state, whiteAngle, correctedAngle, lineAngle);
//     if (state == 1) { // State 1
//         if (line) { // State 2
//             state = 2;
//             correctedLineAngle = correctedAngle;
//         }

//     } else if (state == 4) { // State 4
//         if (line) {
//             state = 3;
//             correctedLineAngle = normaliseAngle(correctedAngle + 180.0f);
//         }

//     } else { // On line (State 2 or 3)
//         if (line) {
//             if (angleDiff(correctedLineAngle, correctedAngle) <= AVOID_ANGLE_FLIP) { // State 2
//                 state = 2;
//                 correctedLineAngle = correctedAngle;
//             } else { // State 3
//                 state = 3;
//                 correctedLineAngle = normaliseAngle(correctedAngle + 180.0f);
//             }
//         } else {
//             if (state == 2) {
//                 state = 1;
//                 correctedLineAngle = -1.0f;
//             } else {
//                 state = 4;
//             }
//         }
//     }
//     avoidAngle = (state != 1) ? normaliseAngle(correctedLineAngle + 180.0f) : -1.0f;
//     return avoidAngle;
// }

// bool LightSensors::stopOnLine(bool ball, float ballAngle) {
//     if (ball && state == 2) {
//         bool stop = angleDiff(ballAngle, whiteAngle) < AVOID_ANGLE_STOP;
//         return stop;
//     } else {
//         return false;
//     }
// }

// void LightSensors::update(const CameraData& cameraData, float heading) {
//     read();
//     float angle;
//     float rawAngle = lineAngle();
//     if (rawAngle == -1.0f) {
//         angle = -1.0f;
//     } else {
//         angle = normaliseAngle(450.0f - rawAngle);
//     }
//     // Serial.println(angle);
//     // Serial.println(angle);
//     // lsData.lineAngle = lineAngle();
//     // Serial.println(lsData.whiteAngle);
//     // lsData.avoidAngle = direction(heading);
//     // lsData.stopOnLine = stopOnLine(cameraData.ball, cameraData.ballAngle);
//     // avoid speed (scalar resolute?)
// }