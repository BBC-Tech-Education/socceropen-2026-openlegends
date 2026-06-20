#ifndef LS_H
#define LS_H

#include "config.h"
#include "definitions.h"

class LightSensors {
    public:
        LightSensors() {};
        LSData lsData;
        void init();
        void read();
        float lineAngle();
        float direction(float heading);
        bool stopOnLine(bool ball, float ballAngle);
        void update(const CameraData& cameraData, float heading);

    private:
        uint8_t lightPins[4] = {LS_0, LS_1, LS_2, LS_3};
        uint8_t lightOutPins[3] = {LS_OUT, LS_OUT_2, LS_OUT_3};
        uint8_t lightPinsIndex[LS_NUMBER] = {8, 16, 17, 18, 19, 20, 21, 22, 23, 31, 30, 29, 28, 27, 26, 25, 24, 0, 1, 2, 3, 4, 5, 6, 7, 15, 14, 13, 12, 11, 10, 9, 34, 35, 36, 37, 38, 39, 47, 46, 45, 44, 43, 42, 41, 40, 32, 33}; // inner then outer
        // do this again
        uint8_t seesLine[LS_NUMBER] = {0};
        uint8_t innerClusters[4][2] = {{0}};
        uint8_t outerClusters[8][2] = {{0}};
        uint8_t state = 1;
        uint16_t lightValues[LS_NUMBER] = {0};
        uint16_t whiteValues[LS_NUMBER] = {0}; 
        float innerClusterAngles[4] = {0.0f};
        float outerClusterAngles[8] = {0.0f};
        float angleBetweenInnerClusters[4] = {0.0f};
        float angleBetweenOuterClusters[8] = {0.0f};
        float whiteAngle = -1.0f;
        float avoidAngle = -1.0f;
        float correctedLineAngle = -1.0f;
        bool clusterStart;
        bool line;
        bool innerRing;
        bool outerRing;
};

#endif