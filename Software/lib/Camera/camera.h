#ifndef CAMERA_H
#define CAMERA_H

#include "config.h"
#include "definitions.h"

class Camera {
    public:
        Camera() {}
        CameraData cameraData;
        
        void init();
        void update(); 

        float fps;
        
    private:
        uint8_t camPins[2] = {CAMERA_TX, CAMERA_RX};  
        uint8_t receivedPacket[12] = {0};
        uint16_t camData[6] = {0};

        float distance(uint16_t cx, uint16_t cy);
        float angle(uint16_t cx, uint16_t cy);

        unsigned long camTime = 0;
        
};

#endif