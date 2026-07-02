#include "definitions.h"

int8_t sign(int8_t value) {
    return (value >= 0) ? 1 : -1;
}

float fsign(float value) {
    return (value >= 0.0f) ? 1.0f : -1.0f;
}

float floatMod(float x, float m) {
    float r = fmodf(x, m);
    return r < 0.0f ? r + m : r;
}

float angleBetween(float angleCounterClockwise, float angleClockwise) {
    return floatMod(angleClockwise - angleCounterClockwise, 360.0f);
}
  
float smallestAngleBetween(float angleCounterClockwise, float angleClockwise) {
    float ang = angleBetween(angleCounterClockwise, angleClockwise);
    return fminf(ang, 360.0f - ang);
}

float midAngleBetween(float angleCounterClockwise, float angleClockwise) {
    return floatMod(angleCounterClockwise + angleBetween(angleCounterClockwise, angleClockwise) / 2.0f, 360.0f);
}

uint8_t mod(int8_t x, int8_t m) {
    int8_t r = x % m;
    return r < 0 ? r + m : r;
}

bool angleIsInside(float angleBoundCounterClockwise, float angleBoundClockwise, float angleCheck) {
    if (angleBoundCounterClockwise < angleBoundClockwise) {
        return (angleBoundCounterClockwise < angleCheck && angleCheck < angleBoundClockwise);
    } else {
        return (angleBoundCounterClockwise < angleCheck || angleCheck < angleBoundClockwise);
    }
}