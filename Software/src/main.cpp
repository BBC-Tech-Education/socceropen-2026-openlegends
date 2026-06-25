#include <Adafruit_BNO055.h>
#include <bluetooth.h>
#include <camera.h>
#include <config.h>
#include <definitions.h>
#include <lightSensors.h>
#include <motors.h>
#include <orbit.h>
#include <PID.h>

Adafruit_BNO055 bno = Adafruit_BNO055(55, BNO055_ADDRESS_B, &Wire2);
// Bluetooth bluetooth; 
Camera camera;
LightSensors lightSensors;
LSData lsdata;
Motors motors;
Orbit orbit;
OrbitData orbitdata;
// IntervalTimer motorsOnTimer;
// IntervalTimer ledTimer;

PID imuPID(IMU_KP, IMU_KI, IMU_KD, IMU_MAX);
PID attackGoalTrackPID(ATTACK_GOAL_TRACK_KP, ATTACK_GOAL_TRACK_KI, ATTACK_GOAL_TRACK_KD, ATTACK_GOAL_TRACK_MAX);

// static volatile uint8_t motorsOn = 0;
// static volatile uint8_t ledOn = 0;
static float attackGoalAngle = 0.0f; // If first loop doesn't have camera values
static float attackGoalRot = 0.0f;

// void motorsOnUpdate() {
//     motorsOn = (motorsOn + 1) % 100;
// }

// void toggleLED() {
//     ledOn = 1 - ledOn;
//     digitalWrite(LED_BUILTIN, ledOn);
// }

void setup() {
    delay(100);

    Serial.begin(9600);

    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, HIGH);
    pinMode(LS_PWM, OUTPUT);
    analogWrite(LS_PWM, 200);
    delay(100);
    pinMode(14, INPUT); //batt volt reader

    motors.init();
    camera.init();
    lightSensors.init();
    // attacker.init();
    // defender.init();
    // bluetooth.init();

    while (!bno.begin(OPERATION_MODE_IMUPLUS)) { 
        Serial.println("BNO not working");
        delay(1000);
    }

    delay(500);
    bno.setExtCrystalUse(true);
    delay(500);

    digitalWrite(LED_BUILTIN, LOW);

    // motorsOnTimer.begin(motorsOnUpdate, 1000);
    // ledTimer.begin(toggleLED, 500000);
}

float getHeading() {
    sensors_event_t imu;
    bno.getEvent(&imu);

    float heading = imu.orientation.x;

    if (heading > 180.0f) {
        return heading - 360.0f;
    }

    return heading;
}

float getAttackGoalRot() {
    attackGoalAngle = camera.cameraData.attackGoalAngle;

    if (attackGoalAngle > 180.0f) {
        attackGoalAngle -= 360.0f;
    }

    attackGoalRot = -attackGoalTrackPID.update(attackGoalAngle, 0.0f);
    return attackGoalRot;
}

void loop() {
    // camera.update();
    // orbit.update(camera.cameraData);

    float heading = getHeading();
    float compassRot = imuPID.update(heading, 0.0f);
    // float attackGoalRot = getAttackGoalRot();
    lightSensors.update(heading);
    float lineAngle = lightSensors.getLineAngle();
    if (lineAngle != -1.0f) {
        float avoidAngle = floatMod(lineAngle + 180.0f - heading, 360.0f);
        motors.move(30.0f, avoidAngle, compassRot);
    } else {
        motors.move(0.0f, 0.0f, compassRot);
    }
    
    // float lineSize = lightSensors.getLineSize();
    // Serial.printf("lineAngle: %.2f\tlineSize: %.2f\tavoidAngle: %.2f", lineAngle, lineSize, avoidAngle);
    // Serial.println();

    float battVoltage = analogRead(14) * 0.01387096774f; // (3.3 * 4.3 / 1023);
    if (battVoltage < 11.5f) {
        motors.move(0.0f, 0.0f, 10.0f);
        Serial.println("low");
    } 
    // else {
    //     motors.move(orbit.orbitData.orbitSpeed, orbit.orbitData.orbitAngle, compassRot);
    // }


    #if DEBUG_IMU
    Serial.printf("Heading: %.2f\tCorrection: %.2f\tMode: %d", heading, compassRot, (uint8_t)bno.getMode());
    #endif

    #if DEBUG_CAMERA
    Serial.printf("ballAngle: %.2f\tattackGoalAngle: %.2f\tdefendGoalAngle: %.2f\tFPS: %.2f", camera.cameraData.ballAngle, camera.cameraData.attackGoalAngle, camera.cameraData.defendGoalAngle, camera.fps);
    #endif

    #if DEBUG_GOAL_CORRECT
    Serial.printf("attackGoalAngle: %.2f\tattackGoalRot: %.2f", attackGoalAngle, attackGoalRot);
    #endif

    #if DEBUG
    Serial.println();
    #endif
}