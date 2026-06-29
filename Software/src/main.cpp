#include <Adafruit_BNO055.h>
#include <bluetooth.h>
#include <camera.h>
#include <config.h>
#include <definitions.h>
#include <lightSensors.h>
#include <motors.h>
#include <PID.h>

Adafruit_BNO055 bno = Adafruit_BNO055(55, BNO055_ADDRESS_B, &Wire2);
// Bluetooth bluetooth; 
Camera camera;
LightSensors lightSensors;
Motors motors;

PID imuPID(IMU_KP, IMU_KI, IMU_KD, IMU_MAX);
PID attackGoalTrackPID(ATTACK_GOAL_TRACK_KP, ATTACK_GOAL_TRACK_KI, ATTACK_GOAL_TRACK_KD, ATTACK_GOAL_TRACK_MAX);

static float heading = 0.0f;
static float absoluteOrbitAngle = 0.0f;
static float orbitSpeed = 0.0f;

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

void getHeading() { // Relative
    sensors_event_t imu;
    bno.getEvent(&imu);

    heading = imu.orientation.x;

    if (heading > 180.0f) {
        heading -= 360.0f;
    }
}

float getAttackGoalRot() { // Relative not absolute (rotating doesn't change goal angle)
    float attackGoalAngle = camera.cameraData.attackGoalAngle;

    if (attackGoalAngle > 180.0f) {
        attackGoalAngle -= 360.0f;
    }

    float attackGoalRot = -attackGoalTrackPID.update(attackGoalAngle, 0.0f);
    return attackGoalRot;
}

void getOrbitMovement() {
    float absoluteBallAngle = floatMod(camera.cameraData.ballAngle + heading, 360.0f);
    float absoluteAttackGoalAngle = floatMod(camera.cameraData.attackGoalAngle + heading, 360.0f);
    float minorAngle = smallestAngleBetween(absoluteBallAngle, absoluteAttackGoalAngle);

    if (minorAngle < ATTACK_SURGE_ANGLE && camera.cameraData.ballDist < ATTACK_CLOSE_DISTANCE) { // Surge case
        absoluteOrbitAngle = absoluteAttackGoalAngle;
        orbitSpeed = ATTACK_SURGE_SPEED;
        return; // can you have return in void
    }

    float angleOffset = constrain(3.0f * minorAngle, 0.0f, ORBIT_MAX_OFFSET);
    float numerator = 1.0f;
    float denominator = expf(0.06f * (camera.cameraData.ballDist - 60.0f)) + 1.0f;
    float distScaleFactor = constrain(numerator/denominator, ORBIT_MIN_MULTIPLIER, ORBIT_MAX_MULTIPLIER);
    float moveOffset = angleOffset * distScaleFactor;

    float moveAngle = absoluteBallAngle > 180.0f ? absoluteBallAngle - moveOffset : absoluteBallAngle + moveOffset;
    absoluteOrbitAngle = floatMod(moveAngle, 360.0f);
    orbitSpeed = camera.cameraData.ballDist == 0.0f ? 0.0f : ((ATTACK_FAST_SPEED - ATTACK_SLOW_SPEED) / ORBIT_MAX_OFFSET) * moveOffset + ATTACK_SLOW_SPEED;;
}

bool batteryLow() {
    float battVoltage = analogRead(14) * 0.01387096774f; // (3.3 * 4.3 / 1023);
    if (battVoltage < 11.1f) {
        motors.move(0.0f, 0.0f, 10.0f, 0.0f);
        Serial.println("low");
        return true;
    }
    return false;
}

void loop() {
    if (batteryLow()) {
        return; // can you have return in void
    }
    camera.update();
    getHeading();
    getOrbitMovement();
    lightSensors.update(heading);
    float absoluteLineAngle = lightSensors.getLineAngle();

    float compassRot = imuPID.update(heading, 0.0f);
    float attackGoalRot = getAttackGoalRot();
    float attackRot = (camera.cameraData.attackGoalDist == 0.0f) ? compassRot : attackGoalRot;

    if (absoluteLineAngle != -1.0f) {
        float avoidAngle = floatMod(absoluteLineAngle + 180.0f - heading, 360.0f);
        motors.move(35.0f, avoidAngle, attackRot, heading);
    } else {
        motors.move(orbitSpeed, absoluteOrbitAngle, attackRot, heading);        
    }
    
    // float lineSize = lightSensors.getLineSize();
    // Serial.printf("lineAngle: %.2f\tlineSize: %.2f\tavoidAngle: %.2f", lineAngle, lineSize, avoidAngle);
    // Serial.println();

    #if DEBUG_IMU
    Serial.printf("Heading: %.2f\tCorrection: %.2f\tMode: %d", heading, compassRot, (uint8_t)bno.getMode());
    #endif

    #if DEBUG_CAMERA
    Serial.printf("ballAngle: %.2f\tattackGoalAngle: %.2f\tdefendGoalAngle: %.2f\tFPS: %d", camera.cameraData.ballAngle, camera.cameraData.attackGoalAngle, camera.cameraData.defendGoalAngle, camera.cameraData.fps);
    #endif

    #if DEBUG_ORBIT
    Serial.printf("ballAngle: %.2f\tballDist: %.2f\tabsoluteOrbitAngle: %.2f\torbitSpeed: %.2f", absoluteBallAngle, absoluteAttackGoalAngle, absoluteOrbitAngle, orbitSpeed);
    #endif

    #if DEBUG_GOAL_CORRECT
    Serial.printf("attackGoalRot: %.2f\tdefendGoalRot: %.2f", attackGoalRot, defendGoalRot);
    #endif

    #if DEBUG_LS
    Serial.printf("absoluteLineAngle: %.2f\tavoidAngle: %.2f\tlineSize: %.2f\tavoidSpeed: %.2f", absoluteLineAngle, avoidAngle, lineSize, avoidSpeed);
    #endif

    #if DEBUG
    Serial.println();
    #endif
}