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
PID defendGoalTrackPID(DEFEND_GOAL_TRACK_KP, DEFEND_GOAL_TRACK_KI, DEFEND_GOAL_TRACK_KD, DEFEND_GOAL_TRACK_MAX);
PID defendHorizontalPID(HORIZONTAL_KP, HORIZONTAL_KI, HORIZONTAL_KD, HORIZONTAL_MAX);
PID defendVerticalPID(VERTICAL_KP, VERTICAL_KI, VERTICAL_KD, VERTICAL_MAX);

static float heading = 0.0f;
static float moveSpeed = 0.0f;
static float moveAngle = 0.0f;

static float side = 0.0f;
static float fwd = 0.0f;

static float fieldLineSize = 0.0f;

static float attackRot = 0.0f;
static float defendRot = 0.0f;

static float orbitSpeed = 0.0f;
static float orbitAngle = 0.0f;

static float defendSpeed = 0.0f;
static float defendAngle = 0.0f;

void setup() {
    delay(100);

    Serial.begin(9600);

    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, HIGH);

    pinMode(LS_PWM, OUTPUT);
    pinMode(COMM, INPUT);

    analogWrite(LS_PWM, DIMMING); // old 75, new 150
    delay(100);
    lightSensors.init();
    // pinMode(14, INPUT); //batt volt reader
    // pinMode(CALI, INPUT);

    motors.init();
    camera.init();

    while (!bno.begin(OPERATION_MODE_IMUPLUS)) { 
        Serial.println("BNO not working");
        delay(1000);
    }

    delay(500);
    bno.setExtCrystalUse(true);
    delay(500);

    digitalWrite(LED_BUILTIN, LOW);
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

float getDefendGoalRot() { // Relative not absolute (rotating doesn't change goal angle)
    float defendGoalAngle = floatMod(camera.cameraData.defendGoalAngle, 360.0f);
    Serial.println(defendGoalAngle);
    float defendGoalRot = -defendGoalTrackPID.update(defendGoalAngle, 180.0f);
    return defendGoalRot;
}

void getOrbitMovement() { // call function only if ball visible
    float absoluteBallAngle = (camera.cameraData.ballDist != 0.0f ? floatMod(camera.cameraData.ballAngle + heading, 360.0f) : 0.0f);
    float absoluteAttackGoalAngle = floatMod(camera.cameraData.attackGoalAngle + heading, 360.0f);
    // make sure attack goal dist correct (not cm)

    // 100 halfway
    // second one when goal not visible
    // float targetAngle = (camera.cameraData.attackGoalDist > 100 || camera.cameraData.attackGoalDist < 10) ? 0.0f : absoluteAttackGoalAngle;
    float targetAngle = 0.0f;

    float dir = floatMod(absoluteBallAngle - targetAngle, 360.0f);

    dir = (dir > 180.0f) ? (dir - 360.0f) : dir; // -180 to 180
    float ballAngleDifference = fsign(dir) * fminf(90.0f, 0.4f * expf(0.25f * smallestAngleBetween(absoluteBallAngle, 0.0f)));
    float strengthFactor = constrain(ATTACK_CLOSE_DISTANCE / camera.cameraData.ballDist, 0.0f, 1.0f);
    float angleAddition = ballAngleDifference * strengthFactor;

    orbitAngle = floatMod((absoluteBallAngle + angleAddition), 360.0f);
    orbitSpeed = ATTACK_SLOW_SPEED + (ATTACK_FAST_SPEED - ATTACK_SLOW_SPEED) * (1.0f - fabsf(angleAddition/90.0f));

    if (camera.cameraData.ballDist == 0.0f){
        orbitAngle = -1.0f;
        orbitSpeed = 0.0f;
    } else if ((smallestAngleBetween(absoluteBallAngle, heading) <= 10.0f) && (camera.cameraData.ballDist < ATTACK_SURGE_DISTANCE)) {
        Serial.println("ol");
        orbitAngle = absoluteBallAngle;
        orbitSpeed = ATTACK_SURGE_SPEED + 20.0f;
    }
    Serial.printf("%f %f", camera.cameraData.ballAngle, orbitAngle);
    Serial.println();
    // increase fast speed undershoot more
    // decrease fast speed overshoot more
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

void defend(float fieldLineSize) {
    float absoluteBallAngle = floatMod(camera.cameraData.ballAngle + heading, 360.0f);

    if (absoluteBallAngle > 180.0f) {
        absoluteBallAngle -= 360.0f;
    }

    side = defendHorizontalPID.update(absoluteBallAngle, 0.0f);
    
    fwd = defendVerticalPID.update(fieldLineSize, 1.0f);
    defendSpeed = sqrtf(side * side + fwd * fwd);
    defendAngle = atan2f(side, fwd) * RAD_TO_DEG_F;
    // Serial.printf("defendSpeed: %.2f\tdefendAngle: %.2f", defendSpeed, defendAngle);
}

void loop() {
    getHeading();
    camera.update();
    lightSensors.update(heading);
    fieldLineSize = lightSensors.getLineSize();

    float compassRot = imuPID.update(heading, 0.0f);
    float absoluteLineAngle = lightSensors.getLineAngle();
    float avoidAngle = floatMod(absoluteLineAngle + 180.0f, 360.0f);
    float avoidSpeed = 60.0f;
    getOrbitMovement();

    if (ROBOT_2) { // Attacker
        float attackGoalRot = getAttackGoalRot();
        attackRot = (camera.cameraData.attackGoalDist > 50.0f) ? attackGoalRot : compassRot; //check if it is 50 outside goal box
        if (camera.cameraData.ballDist <= 5.0f){ //don't see ball
            if (smallestAngleBetween(absoluteLineAngle, 180.0f) < 45.0f){ //change 45 for curve of the goalbox
                moveSpeed = 0.0f;
            } else if (absoluteLineAngle != -1.0f){
                moveAngle = avoidAngle;
                moveSpeed = avoidSpeed;
            } else if (camera.cameraData.defendGoalDist < 50.0f && camera.cameraData.defendGoalDist > 10.0f) { // echeck 50
                moveSpeed = 40.0f; 
                moveAngle = floatMod(camera.cameraData.defendGoalDist + heading, 360.0f);
            } else {
                moveSpeed = 20.0f; 
                moveAngle = 180.0f;
            }
        } else if (fieldLineSize > 0.4f) { // other side of line
            moveSpeed = avoidSpeed;
            moveAngle = avoidAngle;
        } else if (absoluteLineAngle != -1.0f){ //see a line but safe
            moveSpeed = sqrtf(avoidSpeed * avoidSpeed + orbitSpeed * orbitSpeed);
            moveAngle = atan2f(avoidAngle, orbitAngle) * RAD_TO_DEG_F;
        } else { // Doesn't see line
            moveSpeed = orbitSpeed;
            moveAngle = orbitAngle;
        }
        if (digitalRead(COMM) == HIGH) {
            motors.move(moveSpeed, moveAngle, compassRot, heading); // attackRot            
        } else {
            // motors.move(0.0f, 0.0f, compassRot, heading); // attackRot
            motors.move(moveSpeed, moveAngle, compassRot, heading); // attackRot 
        }
    }


















    // else { // Defender
    //     defend(fieldLineSize);
    //     float defendGoalRot = getDefendGoalRot();
    //     defendRot = (camera.cameraData.defendGoalDist <= 15.0f) ? compassRot : defendGoalRot; //9.5f
    //     if (digitalRead(COMM) == HIGH) {
    //         motors.move(side, defendAngle, compassRot, heading); // attackRot
    //     } else {
    //         // motors.move(0.0f, 0.0f, compassRot, heading); // attackRot

    //         motors.move(0.0f, 0.0f, defendRot, heading); // attackRot
    //     }
    // }

    // Serial.println(moveSpeed);

    // else {
    //     defend(fieldLineSize);
    //     float defendGoalRot = getDefendGoalRot();
    //     defendRot = (camera.cameraData.defendGoalDist == 0.0f) ? compassRot : defendGoalRot;
    // }
    // motors.move(moveSpeed, moveAngle, attackRot, heading); // attackRot
    

    // // Serial.println(digitalRead(CALI));

    // if (ATTACK) {
    //     float attackGoalRot = getAttackGoalRot();
    //     attackRot = (camera.cameraData.attackGoalDist == 0.0f) ? compassRot : attackGoalRot;
    //     // float attackRot = compassRot;
    //     if (absoluteLineAngle != -1.0f) {
    //         float avoidAngle = floatMod(absoluteLineAngle + 180.0f, 360.0f);
    //         float avoidSpeed = 15.1509f * fieldLineSize + 18.54608;
    //         if (camera.cameraData.ballAngle == -1.0f) {
    //             attackSpeed = avoidSpeed;
    //             attackAngle = avoidAngle;
    //         } else {
    //             attackSpeed = sqrtf(avoidSpeed * avoidSpeed + orbitSpeed * orbitSpeed);
    //             attackAngle = atan2f(avoidAngle, absoluteOrbitAngle) * RAD_TO_DEG_F;
    //         }
    //     } else {
    //         if (camera.cameraData.ballAngle == -1.0f) {
    //             attackSpeed = 0.0f;
    //             attackAngle = 0.0f;
    //         } else {
    //             attackSpeed = orbitSpeed;
    //             attackAngle = absoluteOrbitAngle;
    //         }
    //     }
    // } else {
    //     defend(fieldLineSize);
    //     float defendGoalRot = getDefendGoalRot();
    //     defendRot = (camera.cameraData.defendGoalDist == 0.0f) ? compassRot : defendGoalRot;
    //     // float defendRot = compassRot;
    // }
    // if (ATTACK) {
    //     motors.move(attackSpeed, attackAngle, attackRot, heading);
    // }

    // motors.move(0.0f, 0.0f, attackRot, heading);

    // Serial.println(attackAngle);

    // if (COMM) {
    //     if(ATTACK){
    //         motors.move(attackSpeed, attackAngle, attackRot, heading);
    //     } else{
    //         motors.move(defendSpeed, defendAngle, defendRot, heading);
    //     }
    // } else{
    //     motors.move(0.0f, 0.0f, compassRot, heading);
    // }
    
    // motors.move(0.0f, 0.0f, compassRot, heading);


    // motors.move(side, floatMod(270.0f + heading, 360.0f), compassRot, heading);
    
    // Serial.printf("lineAngle: %.2f\tlineSize: %.2f\tavoidAngle: %.2f", lineAngle, lineSize, avoidAngle);
    // Serial.println();

    // #if DEBUG_IMU
    // Serial.printf("Heading: %.2f\tCorrection: %.2f\tMode: %d", heading, compassRot, (uint8_t)bno.getMode());
    // #endif

    // #if DEBUG_CAMERA
    // Serial.printf("ballAngle: %.2f\tattackGoalAngle: %.2f\tdefendGoalAngle: %.2f\tFPS: %d", camera.cameraData.ballAngle, camera.cameraData.attackGoalAngle, camera.cameraData.defendGoalAngle, camera.cameraData.fps);
    // #endif

    // #if DEBUG_ORBIT
    // Serial.printf("ballAngle: %.2f\tballDist: %.2f\tabsoluteOrbitAngle: %.2f\torbitSpeed: %.2f", absoluteBallAngle, absoluteAttackGoalAngle, absoluteOrbitAngle, orbitSpeed);
    // #endif

    // #if DEBUG_GOAL_CORRECT
    // Serial.printf("attackGoalRot: %.2f\tdefendGoalRot: %.2f", attackGoalRot, defendGoalRot);
    // #endif

    // #if DEBUG_LS
    // float avoidSpeed = 30.0f;
    // Serial.printf("absoluteLineAngle: %.2f\tavoidAngle: %.2f\tlineSize: %.2f\tavoidSpeed: %.2f", absoluteLineAngle, avoidAngle, lineSize, avoidSpeed);
    // #endif

    // #if DEBUG
    // Serial.println();
    // #endif
}