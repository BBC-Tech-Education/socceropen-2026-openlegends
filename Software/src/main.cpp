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
static float rotation = 0.0f;

static float side = 0.0f;
static float fwd = 0.0f;

static float fieldLineAngle = 0.0f;
static float fieldLineSize = 0.0f;

static float attackRot = 0.0f;
static float defendRot = 0.0f;

#if ATTACK
static float searchTarget = 135.0f;
#else
static float searchTarget = 90.0f;
#endif

bool surge = false;
#define SURGE_LENGTH 100

unsigned long lastTimeBallSeen = millis();
unsigned long lastTimeLineSeen = millis();

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

void getOrbitMovement() { // call function only if ball visible
    float absoluteBallAngle = (camera.cameraData.ballDist != 0.0f ? floatMod(camera.cameraData.ballAngle + heading, 360.0f) : 0.0f);
    float absoluteAttackGoalAngle = floatMod(camera.cameraData.attackGoalAngle + heading, 360.0f);

    float targetAngle = ((camera.cameraData.attackGoalDist == 0.0f) ? 0.0f : absoluteAttackGoalAngle);
    // float targetAngle = 0.0f;
    // Serial.println(camera.cameraData.attackGoalDist);

    float dir = floatMod(absoluteBallAngle - targetAngle, 360.0f);

    dir = (dir > 180.0f) ? (dir - 360.0f) : dir; // -180 to 180
    float ballAngleDifference = fsign(dir) * fminf(90.0f, 0.4f * expf(0.25f * smallestAngleBetween(absoluteBallAngle, 0.0f)));
    float strengthFactor = constrain(ATTACK_CLOSE_DISTANCE / camera.cameraData.ballDist, 0.0f, 1.0f);
    float angleAddition = ballAngleDifference * strengthFactor;

    moveAngle = floatMod((absoluteBallAngle + angleAddition), 360.0f);
    moveSpeed = ATTACK_SLOW_SPEED + (ATTACK_FAST_SPEED - ATTACK_SLOW_SPEED) * (1.0f - fabsf(angleAddition/90.0f));

    if (camera.cameraData.ballDist == 0.0f) {
        if ((millis() - lastTimeBallSeen) > 100) {
            moveAngle = -1.0f;
            moveSpeed = 0.0f;
        }
    } else if ((smallestAngleBetween(absoluteBallAngle, targetAngle) <= 10.0f)) {
        moveAngle = absoluteBallAngle;
        if ((camera.cameraData.ballDist < ATTACK_SURGE_DISTANCE) && (smallestAngleBetween(absoluteBallAngle, heading) <= 6.0f)) {
            Serial.println("1");
            moveSpeed = ATTACK_SURGE_SPEED + 25.0f;
        } 
        else {
            moveSpeed = 22.5f;
        }
    }

    rotation = imuPID.update(heading, 0.0f);
    // Serial.println(smallestAngleBetween(absoluteBallAngle, targetAngle));

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

void defend() {
    float absoluteBallAngle = floatMod(camera.cameraData.ballAngle + heading, 360.0f);
    float absDefGoalAngle = floatMod(camera.cameraData.defendGoalAngle + heading, 360.0f);
    float in;

    if (camera.cameraData.defendGoalDist != 0.0f) {
        in = floatMod(heading - absDefGoalAngle + 180.0f, 360.0f);

        if (in > 180.0f) {
            in -= 360.0f;
        }
        rotation =  defendGoalTrackPID.update(in, 0.0f);
    } else {
        rotation = imuPID.update(heading, 0.0f);
    }

    if (absoluteBallAngle > 180.0f) {
        absoluteBallAngle -= 360.0f;
    }

    in = absoluteBallAngle - absDefGoalAngle + 180.0f;

    if (in > 180.0f) {
        in -= 360.0f;
    }

    if (camera.cameraData.ballDist != 0.0f && (fabs(in) > 90.0f &&  fieldLineSize == -1.0f || ((camera.cameraData.ballAngle > 340.0f || camera.cameraData.ballAngle < 20.0f) && camera.cameraData.ballDist < 40.0f))) {
        getOrbitMovement();
    } else {
        if (camera.cameraData.ballDist != 0.0f) {
            if (fabs(in) > 90.0f) {
                in = floatMod(180.0f - in, 360.0f);
                if (in > 180.0f) {
                    in -= 360.0f;
                }
            }
            side = -defendHorizontalPID.update(in, 0.0f);
        } else {
            if ((millis() - lastTimeBallSeen) > 100) {
                side = 0.0f;
            }
        }

        // 0.0f;//defendHorizontalPID.update(in1, 0.0f);

        // Serial.println(side);
        fwd = -defendVerticalPID.update(fieldLineSize, 0.25f);
        moveSpeed = sqrtf((side * side) + (fwd * fwd));
        moveAngle = floatMod((450.0f - atan2f(fwd, side) * RAD_TO_DEG_F), 360.0f);

        if (camera.cameraData.defendGoalDist != 0.0f) {
            moveAngle = floatMod(moveAngle + absDefGoalAngle + 180.0f, 360.0f);
        }
    }

    // Serial.printf("defendSpeed: %.2f\tdefendAngle: %.2f", defendSpeed, defendAngle);
    // Serial.println();
}

void loop() {

    getHeading();
    camera.update();
    lightSensors.update(heading);
    fieldLineAngle = lightSensors.getLineAngle();
    fieldLineSize = lightSensors.getLineSize();

    if (camera.cameraData.ballDist != 0.0f) {
        lastTimeBallSeen = millis();
    }

    

    

    if ((millis() - lastTimeBallSeen) > 1000) {
        #if ATTACK

        if (fieldLineSize >= 0.0f && (millis() - lastTimeLineSeen) > 1000) {
            searchTarget = floatMod(searchTarget + 90.0f, 360.0f);
            lastTimeLineSeen = millis();
        }
        
        moveAngle = searchTarget;
        moveSpeed = 20.0f;
        rotation = imuPID.update(heading, 0.0f);

        #else
        if (camera.cameraData.defendGoalDist == 0.0f && fieldLineSize != -1.0f) {
            if (smallestAngleBetween(fieldLineAngle, searchTarget) < 45.0f) {
                if (searchTarget == 90.0f) {
                    searchTarget = 270.0f;
                } else {
                    searchTarget = 90.0f;
                }
            }

            moveAngle = searchTarget;
            moveSpeed = 20.0f;

            float diff = floatMod(moveAngle - fieldLineAngle, 360.0f);
            if (diff > 180.0f) {
                diff -= 360.0f;
            }
            if (fabs(diff) < 90.0f) {
                if (diff > 0.0f) {
                    moveAngle = floatMod(fieldLineAngle + 90.0f, 360.0f);
                } else {
                    moveAngle = floatMod(fieldLineAngle - 90.0f, 360.0f);
                }

                moveSpeed = fabs(sinf(diff)) * 40.0f;
            }
            rotation = imuPID.update(heading, 0.0f);
        } else {
            if (camera.cameraData.defendGoalDist != 0.0f) {
                float absDefGoalAngle = floatMod(camera.cameraData.defendGoalAngle + heading, 360.0f);
                float in = floatMod(heading - absDefGoalAngle + 180.0f, 360.0f);
                if (in > 180.0f) {
                    in -= 360.0f;
                }
                rotation =  defendGoalTrackPID.update(in, 0.0f);    
                side = defendHorizontalPID.update(absDefGoalAngle, 180.0);     
            } else {
                side = 0;
                rotation = imuPID.update(heading, 0.0f);
            }
            fwd = -defendVerticalPID.update(fieldLineSize, 0.25f);
            moveSpeed = sqrtf((side * side) + (fwd * fwd));
            moveAngle = floatMod((450.0f - atan2f(fwd, side) * RAD_TO_DEG_F), 360.0f);
        }
        
        #endif

    } else {
        #if ATTACK
        getOrbitMovement();
        #else
        defend();
        #endif
    }



    #if ATTACK
    if (fieldLineSize > 0.75f) {
    #else
    if (fieldLineSize > 1.0f) {
    #endif

        moveAngle = floatMod(fieldLineAngle + 180.0f, 360.0f);
        moveSpeed = 50.0f;
    }
    
    #if ATTACK
    
    
    else if (fieldLineSize >= 0.0f) {
        float diff = floatMod(moveAngle - fieldLineAngle, 360.0f);
        if (diff > 180.0f) {
            diff -= 360.0f;
        }

        if (fabs(diff) < 90.0f) {
            if (diff > 0.0f) {
                moveAngle = floatMod(fieldLineAngle + 90.0f, 360.0f);
            } else {
                moveAngle = floatMod(fieldLineAngle - 90.0f, 360.0f);
            }

            moveSpeed = fabs(sinf(diff)) * 40.0f;
        }
    }

    #endif


    // if (digitalRead(COMM)) {
        motors.move(moveSpeed, moveAngle, rotation, heading);
    // } else {
    //     motors.move(0.0f, 0.0f, 0.0f, 0.0f);
    // }












































    
    // float rawLineAngle = floatMod(absoluteLineAngle - heading, 360.0f);
    // float avoidAngle = floatMod(absoluteLineAngle + 180.0f, 360.0f);
    // float avoidSpeed = 70.0f;

    


    // if (ROBOT_1 || ROBOT_2) { // Attacker
    //     // float attackGoalRot = getAttackGoalRot();
    //     // attackRot = (camera.cameraData.attackGoalDist > 100.0f) ? attackGoalRot : compassRot; //check if it is 50 outside goal box
    //     if (absoluteLineAngle != -1.0f) { // sees line: out avoid
    //         Serial.println("out avoid");
    //         moveSpeed = avoidSpeed;
    //         moveAngle = avoidAngle;
    //     } else if (camera.cameraData.ballDist != 0.0f) { // sees ball: orbit normally, store timer
    //         Serial.println("orbit");
    //         timer = millis();
    //         moveSpeed = orbitSpeed;
    //         moveAngle = orbitAngle;
    //     } else { // ball not visible: measure time since last seen
    //         timeSince = millis() - timer;

    //         if (timeSince < 5000) {
    //             Serial.println("stop");
    //             moveSpeed = 0.0f;
    //             moveAngle = -1.0f;
    //         } else {
    //             Serial.println("searching");
    //             moveSpeed = 20.0f;
    //             moveAngle = 0.0f;
    //             // add searching code
    //         }
    //     }
    // }

    // if (digitalRead(COMM) == HIGH) {
    //     motors.move(moveSpeed, moveAngle, compassRot, heading); // attackRot            
    // } else {
        // if (ROBOT_1) {
        //     motors.move(moveSpeed, moveAngle, compassRot, heading); // attackRot
        //     // motors.move(moveSpeed, moveAngle, compassRot, heading); // attackRot 
        // } else { // check this is the robot that connects to comm module
        //     motors.move(0.0f, 0.0f, 0.0f, heading); // attackRot
        //     // motors.move(moveSpeed, moveAngle, compassRot, heading); // attackRot 
        // }
    //     motors.move(0.0f, 0.0f, 0.0f, heading);
    // }

    // Serial.println(camera.cameraData.ballAngle);
    // else{ //Defender
    //     defend(fieldLineSize);
    //     if(camera.cameraData.ballAngle != -1.0f){
    //         if (absoluteLineAngle == -1.0f && !surge){ //coming back from surge
    //             Serial.print("bitch");
    //             moveSpeed = orbitSpeed;
    //             if(camera.cameraData.ballAngle > 90.0f && camera.cameraData.ballAngle < 270.0f){
    //                 moveAngle = orbitAngle;
    //             } else{
    //                 moveAngle = 180.0f;
    //             }
    //         } else if (absoluteLineAngle > 180.0f && absoluteLineAngle < 270.0f){ //right hand side of goal box
    //             if (camera.cameraData.ballAngle < 180.0f && camera.cameraData.ballAngle > 20.0f){
    //                 moveSpeed = 0.0f;
    //                 moveAngle = -1.0f;
    //             } else{
    //                 moveSpeed = defendSpeed;
    //                 moveAngle = defendAngle;
    //             }
    //         } else if (absoluteLineAngle > 90.0f && absoluteLineAngle < 180.0f){ //left hand side of goalbox
    //             if (camera.cameraData.ballAngle > 180.0f && camera.cameraData.ballAngle < 340.0f){
    //                 moveSpeed = 0.0f;
    //                 moveAngle = -1.0f;
    //             } else{
    //                 moveSpeed = defendSpeed;
    //                 moveAngle = defendAngle;
    //             }
    //         } 
    //         // else if (camera.cameraData.defendGoalDist > 165.0f && surge) { //finished surging?
    //         //     surge = false;
    //         // } 
    //         // else if (camera.cameraData.ballDist < 15.0f && camera.cameraData.defendGoalDist < 165.0f){ // can surge (can use defending goal distance rather lineagle)
    //         //     // Serial.print("SURGE");
    //         //     moveAngle = floatMod(camera.cameraData.ballAngle + heading, 360.0f);
    //         //     moveSpeed = 20.0f;
    //         //     surge = true;
    //         // } 
    //         else{ //defend
    //             // Serial.print("DEFEND");
    //             moveAngle = defendAngle;
    //             moveSpeed = defendSpeed;
    //         }
    //     } else{
    //         // Serial.print("not");
    //         moveAngle = -1.0f;
    //         moveSpeed = 0.0f;
    //     }     
        
    //     /////ADDD COMMM MODULE
    //     // Serial.print(surge);                        Serial.print("\t");
    //     // Serial.print(camera.cameraData.ballAngle);  Serial.print("\t");
    //     // Serial.print(camera.cameraData.ballDist);   Serial.print("\t");
    //     // Serial.println(absoluteLineAngle);

    //     if (digitalRead(COMM) == HIGH) {
    //         motors.move(moveSpeed, moveAngle, compassRot, heading); // attackRot            
    //     } else {
    //         motors.move(0.0f, 0.0f, compassRot, heading); // attackRot
    //         // motors.move(moveSpeed, moveAngle, attackRot, heading); // attackRot 
    //     }

    // }










    // if (smallestAngleBetween(180.0f, absoluteLineAngle) < 45.0f){ //change 45 for curve of the goalbox
            //     moveSpeed = 0.0f;
            // } else 

    // if (fieldLineSize > 0.1f) {
        //     moveSpeed = avoidSpeed;
        //     moveAngle = avoidAngle;
        // } 

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