#ifndef CONFIG_H
#define CONFIG_H

/* ROBOT */
#define ROBOT_1 false
#define ROBOT_2 (!ROBOT_1) // attacker
// #define ATTACK true

/* ATTACK DIRECTION */
#define BLUE_GOAL_ATTACK false
// True: Attacking Blue Goal (Defending Yellow Goal)
// False: Attacking Yellow Goal (Defending Blue Goal)

/* DEBUG */
#define DEBUG_IMU false
#define DEBUG_CAMERA false
#define DEBUG_ORBIT false
#define DEBUG_GOAL_CORRECT false
#define DEBUG_LS false
#define DEBUG (DEBUG_IMU || DEBUG_CAMERA || DEBUG_ORBIT || DEBUG_GOAL_CORRECT || DEBUG_LS)

#endif