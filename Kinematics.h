#ifndef KINEMATICS_H
#define KINEMATICS_H
#include <MultiStepper.h>

//Link lengths (mm)
#define L_Bicep 130
#define L_Forearm 180
#define L3 20

#define R_BASE 65
#define R_WRIST 35
#define SERVO_OFFSET_Z 20

#define SERVO_ANGLE_MIN 1.91986f //110 degrees
#define SERVO_ANGLE_MAX 3.14159f //180 degrees

#define HOME_ANGLE_RAD 1.91986f //110 degrees
#define DEG_TO_STEP 9 // 9 steps to one degre (need to check) 8.888889 actually

/*------------------------------------------------------------------------------------------------------------------------------------------------------*/

struct Coordinate_f {
    float x;
    float y;
    float z;
};

/*------------------------------------------------------------------------------------------------------------------------------------------------------*/

float inverse_kinematics(float, float, float);
void linear_move(float, float, float, float, long int*, MultiStepper*);
void updateEndEffector(Coordinate_f, float, float, float);
#endif
