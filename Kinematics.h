#ifndef KINEMATICS_H
#define KINEMATICS_H
#include <MultiStepper.h>

//Link lengths (mm)
#define L_Bicep 130
#define L_Forearm 185.0f
#define L3 20

#define R_BASE 65
#define R_WRIST 20
#define SERVO_OFFSET_Z 20

#define HOME_ANGLE_RAD 1.9025f    //109 degrees
#define RAD2STEP 254.647908947f   // = 3200 steps / 2pi rad

#define ANGLE_MIN 1.9025f         //90 degrees
#define ANGLE_MAX 4.18879f        //240 degrees
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
