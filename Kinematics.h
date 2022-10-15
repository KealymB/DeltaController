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
#define RAD2STEP 1018.59163579f   //6400 steps / 2*pi rad

#define ANGLE_MIN 1.9025f         //90 degrees
#define ANGLE_MAX 4.18879f        //240 degrees

#define LIFT_HEIGHT 165.0f        //Height in mm to lift pen off surface
#define DRAW_HEIGHT 160.5f        //Height in mm to put pen on surface
#define Z_OFFSET 32.0f              //deffirence in height between base rotation points

#define COS120 cos(2*PI/3)
#define SIN120 sin(2*PI/3)
/*------------------------------------------------------------------------------------------------------------------------------------------------------*/

struct Coordinate_f {
    float x;
    float y;
    float z;
};

/*------------------------------------------------------------------------------------------------------------------------------------------------------*/

float inverse_kinematics(float, float, float);
void linear_move(float, float, float, float, long int*, MultiStepper*);
void cubic_bezier(float*, float*, float*, float*, long int*, MultiStepper*);
void pen_lift(long int*, MultiStepper*);
void pen_drop(long int*, MultiStepper*);
void updateEndEffector(float, float, float);

//utils
float vectorDist(float*, float*);
float bezierLength(float*, float*, float*, float*);
float linearInterp(float, float, float);
#endif
