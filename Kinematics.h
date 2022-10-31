#ifndef KINEMATICS_H
#define KINEMATICS_H
#include <MultiStepper.h>

//Link lengths (mm)
#define L_Bicep 130
#define L_Forearm 185.0f

#define R_BASE 65.122
#define R_WRIST 19.5
#define PEN_OFFSET 62            // Measured from tip of pen to center of ball joints on wrist

#define HOME_ANGLE_RAD 1.9025f    //109 degrees
#define RAD2STEP 2037.18327158f   //6400 steps / 2*pi rad = 1018.59163579f | 12800 steps / 2*pi rad = 2037.18327158

#define XMAX 50
#define YMAX 50

#define ANGLE_MIN 1.6f         //90 degrees
#define ANGLE_MAX 4.18879f        //240 degrees

#define Z_OFFSET 78.0f            //difference in height between base and drawing plane

#define COS120 cos(2*PI/3)
#define SIN120 sin(2*PI/3)

#define MESHADDRESS 0

/*------------------------------------------------------------------------------------------------------------------------------------------------------*/

struct Coordinate_f {
    float x;
    float y;
    float z;
};

struct vert {
    float A;
    float B;
    float C;
    float D;
};

/*------------------------------------------------------------------------------------------------------------------------------------------------------*/

float inverse_kinematics(float, float, float);
void linear_move(float, float, float, float, long int*, MultiStepper*);
void cubic_bezier(float*, float*, float*, float*, float, long int*, MultiStepper*);
void zJog(float, long int*, MultiStepper*);
void updateEndEffector(float, float, float);
void calculateActuatorAngles(float, float, float);
void setMeshHeights(float, float, float, float);
void readMeshHeights();

//utils
float vectorDist(float*, float*);
float bezierLength(float*, float*, float*, float*);
float linearInterp(float, float, float);
float mesh_level(float, float);
#endif
