#include "Kinematics.h"
#include <math.h>
#include <MultiStepper.h>

float M1_angle = HOME_ANGLE_RAD;
float M2_angle = HOME_ANGLE_RAD;
float M3_angle = HOME_ANGLE_RAD;

Coordinate_f end_effector = {0.0, 0.0, 286.0}; // starting coords

/*------------------------------------------------------------------------------------------------------------------------------------------------------*/

void updateEndEffector(struct Coordinate_f *end_effector, float x, float y, float z){
  end_effector -> x = x;
  end_effector -> y = y;
  end_effector -> z = z;
}

/*------------------------------------------------------------------------------------------------------------------------------------------------------*/

float inverse_kinematics(float xt, float yt, float zt){
  /*    
   * Takes in end effector coords (x,y,z)
   * Returns the angle of the stepper motor in rad
   * Returns -1 if theta the angle is not reachable
   */
    float arm_end_y = yt + R_WRIST; //added wrist radius to move center to edge
    float l2_YZ = sqrt(sq(L_Forearm) - sq(xt));//The length of the bicep when projected onto the YZ plane

    float l2_angle = asin(yt/L_Forearm*1.0); //gives the angle of ball joints, can be used to stop motion
    
    float ext = sqrt(sq(zt) + sq(R_BASE - arm_end_y)); //Extension of the arm from the centre of the servo rotation to the end ball joint of link2

    float test = (sq(L_Bicep) + sq(ext) - sq(l2_YZ)) / (2.0 * L_Bicep * ext);
    
    float phi = acos((sq(L_Bicep) + sq(ext) - sq(l2_YZ)) / (2.0 * L_Bicep * ext)); // Cosine rule that calculates the angle between the ext line and L1
    float omega = -atan2(zt, R_BASE - arm_end_y); //Calculates the angle between horizontal (Y) the ext line with respect to its quadrant
    float theta = phi + omega; //Theta is the angle between horizontal (Y) and the bicep

    if(!(theta >= ANGLE_MIN && theta <= ANGLE_MAX)){ //Checks the angle is in the reachable range
      return -1.0;
    }
    
    return theta;
}

/*------------------------------------------------------------------------------------------------------------------------------------------------------*/

void linear_move(float x1, float y1, float z1, float stepDist, long *positions, MultiStepper *actuators){
  /*
   * Takes in desired end effector coords, and interpolation spacing
   * moves end effector to desired coords in a straight line
   */
    //Sets the initial position variables
    float x0 = end_effector.x;
    float y0 = end_effector.y;
    float z0 = end_effector.z;
    
    //Distance change in each axis
    float xDist = x1 - x0;
    float yDist = y1 - y0;
    float zDist = z1 - z0;
    
    float totalDist = sqrt(sq(xDist) + sq(yDist) + sq(zDist));  //Absolute magnitude of the distance
    int numberOfSteps = round(totalDist / stepDist);            //Number of steps required for the desired step distance
    
    float xStep = xDist / numberOfSteps;
    float yStep = yDist / numberOfSteps;
    float zStep = zDist / numberOfSteps;

    //Interpolation variables
    float xInterp = 0.0;
    float yInterp = 0.0;
    float zInterp = 0.0;

    //Error accumulator for each stepper position
    float errorAccumulator[] = {0.0, 0.0, 0.0};

    for(int i = 1; i <= (int) numberOfSteps; i++){
        xInterp = x0 + i * xStep;
        yInterp = y0 + i * yStep;
        zInterp = z0 + i * zStep;

        float th1 = inverse_kinematics(xInterp, yInterp, -zInterp);
        float th2 = inverse_kinematics(xInterp*cos(2*PI/3) - yInterp*sin(2*PI/3), yInterp*cos(2*PI/3) + xInterp*sin(2*PI/3), -zInterp);
        float th3 = inverse_kinematics(xInterp*cos(2*PI/3) + yInterp*sin(2*PI/3), yInterp*cos(2*PI/3) - xInterp*sin(2*PI/3), -zInterp);

        if(th1 < 0 || th2 < 0 || th3 < 0){
          Serial.println("E1-Commanded angle is out of bounds, disregarding move");
          break;
        }
        
        // find difference in angle to figure out how much the must move
        float M1_D = M1_angle - th1;
        float M2_D = M2_angle - th2;
        float M3_D = M3_angle - th3;

        M1_angle = th1;
        M2_angle = th2;
        M3_angle = th3;

        float truePos[] = {M1_D*RAD2STEP, M2_D*RAD2STEP, M3_D*RAD2STEP}; //convert angle in rad into steps

        //accumulate errors
        for (int index = 0; index < 3; index++){
          long roundedPos = (long) truePos[index];                //round position
          errorAccumulator[index] += truePos[index] - roundedPos; //accumulate diffeernce between rounded position and true position
          
          if(abs(errorAccumulator[index]) >= 1.0f){               //empty accumulator if error is larger than one step
            positions[index] -= (int) errorAccumulator[index];
            errorAccumulator[index] -= (errorAccumulator[index] > 0.0f ? 1.0f : -1.0f);
          }
          
          positions[index] -= roundedPos;
        }
        
        //move to new position
        actuators->moveTo(positions);
        actuators->runSpeedToPosition();
    }

    updateEndEffector(&end_effector, x1, y1, z1);
}
