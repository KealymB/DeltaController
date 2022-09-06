#include <AccelStepper.h>
#include <MultiStepper.h>
#include <TMCStepper.h>
#include "Kinematics.h"
/*------------------------------------------------------------------------------------------------------------------------------------------------------*/
// Motors
#define EN_PIN        12
// M1 
#define M1_STEP_PIN   11
#define M1_DIR_PIN    10
// M2 
#define M2_STEP_PIN   8
#define M2_DIR_PIN    7
// M3 
#define M3_STEP_PIN   5
#define M3_DIR_PIN    4

// MicroSwitch
#define MS1           13
#define MS2           9
#define MS3           6

// Driver
#define SW_RX 3 
#define SW_TX 2
#define R_SENSE 0.11f //make sure this is correct
// Motor Params
#define MAXSPEED 150
// Kinematic Params (in mm, origin in center of delta)
#define X_BOUND_MIN -50
#define X_BOUND_MAX 50
#define Y_BOUND_MIN -50
#define Y_BOUND_MAX 50
#define Z_BOUND_MIN 50
#define Z_BOUND_MAX 290

/*------------------------------------------------------------------------------------------------------------------------------------------------------*/

TMC2208Stepper driver = TMC2208Stepper(SW_RX, SW_TX, R_SENSE); // create TMC2208 driver

//create motors
AccelStepper M1(AccelStepper::DRIVER, M1_STEP_PIN, M1_DIR_PIN); 
AccelStepper M2(AccelStepper::DRIVER, M2_STEP_PIN, M2_DIR_PIN);
AccelStepper M3(AccelStepper::DRIVER, M3_STEP_PIN, M3_DIR_PIN);

MultiStepper actuators;

/*------------------------------------------------------------------------------------------------------------------------------------------------------*/

long positions[3]; // Array of desired stepper positions
boolean homed[] = {false, false, false};

// Commands
String command = "";
String commands[20]; 

extern Coordinate_f end_effector; //Stores the end effector coordinates (declared in Kinematics.cpp)

void setup() {
  driver.beginSerial(9600);   // Start driver software serial  
  Serial.begin(115200);       // Start hardware serial
  
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }

  Serial.println("I0-Setting up stepper driver");

  // Stepper Driver Setup
  driver.begin();               // Initiate pins and registeries
  driver.pdn_disable(1);        // Use PDN/UART pin for communication
  driver.toff(1);               // Enables driver in software
  driver.mstep_reg_select(1);   // microstep read from register
  driver.microsteps(16);
  driver.mres(4);
  driver.I_scale_analog(false); // Use internal voltage reference
  driver.rms_current(700);      // Set driver current 700mA (900mA caused drivers to die)
  driver.en_spreadCycle(false);  // Disable Spread cycle
  
  // Stepper Setup
  M1.setEnablePin(EN_PIN);
  M1.setPinsInverted(false, false, true);
  M1.enableOutputs();
  M1.setMaxSpeed(MAXSPEED);
  M1.setSpeed(MAXSPEED);

  M2.setEnablePin(EN_PIN);
  M2.setPinsInverted(false, false, true);
  M2.enableOutputs();
  M2.setMaxSpeed(MAXSPEED);
  M2.setSpeed(MAXSPEED);

  M3.setEnablePin(EN_PIN);
  M3.setPinsInverted(false, false, true);
  M3.enableOutputs();
  M3.setMaxSpeed(MAXSPEED);
  M3.setSpeed(MAXSPEED);

  actuators.addStepper(M1);
  actuators.addStepper(M2);
  actuators.addStepper(M3);

  positions[0] = 0;
  positions[1] = 0;
  positions[2] = 0;

  //home motors
  Serial.println("I1-Homing motors, please wait...");
  
  while (!homed[0]||!homed[1]||!homed[2]) {
    if(digitalRead(MS1) != HIGH && !homed[0]){
      positions[0] = positions[0] - 1;
    }else{
      homed[0] = true;
    }
    if(digitalRead(MS2) != LOW && !homed[1]){
      positions[1] = positions[1] - 1;
    }else{
      homed[1] = true;
    }
    if(digitalRead(MS3) != LOW && !homed[2]){
      positions[2] = positions[2] - 1;
    }else{
      homed[2] = true;
    }
    actuators.moveTo(positions);
    actuators.runSpeedToPosition();
  }
  Serial.println("I2-Homing Complete");
}

void loop() {
  if (Serial.available() > 0) {
    // read the incoming string
    command = Serial.readString();
    splitCommand();
    
    //Linear Move
    if(commands[0] == "LM"){
      float xt = commands[1].toFloat();
      float yt = commands[2].toFloat();
      float zt = commands[3].toFloat();

      //check bounds
      if(inBounds(xt, yt, zt)){
        linear_move(xt, yt, zt, 1.0, positions, &actuators);
        Serial.println("A2-Linear move complete");
      }else{
        Serial.println("E0-Commanded position is out of bounds");
      }
    }
  }

  if(driver.otpw()){ // checks to see if over tempriture flag is true
    Serial.println("E10-Overheating, turning off motors...");
    M1.disableOutputs();
    M2.disableOutputs();
    M3.disableOutputs();
  }
}

void splitCommand(){
  int StringCount = 0;
  while (command.length() > 0)
  {
    int index = command.indexOf(' ');
    if (index == -1) // No space found
    {
      commands[StringCount++] = command;
      break;
    }
    else
    {
      commands[StringCount++] = command.substring(0, index);
      command = command.substring(index+1);
    }
  }
}

bool inBounds(float x, float y, float z){
  if(x>=X_BOUND_MAX || x<=X_BOUND_MIN){
    return false;
  }
  if(y>=Y_BOUND_MAX || y<=Y_BOUND_MIN){
    return false;
  }
  if(z>=Z_BOUND_MAX || z<=Z_BOUND_MIN){
    return false;
  }
  return true;
}
