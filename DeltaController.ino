#include <AccelStepper.h>
#include <MultiStepper.h>
#include <TMC2208Stepper.h>
#include "Kinematics.h"

/*------------------------------------------------------------------------------------------------------------------------------------------------------*/

// Motor pins
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

// Motor Params
#define MAXSPEED 150

/*------------------------------------------------------------------------------------------------------------------------------------------------------*/

TMC2208Stepper driver = TMC2208Stepper(3, 2); // create TMC2208 driver

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
  driver.beginSerial(9600);
  Serial.begin(115200);        // Start hardware serial
  
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }

  Serial.println("Setting up stepper driver");

  // Stepper Driver Setup
  driver.push();                // Reset registers
  driver.mstep_reg_select(1);   // microstep read from register
  driver.microsteps(16);
  driver.mres(4);
  driver.pdn_disable(true);     // Use PDN/UART pin for communication
  driver.I_scale_analog(false); // Use internal voltage reference
  driver.rms_current(700);      // Set driver current 600mA (900mA caused drivers to die)
  driver.toff(2);               // Enable driver in software
  driver.clear_otpw();        // clear over temp flag

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
  Serial.println("Homing motors, please wait...");
  
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
  Serial.println("Homing Complete");
  Serial.println("M1 Pos: ");
  Serial.println(positions[0]);
  Serial.println("M2 Pos: ");
  Serial.println(positions[1]);
  Serial.println("M3 Pos: ");
  Serial.println(positions[2]);
}

void loop() {
  if (Serial.available() > 0) {
    // read the incoming string
    command = Serial.readString();
    splitCommand();
    
    if(commands[0] == "M1"){ //Direct Jog
      positions[0] = (long) command.substring(3).toInt();
    }
    if(commands[0] == "M2"){ //Direct Jog
      positions[1] = (long) command.substring(3).toInt();
    }
    if(commands[0] == "M3"){ //Direct Jog
      positions[2] = (long) command.substring(3).toInt();
    }
    if(commands[0] == "LM"){ //Linear Move
      float xt = commands[1].toFloat();
      float yt = commands[2].toFloat();
      float zt = commands[3].toFloat();

      Serial.println("xt: ");
      Serial.println(xt);
      Serial.println("yt: ");
      Serial.println(yt);      
      Serial.println("zt: ");
      Serial.println(zt);
      
      linear_move(xt, yt, zt, 1.0, positions, &actuators);
    }
  }
  
//  actuators.moveTo(positions);
//  actuators.runSpeedToPosition(); // Blocks until all are in position

//  if(driver.checkOT()){ // checks to see if over tempriture flag is true
//    Serial.println("Overheating, turning off motors...");
//    M1.disableOutputs();
//    M2.disableOutputs();
//    M3.disableOutputs();
//  }
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
