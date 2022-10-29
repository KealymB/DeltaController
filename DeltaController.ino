#include <AccelStepper.h>
#include <MultiStepper.h>
#include <TMC2208Stepper.h> 
#include "Kinematics.h"

/*------------------------------------------------------------------------------------------------------------------------------------------------------*/
// Motor Params
#define RMS_CURRENT   900   //mA
#define EN_PIN        13

// Micro Switchs
#define MS1           4
#define MS2           5
#define MS3           6

// M1 Pins
#define M1_DIR_PIN    7
#define M1_STEP_PIN   8
// M2 Pins
#define M2_DIR_PIN    9
#define M2_STEP_PIN   10
// M3 Pins
#define M3_DIR_PIN    11
#define M3_STEP_PIN   12

// Driver Params
#define SW_RX         3 
#define SW_TX         2
#define R_SENSE       0.11f

// Motor Params
#define MAXSPEED      150
#define ACCEL         50

// Kinematic Params   (in mm)
#define X_BOUND_MIN   -65
#define X_BOUND_MAX   65
#define Y_BOUND_MIN   -65
#define Y_BOUND_MAX   65
#define Z_BOUND_MIN   -10
#define Z_BOUND_MAX   180
/*------------------------------------------------------------------------------------------------------------------------------------------------------*/

TMC2208Stepper driver = TMC2208Stepper(SW_RX, SW_TX); // create TMC2208 driver

//create motors
AccelStepper M1(AccelStepper::DRIVER, M1_STEP_PIN, M1_DIR_PIN); 
AccelStepper M2(AccelStepper::DRIVER, M2_STEP_PIN, M2_DIR_PIN);
AccelStepper M3(AccelStepper::DRIVER, M3_STEP_PIN, M3_DIR_PIN);

MultiStepper actuators;

/*------------------------------------------------------------------------------------------------------------------------------------------------------*/
//Variables
enum error {NoError, OutOfBounds, NoDriverComs, OverHeat};  // low prio errors first (high prio starts at index 3)
error ErrorBuffer;                            //Error Buffer

long positions[3];                            // Array of desired stepper positions
boolean homed[] = {false, false, false};      // Array of homed stepper motors

// Commands
String command = "";
String commands[40];

extern Coordinate_f end_effector;             //Stores the current end effector coordinates (declared in Kinematics.cpp)

/*------------------------------------------------------------------------------------------------------------------------------------------------------*/

void setup() {
  ErrorBuffer = NoError;        // Clear Error Buffer
  driver.beginSerial(115200);   // Start driver software serial  
  driver.push();                // Reset driver registers
  Serial.begin(115200);         // Start hardware serial
  
  while (!Serial) {
    ; // wait for serial port to connect.
  }

  Serial.println("I0-Setting stepper drivers");

  // Stepper Driver Setup
  driver.pdn_disable(true);         // Use PDN/UART pin for communication
  driver.I_scale_analog(false);     // Use internal voltage reference
  driver.rms_current(RMS_CURRENT);  // Set driver current
  driver.toff(2);                   // Enable driver in software
  driver.mstep_reg_select(1);       // Set MS to be read from register
  driver.microsteps(32);            // Set microsteps to 32

  uint32_t data = 0;
  driver.DRV_STATUS(&data);         // Test driver comms

  if(data == 0) 
  {
    Serial.println("E2-No Comms to Stepper drivers");
    ErrorBuffer = NoDriverComs;
  }else {
    Serial.println("I1-Stepper driver setup successful");  
  }
  
  // Stepper Setup
  M1.setEnablePin(EN_PIN);
  M1.setPinsInverted(false, false, true);
  M1.enableOutputs();
  M1.setAcceleration(ACCEL);
  M1.setMaxSpeed(MAXSPEED);
  M1.setSpeed(MAXSPEED);

  M2.setEnablePin(EN_PIN);
  M2.setPinsInverted(false, false, true);
  M2.enableOutputs();
  M2.setAcceleration(ACCEL);
  M2.setMaxSpeed(MAXSPEED);
  M2.setSpeed(MAXSPEED);

  M3.setEnablePin(EN_PIN);
  M3.setPinsInverted(false, false, true);
  M3.enableOutputs();
  M3.setAcceleration(ACCEL);
  M3.setMaxSpeed(MAXSPEED);
  M3.setSpeed(MAXSPEED);

  actuators.addStepper(M1);
  actuators.addStepper(M2);
  actuators.addStepper(M3);

  // Clear Position Buffer
  positions[0] = 0;
  positions[1] = 0;
  positions[2] = 0;

  // Read mesh level values
  readMeshHeights();
}

/*------------------------------------------------------------------------------------------------------------------------------------------------------*/

void loop() {
  if(ErrorBuffer > NoDriverComs)return; // serious error, will stop working
  
  if (Serial.available() > 0) {
    // read the incoming string into command buffer
    command = Serial.readString();
    CommandHandler();
  }

//  if(driver.checkOT()){ // checks to see if over temperature flag is true (100 degrees)
//    Serial.println("E5-Overheating, turning off motors...");
//    driver.clear_otpw(); // Clear Flag
//    M1.disableOutputs();
//    M2.disableOutputs();
//    M3.disableOutputs();
//    ErrorBuffer = OverHeat;
//  }
}

/*------------------------------------------------------------------------------------------------------------------------------------------------------*/

void homeSteppers(){
  // Moves motors till they have all pressed their limit switches
  Serial.println("I2-Homing motors, please wait...");
  
  // clear homing flags
  homed[0] = false; 
  homed[1] = false;
  homed[2] = false;
  
  while (!homed[0]||!homed[1]||!homed[2]) { // wait till all homed buffers are true
    if(!digitalRead(MS1) && !homed[0]){ // rotate stepper counter clockwise until switch is pressed.
      positions[0] = positions[0] - 1;
    }else{
      homed[0] = true;
    }
    if(!digitalRead(MS2) && !homed[1]){
      positions[1] = positions[1] - 1;
    }else{
      homed[1] = true;
    }
    if(!digitalRead(MS3) && !homed[2]){
      positions[2] = positions[2] - 1;
    }else{
      homed[2] = true;
    }
    actuators.moveTo(positions);
    actuators.runSpeedToPosition();
  }

  float startingCoords[] = {0.0, 0.0, 153.0};
  updateEndEffector(startingCoords[0], startingCoords[1], startingCoords[2]);
  calculateActuatorAngles(startingCoords[0], startingCoords[1], startingCoords[2]);
  
  Serial.println("I3-Homing Complete");
}

/*------------------------------------------------------------------------------------------------------------------------------------------------------*/

void CommandHandler(){
  /** Used to handle serial commands
  * Will call the relevant move
  * Will return a serial error(E1) if the command is not correctly formatted
  * All serial commands must end in an '!'
  */
  if(command.indexOf("!") == -1){ // if there is no character end bit, need to return (E1)
    Serial.println("E1-Line end not found, repeat command");
    return;
  }

  splitCommand(); // split command string into commands array

  if(commands[0] == "LM"){ // Linear Move
    float xt = commands[1].toFloat();
    float yt = commands[2].toFloat();
    float zt = commands[3].toFloat();

    //check bounds
    if(inBounds(xt, yt, zt)){
      linear_move(xt, yt, zt, 0.5, positions, &actuators);
      Serial.println("A2-Linear move complete");
    }else{
      Serial.println("E0-Commanded position is out of bounds");
    }
  }

  if(commands[0] == "JZ"){ // Jog Z-Axis
    float zHeight = commands[1].toFloat();
    zJog(zHeight, positions, &actuators);
    Serial.println("A3-Z jog complete");
  }
  
  if(commands[0] == "CB"){ // Cubic Bezier
    float START[] = {commands[1].toFloat(), commands[2].toFloat()};
    float C1[] = {commands[3].toFloat(), commands[4].toFloat()};
    float C2[] = {commands[5].toFloat(), commands[6].toFloat()};
    float END[] = {commands[7].toFloat(), commands[8].toFloat()};
    
    float zHeight = commands[9].toFloat();
    
    cubic_bezier(START, C1, C2, END, zHeight, positions, &actuators);
    Serial.println("A5-Cubic Bezier move complete");
  }

  if(commands[0] == "SV"){ // set velocity (stepers per second)
      float maxSpeed = commands[1].toFloat();
      // TODO:  check for speed bounds...
      M1.setMaxSpeed(maxSpeed);
      M2.setMaxSpeed(maxSpeed);
      M3.setMaxSpeed(maxSpeed);
      Serial.println("A6-Velocity set successfully");
  }

  if(commands[0] == "SB"){ // set bed height (Aheight, Bheight, Cheight, Dheight)
    setMeshHeights(commands[1].toFloat(), commands[2].toFloat(), commands[3].toFloat(), commands[4].toFloat());
    Serial.println("A7-Height Set");
  }
  
  if(commands[0] == "HS"){ // Home Steppers
    homeSteppers();  
  }
  
  //clear the command buffer
  command = "";
}

/*------------------------------------------------------------------------------------------------------------------------------------------------------*/

void splitCommand(){
  /*
   * Splits a string by spaces
   * sets commands string array
   * to each of the splitted commands
   */
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

/*------------------------------------------------------------------------------------------------------------------------------------------------------*/

bool inBounds(float x, float y, float z){
  /*
  * Checks to see if given coords are within the drawing bounds
  * Returns true if in bounds
  * Returns false if out of bounds
  */
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
