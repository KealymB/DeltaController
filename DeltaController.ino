#include <AccelStepper.h>
#include <MultiStepper.h>
#include <TMCStepper.h>
#include "Kinematics.h"
/*------------------------------------------------------------------------------------------------------------------------------------------------------*/
// Motor Params
#define RMS_CURRENT   800   //mA
#define EN_PIN        12

// M1 Pins
#define M1_STEP_PIN   11
#define M1_DIR_PIN    10
// M2 Pins
#define M2_STEP_PIN   8
#define M2_DIR_PIN    7
// M3 Pins
#define M3_STEP_PIN   5
#define M3_DIR_PIN    4

// MicroSwitch
#define MS1           13
#define MS2           9
#define MS3           6

// Driver Params
#define SW_RX         3 
#define SW_TX         2
#define R_SENSE       0.11f

// Motor Params
#define MAXSPEED      50
#define ACCEL         1

// Kinematic Params   (in mm)
#define X_BOUND_MIN   -150
#define X_BOUND_MAX   150
#define Y_BOUND_MIN   -150
#define Y_BOUND_MAX   150
#define Z_BOUND_MIN   50
#define Z_BOUND_MAX   290

/*------------------------------------------------------------------------------------------------------------------------------------------------------*/

TMC2208Stepper driver = TMC2208Stepper(SW_RX, SW_TX, R_SENSE); // create TMC2208 driver

//create motors
AccelStepper M1(AccelStepper::DRIVER, M1_STEP_PIN, M1_DIR_PIN); 
AccelStepper M2(AccelStepper::DRIVER, M2_STEP_PIN, M2_DIR_PIN);
AccelStepper M3(AccelStepper::DRIVER, M3_STEP_PIN, M3_DIR_PIN);

MultiStepper actuators;

/*------------------------------------------------------------------------------------------------------------------------------------------------------*/
//Variables
enum error {NoError, OutOfBounds, OverHeat};  // low value errors first (high starts at index 1)
error ErrorBuffer;                            //Error Buffer

long positions[3];                            // Array of desired stepper positions
boolean homed[] = {false, false, false};      // Array of homed stepper motors




// Commands
String command = "";
String commands[20];

extern Coordinate_f end_effector;             //Stores the current end effector coordinates (declared in Kinematics.cpp)

/*------------------------------------------------------------------------------------------------------------------------------------------------------*/

void setup() {
  ErrorBuffer = NoError;      //Clear Error Buffer
  driver.beginSerial(9600);   // Start driver software serial  
  Serial.begin(115200);       // Start hardware serial
  
  while (!Serial) {
    ; // wait for serial port to connect.
  }

  Serial.println("I0-Setting up stepper driver");

  // Stepper Driver Setup
  driver.begin();                   // Initiate pins and registeries
  driver.pdn_disable(1);            // Use PDN/UART pin for communication
  driver.toff(1);                   // Enables driver in software
  driver.mstep_reg_select(1);       // microstep read from register
  driver.microsteps(16);
  driver.I_scale_analog(false);     // Use internal voltage reference
  driver.rms_current(RMS_CURRENT);  // Set driver current
  driver.en_spreadCycle(false);     // Disable Spread cycle
  
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

  //home motors
  Serial.println("I1-Homing motors, please wait...");
  
  while (!homed[0]||!homed[1]||!homed[2]) { // wait till all homed buffers are true
    if(digitalRead(MS1) != HIGH && !homed[0]){ // rotate stepper counter clockwise until switch is pressed.
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

  Serial.println("positions: ");
  Serial.print(positions[0]);
  Serial.print(", ");
  Serial.print(positions[1]);
  Serial.print(", ");
  Serial.print(positions[2]);
  Serial.print("\n");
}

void loop() {
  if(ErrorBuffer > 1)return; // serious error, will stop processing
  
  if (Serial.available() > 0) {
    // read the incoming string into command buffer
    command = Serial.readString();
    CommandHandler();
  }

  if(driver.otpw()){ // checks to see if over temperature warning flag is true (100 degrees)
    Serial.println("E10-Overheating, turning off motors...");
    M1.disableOutputs();
    M2.disableOutputs();
    M3.disableOutputs();
    ErrorBuffer = OverHeat;
  }
}

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

  if(commands[0] == "LM"){
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
  //clear the command buffer
  command = "";
}
