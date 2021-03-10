#include <math.h>

/* 
 * Initialize all variables that are used throughout the code. 
 * Initializing here makes them "globally" known to all .h files
 * that are included below 
 * 
 */
float lookupRes=501;
float Asin_Table[501]={0};
 

// Initial values for screen dimensions
float screenWidth=292.1;
float screenHeight=200.025;
float eyeZlocation=97.282;

float z=0;
float x=0;

// Initial home angles for servos
float leftXhome=89;
float leftZhome=90;
float rightXhome=91;
float rightZhome=90;

float IPD=61.17; // This must change wth the Eye width
float deltaY=650;
float LxEye=0;
float LzEye=0;
float deltaX=0;
float deltaZ=0;
float leftLength=0;
float alphaLeft=0;
float betaLeft=0;
float RxEye=0;
float RzEye=0;
float rightLength=0;
float alphaRight=0;
float betaRight=0; 
float xStep=0;
float zStep=0;

// Initialize global screen coordinates. 
long bottomScreen=0;
long topScreen=0;
long leftScreen=0;
long rightScreen=0;
  
int i=0;
int xsteps=0; // Index of steps along the X Axis
int ysteps=1; // Index of steps along the Y Axis
int zsteps=2; // Index of steps along the Z Axis
int LSyAng=3; // Index of Left Yaw Angle 
int LSrAng=4; // Index of Left Roll Angle
int RSyAng=5; // Index of Right Yaw Angle
int RSrAng=6; // Index of Right Roll Angle
unsigned int Coordinates[7]={0,0,0,0,0,0,0}; // Initialization of Coodinate Array with all values set to 0
int xpos=0; // Index of dot position along the x axis
int zpos=1; // Index of dot position along the y axis
float DotPos[2]={0,0}; // Initialization of Dot Position Array with all values set to 0

/* These are the XY coordinates of the calibration points used by the Tobii Eye Tracker calibration
   routine. The calibrationCoords array will replace DotPos during the eye tracker calibration routine*/
float calibrationCoords[2][7]={{209.78,209.12,105.6,313.97,213.87,107.48,321.89},{232.88,158.33,324.18,325.9,320.35,158.55,158.08}};

int threshold=3750; // Threshold for Xboc controller joystick inputs

//Pins
const byte Analog_X_pin = A0; //x-axis readings
const byte Analog_Y_pin = A1; //y-axis readings
const byte Analog_R_pin = A2; //r-axis readings
const byte LED_pin = 3; //PWM output for LED

//Variables
int Analog_X = 0; //x-axis value
int Analog_Y = 0; //y-axis value
int Analog_R = 0; //r-axis value
int Analog_Z=0;

int Analog_X_AVG = 0; //x-axis value average
int Analog_Y_AVG = 0; //y-axis value average
int Analog_R_AVG = 0; //r-axis value average

int Lim=75;

int del=0;
int joystick=0;
int joystick2=0;
float xStick=0;
float zStick=0;

//int steps=0;
float LxScreen=0;
float LzScreen=0;
float dir=0;
float dir2=0;
boolean limit=false;

int xEnd=23; // X axis limit switch pin
int zEnd=25; // Y axis limit switch pin
int yEnd=27; // Z axis limit switch pin

// Create State Machine
enum StateMachineState {
MENU   = 0,          // Press Start on X-BOX Controller
CAL    = 1,          // Press B on XBOX Controller
MANUAL = 2,          // Press X on XBOX Controller
AUTO   = 3,          // Press A on XBOX Controller
HOM    = 4,          // Press Y on XBOX Controller
STEPPERS = 5,        // Press RB on XBOX Controller
COORDINATES = 6,     // Press RT on XBOX Controller
SET_COORDINATES = 7, // Press LT on XBOX Controller 
};
StateMachineState state=MENU;


// Include the libraries for the XBOX Controller
#include <XBOXONE.h>
#ifdef dobogusinclude
#include <spi4teensy3.h>
#endif
#include <SPI.h>
// Initialize XBOX and USB Objects
USB Usb;
XBOXONE Xbox(&Usb);
int lim=7500;

// Include Accel Stepper Library and initialize stepper objects
#include <AccelStepper.h>
// Stepper Direction and Pulse Pins
int xdir=19;
int xpulse=18;
int ydir=11;
int ypulse=8;
int zdir=2;
int zpulse=3;

/* Conversion factor for converting stepper motor steps to milimeters. This value was experimentally determined
 *  for each axis by making the axis move by a set amount of steps and then measuring the actual distance traveled.
 *  Taking the ratio between steps traveled and mm traveled leaves us with StepsPerMM.
*/
float StepsPerMM=80.22;

/* Initialize AccelStepper objects for each stepper motor.
 * AccelStepper is an amazing library that offers a ton of great features for stepper control.
 * 
 * Extra documentation and functions can be found here. https://www.airspayce.com/mikem/arduino/AccelStepper/classAccelStepper.html
 */
AccelStepper stepperX(AccelStepper::DRIVER, xdir, xpulse); // X Axis Stepper
AccelStepper stepperY(AccelStepper::DRIVER, ydir, ypulse); // Y Axis Stepper
AccelStepper stepperZ(AccelStepper::DRIVER, zdir, zpulse); // Z Axis Stepper
 
// Create Servo Objects
#include <Servo.h>
Servo XservoL; // Left X Servo (Looks left and right)
Servo ZservoL; // Left Z Servo (Looks up and down)
Servo XservoR; // Right X Servo (Looks left and right)
Servo ZservoR; // Right Z Servo (Looks up and down)


// Inlcude all functions files
#include "distancePerStep.h" // Funtions used to find the distance per step
#include "auto_functions.h"  // Nothing added yet. Was originally meant to contain all auto routines
#include "manual_functions.h" // Not currently in use. Contains the getMove() function which has been replaced by GetPos().  
#include "homing_functions.h" // Contains zeroStepper() which is used to home the stepper axes.
#include "Steppers.h" // Contains all functions needed to manually control the stepepr axes.
#include "Servos.h" // Contains all functions needed to manually control the servos.
#include "calibration.h" // Contains all functions needed to calibrate the steppers and the servos to the rig.
#include "James_functions.h" // Not currently in use. The idea was to use these files as "storage" for conceptual functions
#include "Leif_functions.h"  // Not currently in use. The idea was to use these files as "storage" for conceptual functions


// The initial state after system startup
void runMenuState()
{
  //Conditional statements to change the state if desired.
  if(Xbox.getButtonClick(Y))
  {
    state=HOM; 
  }
  if(Xbox.getButtonClick(X))
  {
    state=MANUAL;
  }
  if(Xbox.getButtonClick(A))
  {
    state=AUTO;
  }
  if(Xbox.getButtonClick(B))
  {
    state=CAL;
  }
    if(Xbox.getButtonClick(R2))
  {
    state=COORDINATES;
  }
  if(Xbox.getButtonClick(L2))
  {
    state=SET_COORDINATES;
  }
  if(Xbox.getButtonClick(R1))
  {
    state=STEPPERS;
  }
}


/*  This function will run through the entire calibration routine for the rig. 
 *   
 *  Make sure to mount both eye lasers, and the chin laser on the rig. 
 *   
 *  First the stepper motors are calibrated via the stepperCalibration() function. 
 *  This is done by homing each axis towards its respective limit switch. Then manual 
 *  control is given to the user so he/she can pilot the chin laser to the bottom left 
 *  and top right corners of the screen. Once this is complete, the rig moves the chin 
 *  laser to the center of the screen and returns from stepperCalibration(). 
 *  
 *  servoCalibration() then gives manual control to first the left and then the
 *  right eye so they can be steered to converege with the chin laser. Once this is complete, 
 *  then the rig is fully calibrated and can be calibrated to the eye tracker. 
*/
void runCalState()
{
stepperCalibration();
servoCalibration();

state=MENU;
}

// This function will allow the eyes to be controlled manually
void runManualState()
{
  GetPos(); // Moves a virtual dot around the screen via the left joystick.
  parallax(); // Contains the relationshps to make the eyes converge on a single point.

  //Conditional statements to change the state if desired.
   if(Xbox.getButtonClick(Y))
  {
    state=HOM;
  }
  if(Xbox.getButtonClick(B))
  {
    state=CAL;
  }
  if(Xbox.getButtonClick(A))
  {
    state=AUTO;
  }
  if(Xbox.getButtonClick(START))
  {
    state=MENU;
  }
  if(Xbox.getButtonClick(R1))
  {
    state=STEPPERS;
  }
    if(Xbox.getButtonClick(R2))
  {
    state=COORDINATES;
  }
  if(Xbox.getButtonClick(L2))
  {
    state=SET_COORDINATES;
  }
}

// No current use. May be used in the future to run more complex automated routines
void runAutoState()
{
  //Conditional statements to change the state if desired.
   if(Xbox.getButtonClick(Y))
  {
    state=HOM;
  }
  if(Xbox.getButtonClick(B))
  {
    state=CAL;
  }
  if(Xbox.getButtonClick(X))
  {
    state=MANUAL;
  }
  if(Xbox.getButtonClick(START))
  {
    state=MENU;
  }
    if(Xbox.getButtonClick(R2))
  {
    state=COORDINATES;
  }
  if(Xbox.getButtonClick(L2))
  {
    state=SET_COORDINATES;
  }
}

/* This function will home the steppers and reset the home location.
 *  
 *  NOTE: DO NOT change the order of X and Z homing routines. If they are swapped, 
 *  there is a chance that the rig may crash.
*/
void runHomeState()
{
  zeroStepper(xpulse,xdir,xEnd); // DO NOT switch with Z
  stepperX.setCurrentPosition(-500);
  zeroStepper(ypulse,ydir,yEnd);
  stepperY.setCurrentPosition(500);
  zeroStepper(zpulse,zdir,zEnd); // DO NOT switch with X
  stepperZ.setCurrentPosition(500);
  state=MENU;
}

// Allows for manual control of the stepper axes
void runSteppersState()
{
  i=0;

  while(i==0) // Keep the system in the stepper state
{
  
  Usb.Task(); // Request data from Xbox controller
  ReadAnalog(); // Take Xbox controller data and convert to desired stepper axis movement
  stepperX.run(); 
  stepperY.run();
  stepperZ.run();
  
  //Conditional statements to change the state if desired.
  if(Xbox.getButtonClick(Y))
  {
    state=HOM;
    i=1;
  }
  if(Xbox.getButtonClick(B))
  {
    state=CAL;
    i=1;
  }
  if(Xbox.getButtonClick(X))
  {
    state=MANUAL;
    i=1;
  }
  if(Xbox.getButtonClick(START))
  {
    state=MENU;
    i=1;
  } 
    if(Xbox.getButtonClick(R2))
  {
    state=COORDINATES;
  }
  if(Xbox.getButtonClick(L2))
  {
    state=SET_COORDINATES;
  }
}  
}

void LookupTables()
{
  // Arcsin
  for(i=0;i<lookupRes;i++)
  {
    Asin_Table[i]=asin((i*(2/(lookupRes-1))-.5));
  }

}



