#include <math.h>

/* 
 * Initialize all variables that are used throughout the code. 
 * Initializing here makes them "globally" known to all .h files
 * that are included below 
 * 
 */

float screenWidth=292.1;
float screenHeight=200.025;
float eyeZlocation=65;
  float z=0;
  float x=0;
  float leftXhome=0;
  float leftZhome=0;
  float rightXhome=0;
  float rightZhome=0;

  float IPD=65;
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
  
int i=0;

int xsteps=0; // Index of steps along the X Axis
int ysteps=1; // Index of steps along the Y Axis
int zsteps=2; // Index of steps along the Z Axis
int LSyAng=3; // Index of Left Yaw Angle 
int LSrAng=4; // Index of Left Roll Angle
int RSyAng=5; // Index of Right Yaw Angle
int RSrAng=6; // Index of Right Roll Angle
unsigned int Coordinates[7]={0,0,0,0,0,0,0}; // Initialization of Coodinate Array with all values set to 0
int xpos=0;
int ypos=1;
//float DotPos[2]={146.05,100.0125};
float DotPos[2]={0,0};
int threshold=3750;

int del=0;
int joystick=0;
int joystick2=0;
int xStick=0;
int yStick=0;

//int steps=0;
float LxScreen=0;
float LyScreen=0;
float dir=0;
float dir2=0;
bool limit=LOW;

// Create State Machine
enum StateMachineState {
MENU   = 0, // Press Start on X-BOX Controller
CAL    = 1, // Press B on XBOX Controller
MANUAL = 2, // Press X on XBOX Controller
AUTO   = 3, // Press A on XBOX Controller
HOM    = 4, // Press Y on XBOX Controller
};
StateMachineState state=MENU;
// Include math library
//#include <math.h>
// Include the libraries for the XBOX Controller
#include <XBOXONE.h>
#ifdef dobogusinclude
#include <spi4teensy3.h>
#endif
#include <SPI.h>
// Initialize XBOX and USB Objects
USB Usb;
XBOXONE Xbox(&Usb);
// Include Accel Stepper Library and initialize stepper objects
#include <AccelStepper.h>
AccelStepper stepperX(AccelStepper::DRIVER, 3, 2);
AccelStepper stepperY(AccelStepper::DRIVER, 5, 4);
AccelStepper stepperZ(AccelStepper::DRIVER, 9, 8);

// Create Servo Objects
#include <Servo.h>
Servo XservoL;
Servo YservoL;
Servo XservoR;
Servo YservoR;
// Inlcude all functions files
#include "auto_functions.h"
#include "manual_functions.h"
#include "homing_functions.h"
#include "calibration.h"
#include "James_functions.h"
#include "Leif_functions.h"
#include "Servos.h"



void runMenuState()
{
  //Serial.println("In Menu");
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
}

void runCalState()
{
  //Serial.println("In Calibration");
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
  if(Xbox.getButtonClick(START))
  {
    state=MENU;
  }
}

void runManualState()
{
  //getMove();
  GetPos();
  //MoveDot();
  parallax();
  //Serial.println("In Manual");
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
}

void runAutoState()
{
  //Serial.println("In Auto");
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
}

void runHomeState()
{
  findLimit();
  
  bounceBack();
    
  smoothFind();

  state=MENU;
}

void servoControl()
{
  
}

void getButtons()
{
  Usb.Task();
  
}




