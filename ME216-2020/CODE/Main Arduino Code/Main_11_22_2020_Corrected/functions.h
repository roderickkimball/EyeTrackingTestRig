#include <math.h>

/* 
 * Initialize all variables that are used throughout the code. 
 * Initializing here makes them "globally" known to all .h files
 * that are included below 
 * 
 */
float lookupRes=501;
float Asin_Table[501]={0};
 

float screenWidth=292.1;
float screenHeight=200.025;
float eyeZlocation=97.282;

  float z=0;
  float x=0;
  float leftXhome=90;
  float leftZhome=90;
  float rightXhome=90;
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
int xpos=0;
int zpos=1;
//float DotPos[2]={146.05,100.0125};
float DotPos[2]={0,0};
int threshold=3750;

// Stepper Initializations
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

int Analog_R_Value = 0; //this is used for the PWM value
int Lim=50;

int del=0;
int joystick=0;
int joystick2=0;
int xStick=0;
int zStick=0;

//int steps=0;
float LxScreen=0;
float LzScreen=0;
float dir=0;
float dir2=0;
boolean limit=false;

int xEnd=23;
int zEnd=25;
int yEnd=27;
//
//boolean x = false;
//boolean y = false;
//boolean z = false;

// Create State Machine
enum StateMachineState {
MENU   = 0, // Press Start on X-BOX Controller
CAL    = 1, // Press B on XBOX Controller
MANUAL = 2, // Press X on XBOX Controller
AUTO   = 3, // Press A on XBOX Controller
HOM    = 4, // Press Y on XBOX Controller
STEPPERS = 5 // Press RB on XBOX Controller
};
StateMachineState state=MENU;
//StateMachineState state=HOM;
//StateMachineState state=MANUAL;
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
int lim=500;

// Include Accel Stepper Library and initialize stepper objects
#include <AccelStepper.h>
int xdir=19;
int xpulse=18;
int ydir=11;
int ypulse=8;
int zdir=2;
int zpulse=3;

float StepsPerMM=80.22;

AccelStepper stepperX(AccelStepper::DRIVER, xdir, xpulse);
AccelStepper stepperY(AccelStepper::DRIVER, ydir, ypulse);
AccelStepper stepperZ(AccelStepper::DRIVER, zdir, zpulse);
 


// Create Servo Objects
#include <Servo.h>
Servo XservoL;
Servo ZservoL;
Servo XservoR;
Servo ZservoR;
// Inlcude all functions files
#include "distancePerStep.h"
#include "auto_functions.h"
#include "manual_functions.h"
#include "homing_functions.h"
#include "Steppers.h"
#include "Servos.h"
#include "calibration.h"
#include "James_functions.h"
#include "Leif_functions.h"







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
stepperCalibration();
servoCalibration();

state=MENU;
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
  if(Xbox.getButtonClick(R1))
  {
    state=STEPPERS;
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
  //zeroStepper(int dir, int pulse, int lim)//direction pin, pulse pin, limit pin
  zeroStepper(xpulse,xdir,xEnd);
  stepperX.setCurrentPosition(500);
  zeroStepper(ypulse,ydir,yEnd);
  stepperY.setCurrentPosition(500);
  zeroStepper(zpulse,zdir,zEnd);
  stepperZ.setCurrentPosition(500);
  state=MENU;
}

void servoControl()
{
  
}

void getButtons()
{
  Usb.Task();
  
}

void runSteppersState()
{
  //i=0;
  while(i==0)
{
  
  Usb.Task();
  if(Xbox.getButtonClick(B)||Xbox.getButtonClick(Y)||Xbox.getButtonClick(X)||Xbox.getButtonClick(START))
  {
    i=1;
  }
  ReadAnalog(); 
  stepperX.run(); 
  stepperY.run();
  stepperZ.run();
}
Usb.Task();
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

void LookupTables()
{
  // Arcsin
  for(i=0;i<lookupRes;i++)
  {
    Asin_Table[i]=asin((i*(2/(lookupRes-1))-.5));
  }

  // Square Root
}


