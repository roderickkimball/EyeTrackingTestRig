/*
   This source file contains declarations for all extern variables
   declared in the header file.
*/

#include "Function.h"

Adafruit_BNO055 bno = Adafruit_BNO055(55);

HardwareSerial *SerialTerminal = &Serial1;

// Initialize XBOX and USB Objects
USB Usb;
XBOXONE Xbox(&Usb);

// String array containing state names corresponding
// to enumeration.
String stateNames[12] = {"MenuMode", "ServoCalibration", "Auto", "StepperHome", "StepperManual", "FindCoordinates",
                         "SetCoordinates", "StepperManual", "NeckCalibration", "MoveToCalibrationState", "Neck", "SpinnyBoi"
                        };

// Declare Servo Pins - check wiring diagram if you have any questions
int xLPin = 11;
int zLPin = 5;
int xRPin = 6;
int zRPin = 3;
// Declaring neck servo pins
int NSPin = 12;
//Declaring Servo relay pin
int servoRelay = 8;

// Declaring Shoulder Limit Switches and Stepper Pins - check wiring diagram if you have any questions
// Limit switch pins
int xEnd = 2; // X axis limit switch pin
int zEnd = 4; // Z axis limit switch pin
int yEnd = 7; // Y axis limit switch pin
// SHOULDER Stepper Direction and Pulse Pins
int xDir = A1;
int xPulse = A0;
int yDir = A3;
int yPulse = A2;
int zDir = A5;
int zPulse = A4;
