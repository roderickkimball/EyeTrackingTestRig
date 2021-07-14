/*
 * This source file contains declarations for all extern variables
 * declared in the header file.
 */

#include "Function.h"

Adafruit_BNO055 bno = Adafruit_BNO055(55);

HardwareSerial *SerialTerminal = &Serial1;

// Initialize XBOX and USB Objects
USB Usb;
XBOXONE Xbox(&Usb);

String stateNames[12] = {"MenuMode", "ServoCalibration", "Auto", "StepperHome", "StepperManual", "FindCoordinates",
    "SetCoordinates", "StepperManual", "NeckCalibration", "MoveToCalibrationState","Neck","SpinnyBoi"};

// Declare Servo Pins - check wiring diagram if you have any questions
int xLPin = 11;
int zLPin = 5;
int xRPin = 6;
int zRPin = 3;
// Declaring neck servo pins
int NSPin = 12;
