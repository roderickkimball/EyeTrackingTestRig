/*
   Eye subsystem class consists of all of the Servo objects that make up the 2 eyes of the robot.
   The class encapsulates all servos used by both eyes,
   and the center locations in microseconds for those servos. The class has public member functions
   to calibrate the eye subsystem and parallax both eyes to a single point on the screen.
*/

#ifndef _EYESUBSYSTEM_H
#define _EYESUBSYSTEM_H

#include <Servo.h>
#include <EEPROM.h>
#include "KinematicChain.h"
#include "Function.h"

/*
   This enumeration is used in servo calibration to determine which servo motor, on which side
   is being calibrated
   X- horizontal
   Z- vertical
*/
enum EyeMotor {
  rightZ = 1,
  leftZ = 2,
  rightX = 3,
  leftX = 4,
};
extern EyeMotor calibrationMotor;

/*
   Eye class declaration consisting of private and public member functions.
   Private - Servo objects, calibration motor and center locations
   Public functions - initialization function, calibration function and parallaxing function for eyes.
*/
class Eyes
{
  private:
    // Servo objects for left and right eyes
    Servo xServoL;
    Servo zServoL;
    Servo xServoR;
    Servo zServoR;
    // center locations in microseconds for the servo objects.
    long int lXCenter, lZCenter, rXCenter, rZCenter;
    // variable denoting the number of steps required to move the eyes 1 degree on the screen.
    // (Empirically determined)
    float lXmicroSecondsPerDegree, lZmicroSecondsPerDegree, rXmicroSecondsPerDegree, rZmicroSecondsPerDegree;
    // this function makes the eyes parallax or look at a single point on the screen
    void parallax(BLA::Matrix<4> leftDotPos, BLA::Matrix<4> rightDotPos);

  public:
    //constructor
    Eyes();
    //initializer
    void init();
    //calibration function
    void CalibrateEyes(char eyeCalCommand, KinematicChain* tfMatrix);
    // functions to write and read variables to/from EEPROM
    void WriteEyeCalibrationVariablesToProm();
    void ReadEyeCalibrationVariablesFromProm();
    // public function called by external objects to parallax eyes
    void ParallaxEyesToPos(BLA::Matrix<4> screenDotPos, KinematicChain* tfMatrix);
};

#endif
