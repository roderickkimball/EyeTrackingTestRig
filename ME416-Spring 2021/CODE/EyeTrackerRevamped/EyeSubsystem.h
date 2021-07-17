/*
   Eye subsystem class consists of all of the Servo objects that make up the 2 eyes of the robot.
   The class encapsulates all servos used by both eyes,
   and the center locations in microseconds for those servos. The class has public member functions
   to calibrate the eye subsystem and parallax both eyes to a single point on the screen.
*/

#ifndef _EYESUBSYSTEM_H
#define _EYESUBSYSTEM_H

#include <Servo.h>
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
    Servo xServoL;
    Servo zServoL;
    Servo xServoR;
    Servo zServoR;
    long int lXCenter, lZCenter, rXCenter, rZCenter;
    void parallax(BLA::Matrix<4> leftDotPos, BLA::Matrix<4> rightDotPos);

  public:
    Eyes();
    void init();
    void CalibrateEyes(char eyeCalCommand);
    void ParallaxEyesToPos(BLA::Matrix<4> screenDotPos);
};

#endif
