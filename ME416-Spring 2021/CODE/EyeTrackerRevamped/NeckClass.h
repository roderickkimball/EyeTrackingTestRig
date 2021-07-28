/*
   Neck Class header file. This class is responsible for encapsulating all the stepper
   objects that are make up the neck of the robot and also the private-public member functions
   that calibrate and move the neck of the robot.
*/

#ifndef _NECKCLASS_H
#define _NECKCLASS_H

#include <AccelStepper.h>
#include "Function.h"
#include "KinematicChain.h"
#include <Servo.h>

/*
   Additional variables that will be used by the neck
   objects to control and calibrate the neck.
*/
enum NeckMotor {
  front = 1,
  backRight = 2,
  backLeft = 3,
  yawServo = 4,
};
extern NeckMotor neckCalibrationMotor;

// The known cable length after calibration. Equal to calibration stick length.
extern float knownStepperPos;
// Stepper configuration variables
extern float spoolRadius, spoolCircumference, mmPerRevs, stepsPerRev, mmPerStep;
// Neck Servo microsecondsPerDegree
extern float yawMicroSecondsPerDegree;

/*
   Neck class declaration. This object encapsulates the entire
   neck mechanism and functions to control the neck.
*/
class RobotNeck
{
  private:
    // private member variables
    // AccelStepper objects for 3 stepper motors of neck
    AccelStepper *frontStepper, *backRightStepper, *backLeftStepper;

    Servo neckServo;
    long int neckServoCenter;

    // front stepper last recorded position and calibration position
    float lastStepperPosFront, calibrationStepperPosFront;
    // back right stepper last recorded position and calibration position
    float lastStepperPosBackRight, calibrationStepperPosBackRight;
    // back left stepper last recorded position and calibration position
    float lastStepperPosBackLeft, calibrationStepperPosBackLeft;

    // length of the spring in the middle of the neck
    float lSpring;

  public:
    RobotNeck();
    ~RobotNeck();
    void init();
    void CalibrateNeck(char neckCalCommand);
    void MoveToCalibratedPosition();
    void MoveNeckManually(float x, float y, float z);
    void RunSteppers();
    void WriteNeckPositionToProm(char calibrationOrLastPosition);
    void ReadNeckPositionFromProm(char calibrationOrLastPosition);

};

#endif
