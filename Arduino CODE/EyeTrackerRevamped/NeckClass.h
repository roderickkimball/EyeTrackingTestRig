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
#include "M3Stepper.h"

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
    M3Stepper m3FrontStepper, m3BackRightStepper, m3BackLeftStepper;

    Servo neckServo;
    long int neckServoCenter;

    float lastPhiR, lastPhiS, lastPhiD;

    // length of the spring in the middle of the neck
    float lSpring;
    // function to update the neck transformation matrices
    void updateNeckTransformation();

  protected:
    /*
      C - will be the origin frame of reference located at the center of the Z-axis stage,
      that the neck and eye mechanisms are mounted to.
      Y - will be the frame of reference located at the center of the yaw rotation servo.
      N - will be the frame of reference located at the base of the neck of the robot.
      T - will be the frame of reference located at the center of the top of the neck.
    */
    BLA::Matrix<4, 4> gCY, gYN, gNT, gTC;
    /*
       Now we define the transformation matrices for the neck.
       O - signifies the origin point of the neck at the middle of the base.
       B1 - front point at the base; B2 - back right point at the base; B3 - back left point at the base;
       P1 - front point at the top; P2 - back right point; P3 - back left point; PC - center point at the top
    */
    BLA::Matrix<4, 4> gO_B1, gO_B2, gO_B3, gPC_P1, gPC_P2, gPC_P3;
    BLA::Matrix<4, 4> gB1_P1, gB2_P2, gB3_P3;

  public:
    RobotNeck();
    ~RobotNeck();
    void init();
    void CalibrateNeck(char neckCalCommand);
    void MoveToCalibratedPosition();
    void MoveNeckManually(float x, float y, float z);
    void SetLastNeckPosition();
    float GetLastNeckPosition(String whichAngle);
    void RunSteppers();
    void WriteNeckPositionToProm();
    void ReadNeckPositionFromProm();
    // functions to get the inverse transformation matrices for the neck subsystem    
    BLA::Matrix<4,4> GetInverseNeckTransformation();

};

#endif
