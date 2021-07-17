#include "EyeSubsystem.h"

/*
   variable that indicates which motor is being calibrated.
*/
EyeMotor calibrationMotor = rightZ;

/*
   Eye class constructor.
*/
Eyes::Eyes()
{
}

/*
   Initialization function for eye subsystem. Attaches the servo objects
   to their respective pins and sets a default value for the center locations.
*/
void Eyes::init()
{
  this->xServoL.attach(xLPin);
  this->zServoL.attach(zLPin);
  this->xServoR.attach(xRPin);
  this->zServoR.attach(zRPin);

  // 1500 microseconds is about the center for servo objects
  this->lXCenter = 1500;
  this->rXCenter = 1500;
  this->lZCenter = 1500;
  this->rZCenter = 1500;

  this->xServoL.writeMicroseconds(this->lXCenter);
  this->zServoL.writeMicroseconds(this->lZCenter);
  this->xServoR.writeMicroseconds(this->rXCenter);
  this->zServoR.writeMicroseconds(this->rZCenter);

  Serial.println("Finished setting up eyes");
}

/*
   Private helper function parallax to make eye servos look at
   desired coordinate position.
*/
void Eyes::parallax(BLA::Matrix<4> leftDotPos, BLA::Matrix<4> rightDotPos)
{
  // calculating angles of rotation for the left eye
  float alphaLeft = atan2(leftDotPos(1), leftDotPos(0)) * (180 / 3.14159);
  float betaLeft = -atan2(leftDotPos(2), sqrt((leftDotPos(0) * leftDotPos(0)) + (leftDotPos(1) * leftDotPos(1)))) * (180 / 3.14159);

  // calculating angles of rotation for the right eye
  float alphaRight  = atan2(rightDotPos(1), rightDotPos(0)) * (180 / 3.14159);
  float betaRight = atan2(rightDotPos(2), sqrt((rightDotPos(0) * rightDotPos(0)) + (rightDotPos(1) * rightDotPos(1))));

  // writing the angle values to the X and Z servos.
  this->xServoL.writeMicroseconds(this->lXCenter + (90 - alphaLeft));
  //delay(50);
  this->zServoL.writeMicroseconds(this->lZCenter + (betaLeft));
  //delay(50);
  this->xServoR.writeMicroseconds(this->rXCenter + (90 - alphaRight));
  //delay(50);
  this->zServoR.writeMicroseconds(this->rZCenter + (betaRight));
  delay(50);
  //Serial.println("moving");
}

/*
   Function to calibrate the eye servos. The function takes eyeCalCommand
   as a parameter, which is the signal transmitted by the Windows API.
   This character is used as input to the calibration sequence.
*/
void Eyes::CalibrateEyes(char eyeCalCommand)
{
  // Dot position array corresponding to desired screen location
  // set to zero for calibration to calibrate to center of screen.
  BLA::Matrix<4> screenDotPos = {0, 0, 0, 1};

  Serial.println(eyeCalCommand);
  // Eye motor determination
  if (eyeCalCommand == 'r') {
    if (calibrationMotor == leftX) calibrationMotor = rightX;
    if (calibrationMotor == rightZ) calibrationMotor = leftX;
    if (calibrationMotor == leftZ) calibrationMotor = rightZ;
  }
  if (eyeCalCommand == 'l') {
    if (calibrationMotor == rightZ) calibrationMotor = leftZ;
    if (calibrationMotor == leftX) calibrationMotor = rightZ;
    if (calibrationMotor == rightX) calibrationMotor = leftX;
  }

  // switch case to calibrate the center location of respective eye servo.
  // the center locations are incremented/decremented by 5 microseconds for each press.
  switch (calibrationMotor)
  {
    case (rightZ) : {
        if (eyeCalCommand == 'u') {
          this->rZCenter += 5;
        }
        if (eyeCalCommand == 'd') {
          this->rZCenter -= 5;
        }
        break;
      }
    case (leftZ) : {
        if (eyeCalCommand == 'u') {
          this->lZCenter += 5;
        }
        if (eyeCalCommand == 'd') {
          this->lZCenter -= 5;
        }
        break;
      }
    case (rightX) : {
        if (eyeCalCommand == 'u') {
          this->rXCenter += 5;
        }
        if (eyeCalCommand == 'd') {
          this->rXCenter -= 5;
        }
        break;
      }
    case (leftX) : {
        if (eyeCalCommand == 'u') {
          this->lXCenter += 5;
        }
        if (eyeCalCommand == 'd') {
          this->lXCenter -= 5;
        }
        break;
      }
    default : break;
  }

  this->ParallaxEyesToPos(screenDotPos);
}

/*
   Function to make the eyes parallax to a single point
   on the screen. Parameter screenDotPos gives the coordinates of
   the desired position.
*/
void Eyes::ParallaxEyesToPos(BLA::Matrix<4> screenDotPos)
{
  // obtaining instance of kinematic chain class
  // this instance will then help determine the coordinate values
  // for left and right eyes.
  KinematicChain* tfMatrix = KinematicChain::getInstance();

  BLA::Matrix<4> leftDotPos = tfMatrix->GetSL() * screenDotPos;
  BLA::Matrix<4> rightDotPos = tfMatrix->GetSR() * screenDotPos;

  // calling private helper function to write to servos.
  this->parallax(leftDotPos, rightDotPos);
}
