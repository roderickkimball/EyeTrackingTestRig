#include "EyeSubsystem.h"

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
void Eyes::init(int xLPin, int zLPin, int xRPin, int zRPin)
{
  this->xServoL.attach(xLPin);
  this->zServoL.attach(zLPin);
  this->xServoR.attach(xRPin);
  this->zServoR.attach(zRPin);
  this->lXCenter = 1500;
  this->rXCenter = 1500;
  this->lZCenter = 1500;
  this->rZCenter = 1500;
}

/*
   Function to calibrate the eye servos. The function takes eyeCalCommand
   as a parameter, which is the signal transmitted by the Windows API.
   This character is used as input to the calibration sequence.
*/
bool Eyes::CalibrateServos(char eyeCalCommand)
{
  // calibration status boolean
  bool calibrationStatus = true;

  // if back is pressed then boolean falsified and returned
  if (eyeCalCommand == 'b')
  {
    calibrationStatus = false;
    return calibrationStatus;
  }

  // Enumeration object used to switch between eye motors.
  EyeMotor calibrationMotor = 1;

  // Eye motor determination
  if (eyeCalCommand == 'r') {
    int tempMotor = static_cast<int>(calibrationMotor);
    tempMotor++;
    calibrationMotor = static_cast<EyeMotor>(tempMotor);
  }
  if (eyeCalCommand == 'l') {
    int tempMotor = static_cast<int>(calibrationMotor);
    tempMotor--;
    calibrationMotor = static_cast<EyeMotor>(tempMotor);
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
  return calibrationStatus;
}

/*
 * Function to make the eyes parallax to a single point
 * on the screen. Parameter screenDotPos gives the coordinates of
 * the desired position.
 */
void Eyes::ParallaxServosToPos(BLA::Matrix<4> screenDotPos)
{
  KinematicChain* tfMatrix = KinematicChain::getInstance();
  
  BLA::Matrix<4> leftDotPos = tfMatrix->GetSL() * screenDotPos;
  BLA::Matrix<4> rightDotPos = tfMatrix->GetSR() * screenDotPos;

  this->parallax(leftDotPos, rightDotPos);
}

/*
 * Private helper function parallax to make eye servos look at
 * desired coordinate position.
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
  delay(10);
}
