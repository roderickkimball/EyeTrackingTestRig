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
   to their respective pins and sets default value for the center locations.
*/
void Eyes::init()
{
  this->xServoL.attach(xLPin);
  this->zServoL.attach(zLPin);
  this->xServoR.attach(xRPin);
  this->zServoR.attach(zRPin);

  // 1500 microseconds is the default center for servo objects from class Servo.h
  this->lXCenter = 1500;
  this->lZCenter = 1500;
  this->rXCenter = 1500;
  this->rZCenter = 1500;

  // setting up the default values for microSecondsPerDegree (empirically determined)
  this->lXmicroSecondsPerDegree = 10.20408163 * 1.05;
  this->lZmicroSecondsPerDegree = 10.20408163 * 1.05;
  this->rXmicroSecondsPerDegree = 10.20408163;
  this->rZmicroSecondsPerDegree = 10.20408163;

  this->xServoL.writeMicroseconds(this->lXCenter);
  this->zServoL.writeMicroseconds(this->lZCenter);
  this->xServoR.writeMicroseconds(this->rXCenter);
  this->zServoR.writeMicroseconds(this->rZCenter);

  // setting initial values to the eye subsystem transformation matrices
  this->gTD = KinematicChain::xform(0, 0, 0, (-0.004), (0.0636), (0.0856));
  this->gDL = KinematicChain::xform(0, 0, 0, (-0.03475), 0, 0);
  this->gDR = KinematicChain::xform(0, 0, 0, (0.03475), 0, 0);

  this->gLT = gDL.Inverse() * gTD.Inverse();
  this->gRT = gDR.Inverse() * gTD.Inverse();

  //SerialTerminal->println("Finished setting up eyes");
}

/*
   Parallax function to make eye servos look at
   desired coordinate position. Left and right dot positions indicate the desired
   position on the screen in the frame of reference of the left and right eyes, that the eyes need to look at.
*/
void Eyes::parallax(BLA::Matrix<4> leftDotPos, BLA::Matrix<4> rightDotPos)
{
  // calculating angles of rotation for the left eye
  float alphaLeft = atan2(leftDotPos(1), leftDotPos(0)) * (180 / M_PI);
  float betaLeft = atan2(leftDotPos(2), sqrt((leftDotPos(0) * leftDotPos(0)) + (leftDotPos(1) * leftDotPos(1)))) * (180 / M_PI);

  // calculating angles of rotation for the right eye
  float alphaRight  = atan2(rightDotPos(1), rightDotPos(0)) * (180 / M_PI);
  float betaRight = atan2(rightDotPos(2), sqrt((rightDotPos(0) * rightDotPos(0)) + (rightDotPos(1) * rightDotPos(1)))) * (180 / M_PI);

  // writing the angle values to the X and Z servos.
  this->xServoL.writeMicroseconds(this->lXCenter + (90 - alphaLeft) * this->lXmicroSecondsPerDegree);
  //delay(50);
  this->zServoL.writeMicroseconds(this->lZCenter + (betaLeft) * this->lZmicroSecondsPerDegree);
  //delay(50);
  this->xServoR.writeMicroseconds(this->rXCenter + (90 - alphaRight) * this->rXmicroSecondsPerDegree);
  //delay(50);
  this->zServoR.writeMicroseconds(this->rZCenter - (betaRight) * this->rZmicroSecondsPerDegree);
  delay(50);
}

/*
   Gets the inverse transformation that goes from center of left eye to top of neck.
*/
BLA::Matrix<4, 4> Eyes::GetInverseLeftEyeTransformation()
{
  this->gLT = gDL.Inverse() * gTD.Inverse();
  return this->gLT;
}

/*
   Gets the inverse transformation that goes from the center of right eye to top of neck.
*/
BLA::Matrix<4, 4> Eyes::GetInverseRightEyeTransformation()
{
  this->gRT = gDL.Inverse() * gTD.Inverse();
  return this->gRT;
}

/*
   Function to write the calibration variables
   for Eye Servos to EEPROM.
*/
void Eyes::WriteEyeCalibrationVariablesToProm()
{
  PromAddress eeAddress = LXCenter;

  EEPROM.put(eeAddress, this->lXCenter);
  // incrementing the address variable by 4 because
  // it is the size of a floating point data type
  eeAddress = (PromAddress)((int)eeAddress + 4);
  EEPROM.put(eeAddress, this->lZCenter);
  eeAddress = (PromAddress)((int)eeAddress + 4);
  EEPROM.put(eeAddress, this->rXCenter);
  eeAddress = (PromAddress)((int)eeAddress + 4);
  EEPROM.put(eeAddress, this->rZCenter);

  //SerialTerminal->println("Eye Calibration Variables written to Prom");
}

/*
   Function to read the calibration variables
   for Eye Servos from EEPROM.
*/
void Eyes::ReadEyeCalibrationVariablesFromProm()
{
  PromAddress eeAddress = LXCenter;
  long int servoCenter = 0.0;

  EEPROM.get(eeAddress, servoCenter);
  this->lXCenter = servoCenter;
  eeAddress = (PromAddress)((int)eeAddress + 4);

  EEPROM.get(eeAddress, servoCenter);
  this->lZCenter = servoCenter;
  eeAddress = (PromAddress)((int)eeAddress + 4);

  EEPROM.get(eeAddress, servoCenter);
  this->rXCenter = servoCenter;
  eeAddress = (PromAddress)((int)eeAddress + 4);

  EEPROM.get(eeAddress, servoCenter);
  this->rZCenter = servoCenter;

  //SerialTerminal->println("Eye Calibration Variables read from Prom");
}

/*
   Function to make the eyes parallax to a single point
   on the screen. Parameter screenDotPos gives the coordinates of
   the desired position.
*/
void Eyes::ParallaxEyesToPos(BLA::Matrix<4> screenDotPos, BLA::Matrix<4,4> gLS, BLA::Matrix<4,4> gRS)
{
  BLA::Matrix<4> leftDotPos = gLS * screenDotPos;
  BLA::Matrix<4> rightDotPos = gRS * screenDotPos;

  // calling private helper function to write to servos.
  this->parallax(leftDotPos, rightDotPos);
}

/*
   Function to calibrate the eye servos. The function takes eyeCalCommand
   as a parameter, which is the signal transmitted by the Windows API.
   This character is used as input to the calibration sequence.
*/
void Eyes::CalibrateEyes(char eyeCalCommand, BLA::Matrix<4,4> gLS, BLA::Matrix<4,4> gRS)
{
  // Dot position array corresponding to desired screen location
  // set to zero for calibration to calibrate to center of screen.
  BLA::Matrix<4> screenDotPos = {0, 0, 0, 1};

  if (eyeCalCommand == '1' || eyeCalCommand == '2' || eyeCalCommand == '3' || eyeCalCommand == '4') {
    // converting integer to enumeration
    int motorChoice = eyeCalCommand - '0';
    calibrationMotor = static_cast<EyeMotor>(motorChoice);
  }

  // switch case to calibrate the center location of respective eye servo.
  // the center locations are incremented/decremented by 5 microseconds for each press.
  switch (calibrationMotor)
  {
    case (rightZ) : {
        if (eyeCalCommand == 'u') {
          this->rZCenter -= 2;
        }
        if (eyeCalCommand == 'd') {
          this->rZCenter += 2;
        }
        break;
      }
    case (leftZ) : {
        if (eyeCalCommand == 'u') {
          this->lZCenter += 2;
        }
        if (eyeCalCommand == 'd') {
          this->lZCenter -= 2;
        }
        break;
      }
    case (rightX) : {
        if (eyeCalCommand == 'u') {
          this->rXCenter += 2;
        }
        if (eyeCalCommand == 'd') {
          this->rXCenter -= 2;
        }
        break;
      }
    case (leftX) : {
        if (eyeCalCommand == 'u') {
          this->lXCenter += 2;
        }
        if (eyeCalCommand == 'd') {
          this->lXCenter -= 2;
        }
        break;
      }
    default : break;
  }

  this->ParallaxEyesToPos(screenDotPos, gLS, gRS);
}
