#include "NeckClass.h"

NeckMotor neckCalibrationMotor = front;

// Stepper Configuration variables.
float spoolRadius = 0.0071;                      // radius of the spool that holds the thread
float spoolCircumference = 2.0 * M_PI * spoolRadius; // circumference of the spool
float mmPerRevs = spoolCircumference;            // mm of thread that each revolution of spool will loosen or tighten
float stepsPerRev = 3838;                        // number of steps in one revolution for the neck steppers
float mmPerStep = mmPerRevs / stepsPerRev;       // mm of thread released in one step

// The known stepper pos after calibration. = (known cable length)/ mmPerStep
// 0.127 is the length of calibration stick and hence the known cable length.
float knownStepperPos = 0.127 / mmPerStep;
// gain for the neck motion
float motionGain = 0.1;

/*
   Neck Class constructor.
*/
RobotNeck::RobotNeck()
{
}

/*
   Neck Class destructor to clear pointer objects.
*/
RobotNeck::~RobotNeck()
{
  delete[] this->frontStepper;
  delete[] this->backRightStepper;
  delete[] this->backLeftStepper;
}

/*
   Function to initialize the Neck objects. Function also sets the maxspeed
   and acceleration for the stepper objects as well.
*/
void RobotNeck::init()
{
  this->frontStepper = new AccelStepper(AccelStepper::DRIVER, frontPulse, frontDir);
  this->backLeftStepper = new AccelStepper(AccelStepper::DRIVER, backLeftPulse, backLeftDir);
  this->backRightStepper = new AccelStepper(AccelStepper::DRIVER, backRightPulse, backRightDir);

  this->frontStepper->setMaxSpeed(3000);
  this->frontStepper->setAcceleration(3000);

  this->backLeftStepper->setMaxSpeed(3000);
  this->backLeftStepper->setAcceleration(3000);

  this->backRightStepper->setMaxSpeed(3000);
  this->backRightStepper->setAcceleration(3000);

  this->calibrationStepperPosFront = 0.0;
  this->calibrationStepperPosBackRight = 0.0;
  this->calibrationStepperPosBackLeft = 0.0;

  this->lastStepperPosFront = 0.0;
  this->lastStepperPosBackRight = 0.0;
  this->lastStepperPosBackLeft = 0.0;

  /*
    Length of the spring in meters. Gets updated after MoveToCalState()
    It is 127mm before calibration.
  */
  this->lSpring = 0.127;
}

/*
   Function to calibrate the neck. Calibration sequence attempts to
   level the neck to a known height indicated by the knownStepperPos variable
   and the calibration stick.
*/
void RobotNeck::CalibrateNeck(char neckCalCommand)
{
  if (neckCalCommand == 'r') {
    if (neckCalibrationMotor == front) neckCalibrationMotor = backRight;
    if (neckCalibrationMotor == backRight) neckCalibrationMotor = backLeft;
  }
  if (neckCalCommand == 'l') {
    if (neckCalibrationMotor == backRight) neckCalibrationMotor = front;
    if (neckCalibrationMotor == backLeft) neckCalibrationMotor = backRight;
  }

  switch (neckCalibrationMotor) {
    case (front): {
        // 'u' will loosen and 'd' will tighten
        if (neckCalCommand == 'u') {
          this->frontStepper->runToNewPosition(knownStepperPos + 100);
          this->frontStepper->setCurrentPosition(knownStepperPos);
        }
        if (neckCalCommand == 'd') {
          this->frontStepper->runToNewPosition(knownStepperPos - 100);
          this->frontStepper->setCurrentPosition(knownStepperPos);
        }
        break;
      }
    case (backRight): {
        // 'u' will loosen and 'd' will tighten
        if (neckCalCommand == 'u') {
          this->backRightStepper->runToNewPosition(knownStepperPos + 100);
          this->backRightStepper->setCurrentPosition(knownStepperPos);
        }
        if (neckCalCommand == 'd') {
          this->backRightStepper->runToNewPosition(knownStepperPos - 100);
          this->backRightStepper->setCurrentPosition(knownStepperPos);
        }
        break;
      }
    case (backLeft): {
        // 'u' will loosen and 'd' will tighten
        if (neckCalCommand == 'u') {
          this->backLeftStepper->runToNewPosition(knownStepperPos + 100);
          this->backLeftStepper->setCurrentPosition(knownStepperPos);
        }
        if (neckCalCommand == 'd') {
          this->backLeftStepper->runToNewPosition(knownStepperPos - 100);
          this->backLeftStepper->setCurrentPosition(knownStepperPos);
        }
        break;
      }
    default: break;
  }
}

/*
   Function to move the neck manually. The function takes in two parameters that determine
   pitch and roll of the neck.
*/
void RobotNeck::MoveNeckManually(float x, float y)
{
  float PhiS = atan2(y, x) - M_PI_2;
  float PhiR = (3.14159/6) * sqrt(sq (y) + sq(x));
  //float PhiD = .......

  // obtaining instance of kinematic chain class
  // this instance will then help determine the coordinate values
  // for left and right eyes.
  KinematicChain* tfMatrix = KinematicChain::getInstance();

  tfMatrix->UpdateNeckTransformationMatrix(PhiR, PhiS, this->lSpring);
  
  BLA::Matrix<4, 4> gB1_P1 = tfMatrix->GetgB1P1();
  BLA::Matrix<4, 4> gB2_P2 = tfMatrix->GetgB2P2();
  BLA::Matrix<4, 4> gB3_P3 = tfMatrix->GetgB3P3();

  float frontCableLength = sqrt(sq(gB1_P1(0,3)) + sq(gB1_P1(1,3)) + sq(gB1_P1(2,3)));
  float backRightCableLength = sqrt(sq(gB2_P2(0,3)) + sq(gB2_P2(1,3)) + sq(gB2_P2(2,3)));
  float backLeftCableLength = sqrt(sq(gB3_P3(0,3)) + sq(gB3_P3(1,3)) + sq(gB3_P3(2,3)));

  this->frontStepper->moveTo(frontCableLength / mmPerStep);
  this->backRightStepper->moveTo(backRightCableLength / mmPerStep);
  this->backLeftStepper->moveTo(backLeftCableLength / mmPerStep);

  this->lastStepperPosFront = this->frontStepper->currentPosition();
  this->lastStepperPosBackRight = this->backRightStepper->currentPosition();
  this->lastStepperPosBackLeft = this->backLeftStepper->currentPosition();
}

/*
   Function to move the neck to the recorded calibrated position.
   The calibration positions are recorded inside the neck object for all three stepper motors.
*/
void RobotNeck::MoveToCalibratedPosition()
{
  // will load all the calibration variables in prom to corresponding private variables.
  this->ReadNeckPositionFromProm('c');

  this->frontStepper->runToNewPosition(this->calibrationStepperPosFront);
  this->frontStepper->setCurrentPosition(this->calibrationStepperPosFront);

  this->backRightStepper->runToNewPosition(this->calibrationStepperPosBackRight);
  this->backRightStepper->setCurrentPosition(this->calibrationStepperPosBackRight);

  this->backLeftStepper->runToNewPosition(this->calibrationStepperPosBackLeft);
  this->backLeftStepper->setCurrentPosition(this->calibrationStepperPosBackLeft);

  this->lSpring = ((this->calibrationStepperPosFront * mmPerStep) + (this->calibrationStepperPosBackRight * mmPerStep) + (this->calibrationStepperPosBackLeft * mmPerStep)) / 3.0;
}

/*
   Function to write the current neck stepper positions to the EEPROM.
   This function can save the positions to either the calibration variables in the PROM
   or the last stepper position variables in the PROM based on the character input.
   'c' for calibration. 'l' for last stepper.
*/
void RobotNeck::WriteNeckPositionToProm(char calibrationOrLastPosition)
{
  PromAddress eeAddress;
  if (calibrationOrLastPosition == 'c')
  {
    eeAddress = FrontNeckCalibrationStepperPosition;
  }
  else if (calibrationOrLastPosition == 'l')
  {
    eeAddress = FrontNeckLastStepperPosition;
  }
  EEPROM.put(eeAddress, this->frontStepper->currentPosition());
  eeAddress = (PromAddress)((int)eeAddress + 4);
  EEPROM.put(eeAddress, this->backRightStepper->currentPosition());
  eeAddress = (PromAddress)((int)eeAddress + 4);
  EEPROM.put(eeAddress, this->backLeftStepper->currentPosition());
  delay(100);

  Serial.println("Neck variables written!");
}

/*
   Function to read the either the calibration stepper positions
   or the last neck stepper positions from the EEPROM. The character input into the function
   determines which variables are read.
*/
void RobotNeck::ReadNeckPositionFromProm(char calibrationOrLastPosition)
{
  PromAddress eeAddress;
  long int stepperPosition = 0;

  if (calibrationOrLastPosition == 'c')
  {
    eeAddress = FrontNeckCalibrationStepperPosition;


    EEPROM.get(eeAddress, stepperPosition);
    this->calibrationStepperPosFront = stepperPosition;
    eeAddress = (PromAddress)((int)eeAddress + 4);
    EEPROM.get(eeAddress, stepperPosition);
    this->calibrationStepperPosBackRight = stepperPosition;
    eeAddress = (PromAddress)((int)eeAddress + 4);
    EEPROM.get(eeAddress, stepperPosition);
    this->calibrationStepperPosBackLeft = stepperPosition;
  }
  else if (calibrationOrLastPosition == 'l')
  {
    eeAddress = FrontNeckLastStepperPosition;

    EEPROM.get(eeAddress, stepperPosition);
    this->lastStepperPosFront = stepperPosition;
    eeAddress = (PromAddress)((int)eeAddress + 4);
    EEPROM.get(eeAddress, stepperPosition);
    this->lastStepperPosBackRight = stepperPosition;
    eeAddress = (PromAddress)((int)eeAddress + 4);
    EEPROM.get(eeAddress, stepperPosition);
    this->lastStepperPosBackLeft = stepperPosition;
  }
  Serial.println("Neck variables read!");
}
