/*
   Source file for Shoulder class. Defines all the public member functions
   declared in Shoulder.h
*/
#include "Shoulder.h"

/*
   Shoulder class constructor
*/
Shoulder::Shoulder()
{
}

/*
   Shoulder class destructor - necessary to free the memory for pointers
*/
Shoulder::~Shoulder()
{
  delete[] this->xStepper;
  delete[] this->yStepper;
  delete[] this->zStepper;
}


/*
   Initialization function for the shoulder object.
*/
void Shoulder::init()
{
  this->xStepper = new AccelStepper(AccelStepper::DRIVER, xPulse, xDir);
  this->yStepper = new AccelStepper(AccelStepper::DRIVER, yPulse, yDir);
  this->zStepper = new AccelStepper(AccelStepper::DRIVER, zPulse, zDir);

  // setting default max speed and acceleration for stepper objects
  this->xStepper->setMaxSpeed(10000);
  this->xStepper->setAcceleration(2000);

  this->yStepper->setMaxSpeed(10000);
  this->yStepper->setAcceleration(2000);

  this->zStepper->setMaxSpeed(10000);
  this->zStepper->setAcceleration(2000);

  // this is the number of steps the steppers have to take
  // to travel 1 meter. Empirically determined.
  this->stepsPerM = 89000;

  // setting initial values to the shoulder transformation matrices
  this->gBS = KinematicChain::xform(0, 0, 0, (0.14123), 0.562, (0.23123));
  this->gBC = KinematicChain::xform(0, 0, 0, 0, 0, 0);

  this->gCS = gBC.Inverse() * gBS;
}

/*
   Private helper function to zero stepper X.
   Referenced by HomeShoulder function.
*/
bool Shoulder::zeroStepperX()
{
  int limitSwitchState = digitalRead(xEnd);

  // until the limit switch trips keep moving forward
  if (limitSwitchState == HIGH)
  {
    this->xStepper->move(5000);
  }
  else if (limitSwitchState == LOW)
  {
    this->xStepper->setCurrentPosition(0);
    bool zeroed = true;
    return zeroed;
  }
  bool zeroed = false;
  return zeroed;
}

/*
   Private helper function to zero stepper Y.
   Referenced by HomeStepper function.
*/
bool Shoulder::zeroStepperY()
{
  int limitSwitchState = digitalRead(yEnd);

  if (limitSwitchState == HIGH)
  {
    this->yStepper->move(5000);
  }
  else if (limitSwitchState == LOW)
  {
    this->yStepper->setCurrentPosition(0);
    bool zeroed = true;
    return zeroed;
  }
  bool zeroed = false;
  return zeroed;
}

/*
   Private helper function to zero stepper Z.
   Referenced by HomeStepper function.
*/
bool Shoulder::zeroStepperZ()
{
  int limitSwitchState = digitalRead(zEnd);

  if (limitSwitchState == HIGH)
  {
    this->zStepper->move(5000);
  }
  else if (limitSwitchState == LOW)
  {
    this->zStepper->setCurrentPosition(0);
    bool zeroed = true;
    return zeroed;
  }
  bool zeroed = false;
  return zeroed;
}

/*
   Function to update the shoulder positions in the kinematic chain.
*/
void Shoulder::updateShoulderPosition()
{
  /*
      y is negative here because in all frames of reference
      forward is positive y. since the stepper always move back
      from zero position, we invert the sign of y.
  */
  this->gBC = KinematicChain::xform(0, 0, 0, (this->GetShoulderPosition('x')), -(this->GetShoulderPosition('y')), (this->GetShoulderPosition('z')));
}

/*
   Function to home all the shoulder steppers and
   set their respective starting position.
*/
bool Shoulder::HomeShoulder()
{
  // boolean indicating whether calibration is ongoing.
  bool calibrating = true;

  bool yCal = this->zeroStepperY();
  this->yStepper->run();

  bool xCal, zCal;

  if (yCal == true) {
    xCal = this->zeroStepperX();
    zCal = this->zeroStepperZ();

    this->xStepper->run();
    this->zStepper->run();
  }

  // if the stepper axes have finished calibrating.
  if (xCal == true && yCal == true && zCal == true)
  {
    this->updateShoulderPosition();
    calibrating = false;
    return calibrating;
  }
  return calibrating;
}

/*
   Function to move the stepper axes to the desired x,y,z position
   transmitted through the API.
*/
void Shoulder::MoveShoulderToPosition(float x, float y, float z)
{
  this->xStepper->runToNewPosition(x * this->stepsPerM);
  this->yStepper->runToNewPosition(y * this->stepsPerM);
  this->zStepper->runToNewPosition(z * this->stepsPerM);

  this->updateShoulderPosition();

  //SerialTerminal->println(this->xStepper->currentPosition());
}

/*
 * Gets the inverse transformation matrix that goes from the center of the z axis stepper stage
 * to the center of the screen.
 */
BLA::Matrix<4,4> Shoulder::GetInverseShoulderTransformation()
{
  this->gCS = gBC.Inverse() * gBS;
  return this->gCS;
}

/*
   Function get the current position of the desired stepper axes.
*/
float Shoulder::GetShoulderPosition(char desiredStepper)
{
  switch (desiredStepper) {
    case 'x':
      return -(this->xStepper->currentPosition() / this->stepsPerM);
    case 'y':
      return -(this->yStepper->currentPosition() / this->stepsPerM);
    case 'z':
      return -(this->zStepper->currentPosition() / this->stepsPerM);
    default:
      return NULL;
  }
}

/*
   Function to write the current shoulder position
   to the EEPROM
*/
void Shoulder::WriteShoulderPositionToProm()
{
  PromAddress eeAddress = XStepperPosition;

  EEPROM.put(eeAddress, this->xStepper->currentPosition());
  eeAddress = (PromAddress)((int)eeAddress + 4);

  EEPROM.put(eeAddress, this->yStepper->currentPosition());
  eeAddress = (PromAddress)((int)eeAddress + 4);

  EEPROM.put(eeAddress, this->zStepper->currentPosition());

  //SerialTerminal->println("Shoulder position written to Prom");
}

/*
   Function to read the shoulder position
   from the EEPROM
*/
void Shoulder::ReadShoulderPositionFromProm()
{
  PromAddress eeAddress = XStepperPosition;
  long int stepperPosition = 0;

  EEPROM.get(eeAddress, stepperPosition);
  this->xStepper->setCurrentPosition(stepperPosition);
  eeAddress = (PromAddress)((int)eeAddress + 4);

  //SerialTerminal->println(stepperPosition);

  EEPROM.get(eeAddress, stepperPosition);
  this->yStepper->setCurrentPosition(stepperPosition);
  eeAddress = (PromAddress)((int)eeAddress + 4);

  EEPROM.get(eeAddress, stepperPosition);
  this->zStepper->setCurrentPosition(stepperPosition);

  this->updateShoulderPosition();

  //SerialTerminal->println("Shoulder positions read from Prom");
}
