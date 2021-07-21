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
  this->xStepper->setAcceleration(1000);

  this->yStepper->setMaxSpeed(10000);
  this->yStepper->setAcceleration(1000);

  this->zStepper->setMaxSpeed(10000);
  this->zStepper->setAcceleration(1000);

  this->stepsPerMM = 89;
}

/*
   Private helper function to zero stepper X.
   Referenced by HomeShoulder function.
*/
bool Shoulder::zeroStepperX()
{
  int limitSwitchState = digitalRead(xEnd);

  if (limitSwitchState == HIGH)
  {
    this->xStepper->move(-1000);
    this->xStepper->run();
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
    this->yStepper->move(-1000);
    this->yStepper->run();
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
    this->zStepper->move(1000);
    this->zStepper->run();
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
  Private helper function to update the current position of the
  stepper axes in the kinematic chain instance.
*/
void Shoulder::updateKinematicChain()
{
  // obtaining instance of kinematic chain class
  // this instance will then help determine the coordinate values
  // for left and right eyes.
  KinematicChain* tfMatrix = KinematicChain::getInstance();

  tfMatrix->SetStepperPositions(this->GetShoulderPosition('x'), this->GetShoulderPosition('y'), this->GetShoulderPosition('z'));
  tfMatrix->UpdateKinematicChain();
}

/*
   Function to home all the shoulder steppers and
   set their respective starting position.
*/
bool Shoulder::HomeShoulder()
{
  // boolean indicating whether calibration is ongoing.
  bool calibrating = true;

  bool xCal = this->zeroStepperX();
  bool yCal = this->zeroStepperY();
  bool zCal = this->zeroStepperZ();

  // if the stepper axes have finished calibrating.
  if (xCal == true && yCal == true && zCal == true)
  {
    calibrating = false;
    this->updateKinematicChain();
    return calibrating;
  }
}

/*
   Function to move the stepper axes to the desired x,y,z position
   transmitted through the API.
*/
void Shoulder::MoveShoulderToPosition(int x, int y, int z)
{
  this->xStepper->moveTo(x * this->stepsPerMM);
  this->yStepper->moveTo(y * this->stepsPerMM);
  this->zStepper->moveTo(z * this->stepsPerMM);

  this->xStepper->run();
  this->yStepper->run();
  this->zStepper->run();
}

/*
   Function get the current position of the desired stepper axes.
*/
int Shoulder::GetShoulderPosition(char desiredStepper)
{
  switch (desiredStepper) {
    case 'x':
      return this->xStepper->currentPosition();
    case 'y':
      return this->yStepper->currentPosition();
    case 'z':
      return this->zStepper->currentPosition();
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

  Serial.println("Shoulder position written to Prom");
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

  EEPROM.get(eeAddress, stepperPosition);
  this->yStepper->setCurrentPosition(stepperPosition);
  eeAddress = (PromAddress)((int)eeAddress + 4);

  EEPROM.get(eeAddress, stepperPosition);
  this->zStepper->setCurrentPosition(stepperPosition);

  this->updateKinematicChain();
}
