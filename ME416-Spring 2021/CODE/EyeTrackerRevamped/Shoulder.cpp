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
}

/*
   Private helper function to zero stepper X.
   Referenced by HomeStepper function.
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
   Function to home all the shoulder steppers and
   set their respective starting position.
*/
bool Shoulder::HomeAllSteppers()
{
  // boolean indicating whether calibration is ongoing.
  bool calibrating = true;
  
  bool xCal = this->zeroStepperX();
  bool yCal = this->zeroStepperY();
  bool zCal = this->zeroStepperZ();

  if (xCal == true && yCal == true && zCal == true)
  {
    calibrating = false;
    return calibrating;
  }
}
