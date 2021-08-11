/*
   M3 stepper source file
*/

#include "M3Stepper.h"

M3Stepper::M3Stepper()
{
}

/*
   Class initialization function.
*/
void M3Stepper::init(int directionPin, int pulsePin)
{
  this->_direction = directionPin;
  this->_pulse = pulsePin;

  pinMode(this->_direction, OUTPUT);
  pinMode(this->_pulse, OUTPUT);

  this->_currentPosition = 0;
}

/*
   Sets the current position of the stepper.
*/
void M3Stepper::SetCurrentPosition(long stepPosition)
{
  this->_currentPosition = stepPosition;
  this->_targetPosition = stepPosition;
}

/*
   Gets the current position of the stepper.
*/
long M3Stepper::CurrentPosition()
{
  return this->_currentPosition;
}

/*
   Sets the current speed for the stepper.
*/
void M3Stepper::SetSpeed(float desiredSpeed)
{
  this->_speed = desiredSpeed;
}

/*
   Gets the current speed of the stepper.
*/
float M3Stepper::GetSpeed()
{
  return this->_speed;
}

/*
   Checks if the stepper has reached the target position
*/
bool M3Stepper::IsReached()
{
  return (this->_currentPosition == this->_targetPosition);
}

/*
   Sets the target position for the stepper.
*/
void M3Stepper::MoveTo(long absolute)
{
  this->_targetPosition = absolute;
}

/*
   Pulses the stepper from low to high moving one step.
*/
void M3Stepper::RunStepper()
{
  if (!(this->IsReached()))
  {
    if (this->_currentPosition < this->_targetPosition)
    {
      digitalWrite(this->_direction, HIGH);
      this->_currentPosition++;
    }
    else if (this->_currentPosition > this->_targetPosition)
    {
      digitalWrite(this->_direction, LOW);
      this->_currentPosition--;
    }
    digitalWrite(this->_pulse, HIGH);
    digitalWrite(this->_pulse, LOW);
  }
  else
  {
    return;
  }
}
