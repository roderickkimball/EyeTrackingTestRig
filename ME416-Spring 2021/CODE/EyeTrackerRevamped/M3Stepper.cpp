#include "M3Stepper.h"

M3Stepper::M3Stepper()
{
}

void M3Stepper::init(int directionPin, int pulsePin)
{
  this->_direction = directionPin;
  this->_pulse = pulsePin;
  
  pinMode(this->_direction, OUTPUT);
  pinMode(this->_pulse, OUTPUT);
  
  this->_currentPosition = 0;
}

void M3Stepper::SetCurrentPosition(long stepPosition)
{
  this->_currentPosition = stepPosition;
  this->_targetPosition = stepPosition;
}

long M3Stepper::CurrentPosition()
{
  return this->_currentPosition;
}

void M3Stepper::SetSpeed(float desiredSpeed)
{
  this->_speed = desiredSpeed;
}

float M3Stepper::GetSpeed()
{
  return this->_speed;
}

bool M3Stepper::IsReached()
{
  return (this->_currentPosition == this->_targetPosition);
}

void M3Stepper::MoveTo(long absolute)
{
  this->_targetPosition = absolute;
}

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
