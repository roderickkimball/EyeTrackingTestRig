#ifndef _M3STEPPER_H
#define _M3STEPPER_H

#include <Arduino.h>

/*
 * M3 Stepper class uses HIGH_LOW pulses to move the steppers one step at a time.
 * Class allows us to set the speed of the respective stepper controllers as well.
 * Using this class ensures that the steppers dont miss any steps, especially when accuracy is important.
 */
class M3Stepper{

  private: 
    long _currentPosition;
    long _targetPosition;

    int _direction, _pulse;

    float _speed;

  public: 
    M3Stepper();

    void init(int directionPin, int pulsePin);
    
    void SetCurrentPosition(long stepPosition);
    long CurrentPosition();

    void SetSpeed(float desiredSpeed);
    float GetSpeed();

    bool IsReached();

    void MoveTo(long absolute);

    void RunStepper();
};

#endif
