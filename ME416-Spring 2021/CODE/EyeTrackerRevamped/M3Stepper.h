#ifndef _M3STEPPER_H
#define _M3STEPPER_H

#include <Arduino.h>

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
