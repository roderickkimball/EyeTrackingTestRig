/*
   Shoulder class header file. This class is responsible for encapsulating the stepper objects
   that make up the shoulder of the robot. The class also has public member functions for
   controlling and homing the shoulder steppers.
*/

#ifndef _SHOULDER_H
#define _SHOULDER_H

#include <AccelStepper.h>
#include "KinematicChain.h"
#include "Function.h"

/*
   Shoulder class declaration. The class consists of private stepper objects for each
   shoulder axis. The public functions allow for initialization, homing, manual control and
   accessors for the current position of each stepper.
*/
class Shoulder
{
    // individual axes stepper objects
  private:
    AccelStepper *xStepper, *yStepper, *zStepper;

    bool zeroStepperX();
    void zeroStepperY();
    void zeroStepperZ();

  public:
    Shoulder();
    void init();
    bool HomeAllSteppers();
    void MoveStepperToPosition(char desiredStepper, float desiredStepPosition);
    float GetStepperPosition(char desiredStepper);
    void SetStepperPosition(char desiredStepper, float currentStepPosition);

};

#endif
