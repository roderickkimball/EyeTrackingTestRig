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
    float stepsPerMM;

    bool zeroStepperX();
    bool zeroStepperY();
    bool zeroStepperZ();
    void updateKinematicChain();

  public:
    Shoulder();
    void init();
    bool HomeAllSteppers();
    void MoveSteppersToPosition(int x, int y, int z);
    int GetStepperPosition(char desiredStepper);
};

#endif
