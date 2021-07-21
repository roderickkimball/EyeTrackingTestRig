/*
   Shoulder class header file. This class is responsible for encapsulating the stepper objects
   that make up the shoulder of the robot. The class also has public member functions for
   controlling and homing the shoulder steppers.
*/

#ifndef _SHOULDER_H
#define _SHOULDER_H

#include <AccelStepper.h>
#include <EEPROM.h>
#include "KinematicChain.h"
#include "Function.h"

/*
   Shoulder class declaration. The class consists of private stepper objects for each
   shoulder axis. The public functions allow for initialization, homing, manual control and
   accessors for the current position of each stepper.
   TO_DO: Make destructor for the pointer objects. - in progress
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
    ~Shoulder();
    void init();
    bool HomeShoulder();
    void MoveShoulderToPosition(int x, int y, int z);
    int GetShoulderPosition(char desiredStepper);
    void WriteShoulderPositionToProm();
    void ReadShoulderPositionFromProm();
};

#endif
