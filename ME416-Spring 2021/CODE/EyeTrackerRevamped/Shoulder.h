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
    float stepsPerM;

    bool zeroStepperX();
    bool zeroStepperY();
    bool zeroStepperZ();

    void updateShoulderPosition();

  protected:
    /*
      S - will be the screen frame of reference with the zero coordinate at the center of the screen,
      X will be to the right, Z up, and Y into the screen
      B - will be the origin frame of reference with the zero coordinate at the position where the
      stages of the 3-axis shoulder steppers hit the limit switches or zero position
      C - will be the origin frame of reference located at the center of the Z-axis stage,
      that the neck and eye mechanisms are mounted to.
    */
    BLA::Matrix<4, 4> gBS, gBC, gCS;

  public:
    Shoulder();
    ~Shoulder();
    void init();
    bool HomeShoulder();
    void MoveShoulderToPosition(float x, float y, float z);
    float GetShoulderPosition(char desiredStepper);
    void WriteShoulderPositionToProm();
    void ReadShoulderPositionFromProm();
    // functions to get the inverse transformation matrices for the shoulder subsystem
    BLA::Matrix<4,4> GetInverseShoulderTransformation();
};

#endif
