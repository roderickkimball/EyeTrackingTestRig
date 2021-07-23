/*
   Robot source file containing definitions for all function declarations in
   header file.
*/

#include "Robot.h"

/*
   Robot class constructor.
*/
Robot::Robot()
{
  this->robotState = MenuMode;
  this->screenDotPos = {0, 0, 0, 1};
  SerialTerminal->println("Created Robot Object");
}

/*
   Initialization function for the robot class that
   will create the robot eye, shoulder and neck objects.
*/
void Robot::init()
{
  this->robotEyes.init();
  this->robotShoulder.init();
  this->robotNeck.init();

  this->robotEyes.ReadEyeCalibrationVariablesFromProm();

  this->robotShoulder.ReadShoulderPositionFromProm();

  this->robotNeck.ReadNeckPositionFromProm('c');
  this->robotNeck.ReadNeckPositionFromProm('l');

  this->updateKinematicChain();

  SerialTerminal->println("Finished ROBOT init");
}

/*
   Function to get a command from the SerialTerminal
   that obtains commands from the windows API
*/
char Robot::getSerialCommand()
{
  char command = ' ';
  if (SerialTerminal->available()) {
    command = SerialTerminal->read();
  }
  return command;
}

/*
   Function that will be used to update the kinematic chain instance.
   Calling this function will recalculate the entire kinematic chain for the robot
   going from the eyes to the screen. Therefore this function must be called whenever the robot has moved.
*/
void Robot::updateKinematicChain()
{
  // obtaining instance of kinematic chain class
  // this instance will then help determine the coordinate values
  // for left and right eyes.
  KinematicChain* tfMatrix = KinematicChain::getInstance();

  tfMatrix->UpdateKinematicChain();
}

/*
   Function to obtain serial communication signals from Windows API.
   Based on the predefined code established here, functionality is chosen.
*/
void Robot::runMenuModeState()
{
  char command = this->getSerialCommand();

  if (command == 'm') { //returns current mode (for now should always be menuMode)
    SerialTerminal->println("Mode: " + stateNames[this->GetState()]);
  }

  else if (command == 's') { //changes state
    int newstate = SerialTerminal->parseInt();
    this->SetState(newstate);
    SerialTerminal->println("tring to go to state: " + stateNames[newstate]);
  }
}

/*
   Function to obtain calibration commands from API and
   subsequently pass message onto Eye subsystem class.
*/
void Robot::runServoCalibrationState()
{
  char eyeCalCommand = this->getSerialCommand();

  // calling robotEyes object to perform calibration of eyes.
  this->robotEyes.CalibrateEyes(eyeCalCommand);
  // if calibration has been stopped, return to MenuMode;
  if (eyeCalCommand == 'b')
  {
    this->robotEyes.WriteEyeCalibrationVariablesToProm();
    this->SetState(static_cast<int>(MenuMode));
  }
}

/*
   Function to enable manual control of the eye subsystem
*/
void Robot::runServoManualState()
{
  char command = this->getSerialCommand();

  if (command == 'g') {
    float goX = SerialTerminal->parseFloat();
    float goZ = SerialTerminal->parseFloat();
    this->screenDotPos(0) = goX;
    this->screenDotPos(2) = goZ;

    this->robotEyes.ParallaxEyesToPos(this->screenDotPos);
  }
  else if (command == 'b') {
    this->SetState(static_cast<int>(MenuMode));
  }
}

/*
   Shoulder Home state will home the steppers. Homing the steppers will include moving the
   steppers till they hit the limit switches. Hitting the limit switches will reset the zero locations
   of the steppers.
*/
void Robot::runShoulderHomeState()
{
  // Home all steppers function will return false when steppers have finished homing
  bool calibrating = this->robotShoulder.HomeShoulder();

  char command = this->getSerialCommand();


  if (command == 'b' || calibrating == false)
  {
    this->updateKinematicChain();
    this->robotShoulder.WriteShoulderPositionToProm();
    this->SetState(static_cast<int>(MenuMode));
  }
}

/*
   Shoulder Manual state will allow the user to obtain manual control of the shoulder
   steppers.
*/
void Robot::runShoulderManualState()
{
  char command = this->getSerialCommand();

  if (command == 'g') {
    float goX = SerialTerminal->parseFloat();
    float goY = SerialTerminal->parseFloat();
    float goZ = SerialTerminal->parseFloat();

    // When shoulders move from their home position, all positions are -ve, so we have to invert the values
    // so the input from API can be positive.
    this->robotShoulder.MoveShoulderToPosition(-goX, -goY, -goZ);
  }
  else if (command == 'b') {
    this->updateKinematicChain();
    this->robotShoulder.WriteShoulderPositionToProm();
    this->SetState(static_cast<int>(MenuMode));
  }
}

/*
   Function to obtain calibration commands for the neck from the API
   and subsequently pass them onto the neck object.
*/
void Robot::runNeckCalibrationState()
{
  char neckCalCommand = this->getSerialCommand();

  // calling robotEyes object to perform calibration of eyes.
  this->robotNeck.CalibrateNeck(neckCalCommand);
  // if calibration has been stopped, return to MenuMode;
  if (neckCalCommand == 'b')
  {
    this->updateKinematicChain();
    // writing calibration variables to prom
    this->robotNeck.WriteNeckPositionToProm('c');
    // writing last neck position variables to prom
    this->robotNeck.WriteNeckPositionToProm('l');
    this->SetState(static_cast<int>(MenuMode));
  }
}

/*
   Function to obtain manual control of the neck.
   X and Y positions have to be scaled between -1.0 and 1.0
*/
void Robot::runNeckManualState()
{
  char command = this->getSerialCommand();

  if (command == 'g')
  {
    float goX = SerialTerminal->parseFloat();
    float goY = SerialTerminal->parseFloat();
    float goZ = SerialTerminal->parseFloat();

    this->robotNeck.MoveNeckManually(goX, goY, goZ);
  }

  else if (command == 'b')
  {    
    this->updateKinematicChain();
    this->robotNeck.WriteNeckPositionToProm('l');
    this->SetState(static_cast<int>(MenuMode));
  }
}

/*
   Function to set the state for the Robot.
*/
void Robot::SetState(int stateNumber)
{
  this->robotState = static_cast<StateMachineState>(stateNumber);
  SerialTerminal->println("Set New State");
}

/*
   Function to get the current state of the Robot.
*/
int Robot::GetState()
{
  return static_cast<int>(this->robotState);
  SerialTerminal->println("Current State is " + String(this->robotState));
}

/*
   Function to determine the state of the robot, and run the
   corresponding action of the state.
*/
void Robot::RunState()
{
  switch (this->robotState)
  {
    case (MenuMode):
      {
        (this)->runMenuModeState();
        break;
      }
    case (ServoCalibration):
      {
        (this)->runServoCalibrationState();
        break;
      }
    case (ServoManual):
      {
        (this)->runServoManualState();
        break;
      }
    case (StepperHome):
      {
        (this)->runShoulderHomeState();
        break;
      }
    case (StepperManual):
      {
        (this)->runShoulderManualState();
        break;
      }
    case (NeckCalibration):
      {
        (this)->runNeckCalibrationState();
        break;
      }
    case (MoveToCalibrationState):
      {
        (this)->robotNeck.MoveToCalibratedPosition();
        this->updateKinematicChain();
        this->SetState(static_cast<int>(MenuMode));
        break;
      }
    case (Neck):
      {
        (this)->runNeckManualState();
        break;
      }
    default :
      {
        break;
      }

      //    case (SetCoordinates):
      //      {
      //        runSetCoordinatesState();
      //        break;
      //      }
      //    case (FindCoordinates):
      //      {
      //        runFindCoordinatesState();
      //        break;
      //      }
      //    case (Auto):
      //      {
      //        runAutoState();
      //        break;
      //      }



      //    case (SpinnyBoi):
      //      {
      //        runSpinnyBoiState();
      //        break;
      //      }
  }

}
