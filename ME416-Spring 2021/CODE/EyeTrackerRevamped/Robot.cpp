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
  Serial.println("Created Robot Object");
}

/*
   Initialization function for the robot class that
   will create the robot eye, shoulder and neck objects.
*/
void Robot::init()
{
  this->robotEyes.init();
  this->robotShoulder.init();
  Serial.println("Finished Eye init");
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
  this->robotEyes.CalibrateServos(eyeCalCommand);
  // if calibration has been stopped, return to MenuMode;
  if (eyeCalCommand == 'b')
  {
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
    int goX = SerialTerminal->parseInt();
    int goZ = SerialTerminal->parseInt();
    this->screenDotPos(0) = goX;
    this->screenDotPos(2) = goZ;

    this->robotEyes.ParallaxServosToPos(this->screenDotPos);
  }
  else if (command == 'b') {
    this->SetState(static_cast<int>(MenuMode));
  }
}

/*
   Stepper Home state will home the steppers. Homing the steppers will include moving the
   steppers till they hit the limit switches. Hitting the limit switches will reset the zero locations
   of the steppers.
*/
void Robot::runStepperHomeState()
{
  // Home all steppers function will return false when steppers have finished homing
  bool calibrating = this->robotShoulder.HomeAllSteppers();

  char command = this->getSerialCommand();


  if (command == 'b' || calibrating == false)
  {
    this->SetState(static_cast<int>(MenuMode));
  }
}

/*
   Function to set the state for the Robot.
*/
void Robot::SetState(int stateNumber)
{
  this->robotState = static_cast<StateMachineState>(stateNumber);
  Serial.println("Set New State");
}

/*
   Function to get the current state of the Robot.
*/
int Robot::GetState()
{
  return static_cast<int>(this->robotState);
  Serial.println("Current State is " + String(this->robotState));
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
        (this)->runStepperHomeState();
        break;
      }
    case (StepperManual):
      {
        //(this)->runStepperManualState();
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
      //    case (NeckCalibration):
      //      {
      //        runNeckCalibrationState();
      //        break;
      //      }
      //    case (MoveToCalibrationState):
      //      {
      //        runMoveToCalibrationState();
      //        break;
      //      }
      //    case (Neck):
      //      {
      //        runNeckState();
      //        break;
      //      }
      //    case (SpinnyBoi):
      //      {
      //        runSpinnyBoiState();
      //        break;
      //      }
  }

}
