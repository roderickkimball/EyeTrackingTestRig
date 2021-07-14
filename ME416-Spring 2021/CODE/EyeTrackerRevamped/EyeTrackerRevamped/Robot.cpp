/*
 * Robot source file containing definitions for all function declarations in
 * header file. 
 */

#include "Robot.h"

/*
 * Robot class constructor.
 */
Robot::Robot()
{
  this->robotState = MenuMode;
  this->robotEyes.init(xLPin, zLPin, xRPin, zRPin);
}

/*
 * Function to set the state for the Robot.
 */
void Robot::SetState(int stateNumber)
{
  this->robotState = static_cast<StateMachineState>(stateNumber);
}

/*
 * Function to get the current state of the Robot.
 */
int Robot::GetState()
{
  return static_cast<int>(this->robotState);
}

/*
 * Function to obtain serial communication signals from Windows API.
 * Based on the predefined code established here, functionality is chosen.
 */
void Robot::runSerialComm()
{
  char command = SerialTerminal->read();
  
  if (command == 'g') { // gazes at input coordinates
    int setx = SerialTerminal->parseInt();
    int setz = SerialTerminal->parseInt();
    //moveEyesTo(setx, setz);
  }
  
  else if (command == 'p') { //outputs current postion
    //SerialTerminal->println(String(screenDotPos(0)) + ',' + String(-1 * screenDotPos(2)));
  }
  
  else if (command == 'm') { //returns current mode (for now should always be menuMode)
    SerialTerminal->println("mode: " + this->GetState());
  }
  
  else if (command == 'c') { //gazes at center screen
    //moveEyesTo(0, 0);
  }
  
  else if (command == 's') { //changes state
    int newstate = SerialTerminal->parseInt();
    this->SetState(newstate);
    SerialTerminal->println("tring to go to state: " + stateNames[newstate]);
  }
  //delay(500);
}

/*
 * Function to obtain calibration commands from API and
 * subsequently pass message onto Eye subsystem class.
 */
void Robot::runServoCalibrationState()
{
  char eyeCalCommand = ' ';
  if (SerialTerminal->available()){
    eyeCalCommand = SerialTerminal->read();
  }

  bool calStatus;
  calStatus = this->robotEyes.CalibrateServos(eyeCalCommand);
  // if calibration has been stopped, return to MenuMode;
  if (!calStatus)
  {
    this->SetState(0);
  }
}

/*
 * Function to determine the state of the robot, and run the
 * corresponding action of the state.
 */
void Robot::RunState()
{
  switch (this->robotState)
  {
    case (MenuMode):
      {
        (this)->runSerialComm();
        break;
      }
    case (ServoCalibration):
      {
        (this)->runServoCalibrationState();
        break;
      }
    case (ServoManual):
      {
        //(this)->runServoManualState();
        break;
      }
      //    case (StepperHome):
      //      {
      //        runStepperHomeState();
      //        break;
      //      }
      //    case (StepperManual):
      //      {
      //        runStepperManualState();
      //        break;
      //      }
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
