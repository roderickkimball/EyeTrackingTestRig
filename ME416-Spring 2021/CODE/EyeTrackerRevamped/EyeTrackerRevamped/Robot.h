#ifndef _ROBOT_H
#define _ROBOT_H

#include "EyeSubsystem.h"
#include "Function.h"

class Robot
{
	private:
		StateMachineState robotState;
		Eyes robotEyes;
		//Shoulder robotShoulder;
		//Neck robotNeck;
   
   void runSerialComm();
   void runServoCalibrationState();
   void runServoManualState();

	public:
		Robot();
		void SetState(int stateNumber);
		int GetState();
		void RunState();
};



#endif
