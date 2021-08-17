State Machine Summary:

•	Menu Mode - Idle state of the robot. Waits for commands from the API or serial communication to switch states or return the current state.
•	Robot Run - Runs the robot by moving the desired subsystem to the commanded position. The commanded position is decoded from the g-code that is sent from the API. 
		ge(float, float) - moves eyes; gs(float, float, float) - moves shoulder; gn(float, float, float) - moves neck
•	Eye Calibrate - State to calibrate the eye servos. Calibration of the eye servos is important for all 4 eye servos to know what the 90-degree straight forward position of the eyes is. All commanded angles are incremented/decremented from this 90-degree position.
•	Neck Calibrate - State to calibrate the neck steppers. Calibration of the neck steppers is important to save the position of the steppers at which the neck platform is completely level with the base plate.
•	Shoulder Calibrate - State to calibrate the shoulder steppers. Calibration process includes the shoulder steppers hitting their "zero" or "home" position limit switches. 

Class Structure Summary:

ROBOT.h

•	Robot Class: Class encapsulates the entire eye tracker robot, including the eye, neck and shoulder objects, along with members indicating the current state of the robot, transformation matrices that go from the center 
		of the left and right eye to the screen, and public member functions to run the different states of the robot and interface with the API. 

	Private Members: 
		robotState (StateMachineState): Enumeration object that stores the current state of the robot (refer StateMachineState enumeration in Function.h).
		robotEyes (Eyes object): Eyes class object encapsulating the eye subsystem.
		robotNeck (NeckClass object): NeckClass object that encapsulates the neck subsystem.
		robotShoulder (Shoulder object): Shoulder object that encapsulates the shoulder subsystem.
		screenDotPos (Matrix(4)): Matrix that denotes the coordinates of the desired gaze position for the eyes.
	
	Protected Members:
		gLS, gRS (Matrix(4,4)): Matrices that denote the transformation matrix that goes from the center of left and right eyes respectively to the center of the screen.


	Private Member Functions:

		char getSerialCommand(): Obtains a single character command from the Hardware Serial or the connection to the API by performing Serial.read().
			Returns the passed in character from the serial port. 

		void updateKinematicChain(): Recalculates gLS and gRS by pulling the inverse transformation matrices from the shoulder, neck and eyes. Refer GetInverse****Transformation() in Eyes.h, RobotNeck.h and Shoulder.h.

		void setRobotPosition(char command): Sets the desired subsystem to the commanded position. This function is called immediately after the g code is sent from the API or serial communication, and is responsible for decoding the rest of the 
			g command to determine which subsystem is to be moved. 
	
		void getRobotPosition(char command): Gets the current position of the desired subsystem. This function is called immediately after the p command is sent from the API or serial communication, and is responsible for decoding the rest of the
			p command to determine which subsystem's position is to be printed.

		void runMenuModeState(): Runs the MenuMode state, that essentially waits for a user input from the Serial terminal to change the state to a desired one. 
			References getSerialCommand(). If the command is m, then it prints the current state. If the command is s-(int), it changes the state to the enumerated integer passed in.

		void runRobotRunState(): Runs the Robot Run state, that essentially reads the commands set in from the serial port or API, decodes the g or p code that is sent is and subsequently calls setRobotPosition(char) or getRobotPosition(char).
			'm' returns the state to MenuMode.

		void runEyeCalibrationState(): References getSerialCommand() and passes in the character obtained into Eyes::CalibrateEyes(char) function. 
			'm' returns the state to MenuMode.

		void runShoulderCalibrationState(): References the Shoulder::HomeShoulder() function that zeroes all three stepper axes. 'm' returns the state to MenuMode.

		void runNeckCalibrationState(): References getSerialCommand() and passes the obtained character into the RobotNeck::CalibrateNeck(char) function. 'm' returns the state to MenuMode.

	Public Member Functions:

		void init(): Initialization function for the robot class object. Initializes the eyes, neck and shoulder objects encapsulated. Calls the read-from-EEPROM function in each of the class objects. Calls the updateKinematicChain() function
			to load all of the variable updates into the kinematic chain instance. 

		void SetState(int stateNumber): Sets the state of the robot to the enumerated value of the stateNumber integer. 

		int GetState(): Gets the current state of the robot and returns the integer value of the enumeration.

		void RunState(): Determines the state of the robot through a switch-case and calls the private member function corresponding to that state of the robot. 

EYESUBSYSTEM.h : 
The Eye Subsystem header file includes all the enumeration, variables, and the Eye Subsystem class. Details of each can be found below.
•	Enumerations: 
		EyeMotor: Symbolic names for denoting which motor is being individually controlled and calibrated. Used in eye calibration. 

•	Eyes Class: Class encapsulates the entire eye subsystem of the robot, with objects for the 4 servo motors and functionalities to calibrate the eyes and parallax the eyes on the screen. 

	Private Members: 
		xServoL, zServoL, xServoR, zServoR (Servo): Servo objects for the LeftX, LeftZ, RightX and RightZ servo motors. 
		lXCenter, lZCenter, rXCenter, rZCenter (long int): Center positions in microseconds for each of the servo motors. 
		lXmicroSecondsPerDegree, lZmicroSecondsPerDegree, rXmicroSecondsPerDegree, rZmicroSecondsPerDegree (float): Denotes the number of microseconds required to move the left/right eye servos one degree.
	
	Protected Members:
		    /*
      		T - will be the frame of reference located at the center of the top of the neck.
      		D - will be the frame of reference at the center of the eye mechanisms,
      		with Z up, X to the right and Y along the axis of the straight 90-degree eyes
	      	L - will be the left eye's frame of reference with the origin at the center of rotation
	        and is aligned with the D frame when the 90-degree angle is commanded
	        R - will be the right eye's frame of reference with the origin at the center of rotation
	        and is aligned with D frame when the 90-degree angle is commanded
		    */
		gTD, gDL, gDR (Matrix(4,4): Transformation matrices that go from one frame of reference to another given above.
		gLT, gRT (Matrix(4,4)): Inverse transformation matrices that go from the center of the left and right eyes to the top of the neck.

	Private Member Functions:
		void parallax(Matrix(4) leftDotPos, Matrix(4) rightDotPos): Commands the left and right X,Z servo motors to the desired position on the screen indicated by parameters. 
			Parameters: leftDotPos indicates the desired gaze position in the left eye’s frame of reference. rightDotPos indicates the desired gaze position in the right eye’s frame of reference.

	Public Member Functions:
		void init(): Initializes eye class object. Must be called anytime the class object is created. Attaches servo objects to corresponding pins. Defaults value for center positions. Writes servos to corresponding center positions.
			Provides transformation matrices with starting values.

		void ParallaxEyesToPos(Matrix(4) screenDotPos, Matrix(4,4) gLS, Matrix(4,4) gRS): Creates the gaze position coordinates in the left and right eye’s frame of reference using the dotPosition and inverse transformation matrix parameters.
			Parameters: screenDotPos matrix denotes the coordinates of the desired gaze position in the screen’s frame of reference.
				gLS denotes the TF matrix that goes from center of left eye to the center of screen. gRS is the same but for the right eye.

		void CalibrateEyes(char eyeCalCommand,  Matrix(4,4) gLS, Matrix(4,4) gRS): The aim of the calibration function is to make the 2 eyes point to a single point at the center of the screen. This is done by incrementing/decrementing the center positions of each servo motor by 1 microsecond. 
			Parameters: eyeCalCommand will either pass in ‘u’ for increment, ‘d’ for decrement and corresponding EyeMotor enumeration for changing calibration motor.

		void WriteEyeCalibrationVariablesToProm(): Writes the center positions of each servo motor to the predetermined address in the EEPROM (refer PromAddress enumeration in Function.h).

		void ReadEyeCalibrationVariablesFromProm(): Reads the center positions of each servo motor from the predetermined address in the EEPROM and stores it in the center position members in the class object.

		Matrix(4,4) GetInverseLeftEyeTransformation(), GetInverseRightEyeTransformation(): Gets the inverse transformation matrices for the left and right eyes in the eye subsystem. Referenced by Robot::updateKinematicChain().

NECKCLASS.h : 
The Neck class header file includes all the enumerations, variables and the RobotNeck class. Details of each can be found below:

•	Enumerations: 
		NeckMotor: Symbolic names for denoting which neck stepper motor is being individually controlled. Used in neck calibration.

•	Global Variables: 
		float knowStepperPos: This is used as the default or zero stepper position for all neck motors when the neck is parallel to the base. 
		float spoolRadius: Radius of the spool on stepper shafts.
		float spoolCircumference: Circumference of the spool
		float mmPerRevs: Length of thread released in each stepper revolution.
		float stepsPerRev: Number of steps for neck steppers in one revolution.
		float mmPerStep: Length of thread released in one stepper step.
		float yawMicroSecondsPerDegree: number of microseconds required to rotate the yaw servo one degree.

•	Robot Neck Class: Class encapsulates the stepper and servo objects that make up the entire neck subsystem, along with the functionalities necessary to calibrate and rotate the neck to the desired angle. 

	Private Members: 
		frontStepper, backRightStepper, backLeftStepper (AccelStepper *): Stepper objects made using AccelStepper for the 3 stepper motors of the neck. 
		m3FrontStepper, m3BackRightStepper, m3BackLeftStepper (M3Stepper): Stepper objects made using M3Stepper class for the 3 stepper motors of the neck. Using this class gives greater precise control over the steppers and avoids the steppers from missing steps (was a problem with AccelStepper).
		neckServo (Servo): Servo object for the neck that induces the yaw rotation.
		neckServoCenter (long int): center position in microseconds for the neck servo object.
		lastPhiR, lastPhiS, lastPhiD (float): stores the angles that the neck was last commanded to. 		
		lSpring (float): the length of the spring in the middle of the neck. 

	Protected Members:
		    /*
	      C - will be the origin frame of reference located at the center of the Z-axis stage,
	      that the neck and eye mechanisms are mounted to.
	      Y - will be the frame of reference located at the center of the yaw rotation servo.
	      N - will be the frame of reference located at the base of the neck of the robot.
	      T - will be the frame of reference located at the center of the top of the neck.
		
		These frames of references are used solely to calculate the cable length that is required to move the neck to the commanded angles.
	       O - signifies the origin point of the neck at the middle of the base.
	       B1 - front point at the base; B2 - back right point at the base; B3 - back left point at the base;
	       P1 - front point at the top; P2 - back right point; P3 - back left point; PC - center point at the top
		    */
		gCY, gYN, gNT (Matrix(4,4): Denotes the TF matrices that go from one frame of reference to the other given above.
		gO_B1, gO_B2, gO_B3, gPC_P1, gPC_P2, gPC_P3 (Matrix(4,4)): """""""
		gB1_P1, gB2_P2, gB3_P3 (Matrix(4,4)): """"""""
		gTC (Matrix(4,4)): Denotes the complete inverse transformation matrix that goes from the top of the neck to center of z-axis stepper stage.
	
	Private Member Functions:
		void updateNeckTransformation(): Takes in the angles that the neck was commanded to and performs the computation of the cable length required from all 3 steppers to move the neck to the commanded angle. 

	Public Member Functions: 
		~RobotNeck(): Destructor function for the neck class object that deletes the pointer objects in the class.

		void init(): Initializes the three-neck stepper object, yaw servo object, sets maximum speed and acceleration for the stepper motors, and defaults the last stepper position and lSpring values. 

		void CalibrateNeck(char neckCalCommand): The aim of the calibration process is to level the neck so that it is parallel to the base of the neck. and 0.127m away from the base of the neck. This is done by incrementing/decrementing the current position of the steppers by 50 steps, individually to a point where it is level. 
			Parameters: neckCalCommand will either pass ‘u’ for incrementing, ‘d’ for decrementing, corresponding enumeration for NeckMotor to change the calibration motor or ‘z’ to save a hand-cranked position as the level position. 

		void MoveNeckManually(float PhiS, float PhiR, float Yaw): Commands the neck steppers to the desired angle given by the parameters. Saves the current positions of the steppers as the last steppers positions for corresponding stepper. Note: This function only sets the desired target location for the steppers. This function call must be followed by RunSteppers() function call.
			Parameters: PhiS – gives the direction of tilt. PhiR – gives the amplitude of tilt. Yaw – gives the desired angle of yaw. 
		
		void SetLastNeckPosition(): Sets the current positions for the neck steppers upon startup based on the last commanded neck angles.

		void RunSteppers(): Runs the steppers to the desired target location set prior to this function call.

		void WriteNeckPositionToProm(): Writes the last commanded neck angles variables for each stepper object to the predetermined address in the EEPROM (refer PromAddress enumeration in Function.h).

		void ReadNeckPositionFromProm(): Reads the last commanded neck angles variables from the EEPROM and sets the current positions for all stepper objects as their corresponding last stepper position variable. 

		Matrix(4,4) GetInverseNeckTransformation(): Returns the inverse neck transformation matrix that goes from the top of the neck to the center of z-axis stepper stage. Referenced by Robot::updateKinematicChain().
		
SHOULDER.h

•	Shoulder Class: Class encapsulates the stepper objects that makes up the shoulder of the robot, along with the functionalities necessary to home and move the shoulder to the desired position.

	Private Members: 
		xStepper, yStepper, zStepper (AccelStepper *): AccelStepper pointer objects for the X,Y and Z shoulder steppers. 
		stepsPerM (float): The number of steps required to move the steppers 1 meter. This value was experimentally determined and maybe subject to change depending upon the stepper motor being used. 

	Protected Members:
		    /*
	      S - will be the screen frame of reference with the zero coordinate at the center of the screen,
	      X will be to the right, Z up, and Y into the screen
	      B - will be the origin frame of reference with the zero coordinate at the position where the
	      stages of the 3-axis shoulder steppers hit the limit switches or zero position
	      C - will be the origin frame of reference located at the center of the Z-axis stage,
	      that the neck and eye mechanisms are mounted to.
		    */
		gBS, gBC (Matrix(4,4)): Transformation matrices that go from one frame of reference to another given above.
		gCS (Matrix(4,4)): Total inverse transformation matrix that goes from the center of z-axis stepper stage to the center of the screen. 

	Private Member Functions: 

		bool zeroStepperX(), zeroStepperY(), zeroStepperZ(): Homes the X,Y or Z axis stepper till the limit switch on the corresponding axis is tripped. Sets that position as the zero position of the corresponding axis stepper. 
			Returns true when the limit switch is tripped. 

		void updateShoulderPositions(): Updates the current positions of the shoulder steppers in the gBC transformation matrix. 

	Public Member Functions: 
		~Shoulder(): Destructor for the shoulder class object that deletes the AccelStepper pointer objects. 

		void init(): Initialization function for the shoulder class object that initializes the AccelStepper objects, sets maximum speed and acceleration for the individual steppers and defaults the value for stepsPerM. 

		bool HomeShoulder(): Homes the shoulder axis steppers to set their zero positions. Once all steppers have finished homing, it calls the updateShoulderPositions function to update TF matrix.
			Returns true when all steppers have finished homing.

		void MoveShoulderToPosition(float x, float y, float z): Moves the shoulder steppers to the desired X,Y,Z coordinates denoted by the parameters. Calls updateShoulderPosition() once the shoulders have moved to their final position.
			Parameters: the parameters denoting the desired position must be in meters with the home position as zero. 

		float GetShoulderPosition(char desiredStepper): Returns the current shoulder position of the stepper denoted by the character parameter.
			Parameters: desiredStepper must be 'x' for X, 'y' for Y and 'z' for Z axis stepper.

		void WriteShoulderPositionToProm(): Writes the current shoulder positions to the predetermined address in the EEPROM (refer PromAddress in Function.h for more details).

		void ReadShoulderPositionFromProm(): Reads the shoulder positions from the predetermined address in the EEPROM and sets the current positions of the steppers to the extracted values.

		Matrix(4,4) GetInverseShoulderTransformation(): Gets the total inverse transformation matrix for the shoulder subsystem. Referenced by Robot::updateKinematicChain().

M3STEPPER.H

•	M3Stepper Class: Class that encapsulates members of a stepper object, along with functions to move the stepper one step, set stepper speed, current position and target position.

	Private Members:
		_currentPosition (long): Denotes the current position of the stepper object.
		_targetPosition (long): Denotes the target position of the stepper object.
		_direction, _pulse (int): Denotes the direction and pulse pins of the stepper object.
		_speed (float): Denotes the current set speed of the stepper.

	Public Member functions:
		void init(int directionPin, int pulsePin): Initialization function for the stepper object. Sets the pin modes for the direction and pulse pins.
		
		void SetCurrentPosition(long stepPosition): Sets the current position of the stepper to the parameter passed in. 

		long CurrentPosition(): Gets the current position of the stepper.

		void SetSpeed(float desiredSpeed): Sets the desired speed for the stepper.

		float GetSpeed(): Gets the current speed of the stepper.

		bool IsReached(): Checks if the stepper has reached its target position.

		void MoveTo(long target): Sets the target position for the stepper.

		void RunStepper(): Pulses the stepper from low to high, producing one step in the desired direction.

KINEMATICCHAIN.h

•	Kinematic Chain Class: Class that encapsulates all the static functions that are used to create and manipulate the transformation matrices.
	
	Public Member Functions:
		Kinematic Chain(): Constructor of the instance of the class. This is a private member function because it should only be instantiated once throughout
					the execution of the code. The function populates the transformation matrices with default values.

	Static Public Member functions:
		Matrix(3,3) eye(): Creates a 3x3 identity matrix.

		Matrix(3,3) hat(): Function to find the matrix projection of a given 3x1 matrix.

		float norm(Matrix(3) v): Normalizes a given 3x1 matrix. This is done by finding the square root of the inner product of the parameter vector with itself.

		Matrix(3,3) rodrigues(Matrix(3) xI): Function to obtain the projection of an identity matrix about the given axis of rotation passed in as parameter.

		Matrix(4,4) expmXI(Matrix(6) xI): Function to calculate the matrix exponential of the given rotational/translation vector. This function is meant to return the transformation matrix
							given that we provide the rotational and translation values from one reference frame to another.

		Matrix(3,3) OuterProduct(Matrix(3) w): Returns the result of performing outer product.

		Matrix(3) CrossProduct(Matrix(3) w, Matrix(3) v): Returns the result of performing cross product.

		Matrix(4,4) xForm(float alpha, float beta, float gamma, float x, float y, float z): Creates the transformation matrix, given the rotation and translation values.

		
