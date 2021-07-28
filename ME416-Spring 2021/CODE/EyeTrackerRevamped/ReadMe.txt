EYESUBSYSTEM.h : 
The Eye Subsystem header file includes all the enumeration, variables, and the Eye Subsystem class. Details of each can be found below.
•	Enumerations: 
		EyeMotor: Symbolic names for denoting which motor is being individually controlled. Used in eye calibration. 

•	Global Variables: 
		float microSecondsPerDegree: number of microseconds required to move the eye servo motors 1 degree on the screen. 
		float lXmicroSecondsPerDegree: same as microSecondsPerDegree, but pertaining to the left-X servo motor.
		float lZmicroSecondsPerDegree: same as microSecondsPerDegree, but pertaining to the left-Z servo motor. 

•	Eyes Class: Class encapsulates the entire eye subsystem of the robot, with objects for the 4 servo motors and functionalities to calibrate the eyes and parallax the eyes on the screen. 

	Private Members: 
		xServoL, zServoL, xServoR, zServoR (Servo): Servo objects for the LeftX, LeftZ, RightX and RightZ servo motors. 
		lXCenter, lZCenter, rXCenter, rZCenter (long int): Center positions in microseconds for each of the servo motors. 

	Private Member Functions:
		void parallax(Matrix(4) leftDotPos, Matrix(4) rightDotPos): Commands the left and right X,Z servo motors to the desired position on the screen indicated by parameters. 
			Parameters: leftDotPos indicates the desired gaze position in the left eye’s frame of reference. rightDotPos indicates the desired gaze position in the right eye’s frame of reference.

	Public Member Functions:
		void init(): Initializes eye class object. Must be called anytime the class object is created. Attaches servo objects to corresponding pins. Defaults value for center positions. Writes servos to corresponding center positions.

		void CalibrateEyes(char eyeCalCommand): The aim of the calibration function is to make the 2 eyes point to a single point at the center of the screen. This is done by incrementing/decrementing the center positions of each servo motor by 1 microsecond. 
			Parameters: eyeCalCommand will either pass in ‘u’ for increment, ‘d’ for decrement and corresponding EyeMotor enumeration for changing calibration motor.

		void ParallaxEyesToPos(Matrix(4) screenDotPos): Creates the gaze position coordinates in the left and right eye’s frame of reference using the parameter and kinematic chain (refer KinematicChain.h).
			Parameters: screenDotPos matrix denotes the coordinates of the desired gaze position in the screen’s frame of reference.
			References KinematicChain class: GetSL() and GetSR().

		void WriteEyeCalibrationVariablesToProm(): Writes the center positions of each servo motor to the predetermined address in the EEPROM (refer PromAddress enumeration in Function.h).

		void ReadEyeCalibrationVariablesFromProm(): Reads the center positions of each servo motor from the predetermined address in the EEPROM and stores it in the center position members in the class object.

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
		neckServo (Servo): Servo object for the neck that induces the yaw rotation.
		neckServoCenter (long int): center position in microseconds for the neck servo object.
		lastStepperPosFront, lastStepperPosBackRight, lastStepperPosBackLeft (float): The position in steps for all three steppers denoting the angle the neck was commanded to last. 
		lSpring (float): the length of the spring in the middle of the neck. 

	Public Member Functions: 
		~RobotNeck(): Destructor function for the neck class object that deletes the pointer objects in the class.

		void init(): Initializes the three-neck stepper object, yaw servo object, sets maximum speed and acceleration for the stepper motors, and defaults the last stepper position and lSpring values. 

		void CalibrateNeck(char neckCalCommand): The aim of the calibration process is to level the neck so that it is parallel to the base of the neck. and 0.127m away from the base of the neck. This is done by incrementing/decrementing the current position of the steppers by 50 steps, individually to a point where it is level. 
			Parameters: neckCalCommand will either pass ‘u’ for incrementing, ‘d’ for decrementing, corresponding enumeration for NeckMotor to change the calibration motor or ‘z’ to save a hand-cranked position as the level position. 

		void MoveNeckManually(float PhiS, float PhiR, float Yaw): Commands the neck steppers to the desired angle given by the parameters. Saves the current positions of the steppers as the last steppers positions for corresponding stepper. Note: This function only sets the desired target location for the steppers. This function call must be followed by RunSteppers() function call.
			Parameters: PhiS – gives the direction of tilt. PhiR – gives the amplitude of tilt. Yaw – gives the desired angle of yaw. 
			References Kinematic chain class: UpdateNeckTransformationMatrix() and GetgB1P1, GetgB2P2, GetgB3P3.

		void RunSteppers(): Runs the steppers to the desired target location set prior to this function call.

		void WriteNeckPositionToProm(): Writes the last stepper position variables for each stepper object to the predetermined address in the EEPROM (refer PromAddress enumeration in Function.h).

		void ReadNeckPositionFromProm(): Reads the last stepper position variables from the EEPROM and sets the current positions for all stepper objects as their corresponding last stepper position variable. 
SHOULDER.h

•	Shoulder Class: Class encapsulates the stepper objects that makes up the shoulder of the robot, along with the functionalities necessary to home and move the shoulder to the desired position.

	Private Members: 
		xStepper, yStepper, zStepper (AccelStepper *): AccelStepper pointer objects for the X,Y and Z shoulder steppers. 
		stepsPerM (float): The number of steps required to move the steppers 1 meter. This value was experimentally determined and maybe subject to change depending upon the stepper motor being used. 

	Private Member Functions: 

		bool zeroStepperX(), zeroStepperY(), zeroStepperZ(): Homes the X,Y or Z axis stepper till the limit switch on the corresponding axis is tripped. Sets that position as the zero position of the corresponding axis stepper. 
			Returns true when the limit switch is tripped. 

		void updateShoulderPositions(): Updates the current positions of the shoulder steppers in the kinematic chain class. 
			References SetStepperPosition in Kinematic Chain class.

	Public Member Functions: 
		~Shoulder(): Destructor for the shoulder class object that deletes the AccelStepper pointer objects. 

		void init(): Initialization function for the shoulder class object that initializes the AccelStepper objects, sets maximum speed and acceleration for the individual steppers and defaults the value for stepsPerM. 

		bool HomeShoulder(): Homes the shoulder axis steppers to set their zero positions. Once all steppers have finished homing, it calls the updateShoulderPositions function to update kinematic chain.
			Returns true when all steppers have finished homing.

		void MoveShoulderToPosition(float x, float y, float z): Moves the shoulder steppers to the desired X,Y,Z coordinates denoted by the parameters. Calls updateShoulderPosition() once the shoulders have moved to their final position.
			Parameters: the parameters denoting the desired position must be in meters with the home position as zero. 

		float GetShoulderPosition(char desiredStepper): Returns the current shoulder position of the stepper denoted by the character parameter.
			Parameters: desiredStepper must be 'x' for X, 'y' for Y and 'z' for Z axis stepper.

		void WriteShoulderPositionToProm(): Writes the current shoulder positions to the predetermined address in the EEPROM (refer PromAddress in Function.h for more details).

		void ReadShoulderPositionFromProm(): Reads the shoulder positions from the predetermined address in the EEPROM and sets the current positions of the steppers to the extracted values.


ROBOT.h

•	Robot Class: Class encapsulates the entire eye tracker robot, including the eye, neck and shoulder objects, along with members indicating the current state of the robot, and public member functions to
		run the different states of the robot and interface with the API. 

	Private Members: 
		robotState (StateMachineState): Enumeration object that stores the current state of the robot (refer StateMachineState enumeration in Function.h).
		robotEyes (Eyes object): Eyes class object encapsulating the eye subsystem.
		robotNeck (NeckClass object): NeckClass object that encapsulates the neck subsystem.
		robotShoulder (Shoulder object): Shoulder object that encapsulates the shoulder subsystem.
		screenDotPos (Matrix(4)): Matrix that denotes the coordinates of the desired gaze position for the eyes.

	Private Member Functions:

		char getSerialCommand(): Obtains a single character command from the Hardware Serial or the connection to the API by performing Serial.read().
			Returns the passed in character from the serial port. 

		void updateKinematicChain(): Updates the kinematic chain instance. References the UpdateKinematicChain() function in KinematicChain.h that recalculates the entire kinematic chain going
			from the eyes to the screen.

		void runMenuModeState(): Runs the MenuMode state, that essentially waits for a user input from the Serial terminal to change the state to a desired one. 
			References getSerialCommand(). If the command is m, then it prints the current state. If the command is s-(int), it changes the state to the enumerated integer passed in.

		void runServoCalibrationState(): References getSerialCommand() and passes the obtained character into the CalibrateEyes(char) function in EyeSubsystem.h.

		void runServoManualState(): References getSerialCommand() and if the obtained character is g, it parses for a (float,float), loads these values into screenDotPos, and then passes these coordinates into ParallaxEyesToPos(screenDotPosition) function in EyeSubsystem.h.

		void runShoulderHomeState(): References the HomeShoulder() function in the Shoulder class. If a 'b' is encountered during the homing process, the robot is set back to MenuMode.

		void runShoulderManualState(): References getSerialCommand() and if the obtained character is g, it parses for a (float,float,float) and passes these values into the MoveShoulderToPosition(x,y,z) function in Shoulder.h.

		void runNeckCalibrationState(): References getSerialCommand() and passes the obtained character into the CalibrateNeck(char) function in NeckClass.h.

		void runNeckManualState(): References getSerialCommand() and if the obtained character is g, it parses for a (float,float,float) and passes these rotation values into the MoveNeckManually(PhiS,PhiR,Yaw) function in Neck.h.
			References RunSteppers() function in NeckClass.h to move the steppers to the set target position.

	Public Member Functions:

		void init(): Initialization function for the robot class object. Initializes the eyes, neck and shoulder objects encapsulated. Calls the read-from-EEPROM function in each of the class objects. Calls the updateKinematicChain() function
			to load all of the variable updates into the kinematic chain instance. 

		void SetState(int stateNumber): Sets the state of the robot to the enumerated value of the stateNumber integer. 

		int GetState(): Gets the current state of the robot and returns the integer value of the enumeration.

		void RunState(): Determines the state of the robot through a switch-case and calls the private member function corresponding to that state of the robot. 

KINEMATICCHAIN.h

•	Kinematic Chain Class: Class that encapsulates the information about the location of origin in the different frames of reference of the robot. The frames of reference of the robot are as follows:

      S - will be the screen frame of reference with the zero coordinate at the center of the screen,
      X will be to the right, Z up, and Y into the screen
      B - will be the origin frame of reference with the zero coordinate at the position where the
      stages of the 3-axis shoulder steppers hit the limit switches or zero position
      C - will be the origin frame of reference located at the center of the Z-axis stage,
      that the neck and eye mechanisms are mounted to.
      Y - will be the frame of reference located at the center of the yaw rotation servo.
      N - will be the frame of reference located at the base of the neck of the robot.
      T - will be the frame of reference located at the center of the top of the neck.
      D - will be the frame of reference at the center of the eye mechanisms,
      with Z up, X to the right and Y along the axis of the straight 90-degree eyes
      L - will be the left eye's frame of reference with the origin at the center of rotation
      and is aligned with the D frame when the 90-degree angle is commanded
      R - will be the right eye's frame of reference with the origin at the center of rotation
      and is aligned with D frame when the 90-degree angle is commanded

      We also need to define additional frames of reference for the neck.
      O - signifies the origin point of the neck at the middle of the base.
      B1 - front point at the base; B2 - back right point at the base; B3 - back left point at the base;
      P1 - front point at the top; P2 - back right point; P3 - back left point; PC - center point at the top
	
	Private Members:
		instance (Kinematic Chain*)

	Protected Members:
		gBS, gBC, gCY, gYN, gNT, gTD, gDL, gDR, gSL, gSR (Matrix(4,4))
		gO_B1, gO_B2, gO_B3, gPC_P1, gPC_P2, gPC_P3 (Matrix(4,4)): These constitute the frame-to-frame transformation matrices of pairs of reference frames from above. 
	
	Private Member functions:
		Kinematic Chain(): Constructor of the instance of the class. This is a private member function because it should only be instantiated once throughout
					the execution of the code. It also populates the transformation matrices with default values.
		Matrix(3,3) eye(): Creates a 3x3 identity matrix

		Matrix(3,3) hat(): Function to 

		float norm(Matrix(3) v)

		Matrix(3,3) rodrigues(Matrix(3) xI)

		Matrix(4,4) expmXI(Matrix(6) xI)

		Matrix(3,3) OuterProduct(Matrix(3) w);

		Matrix(3) CrossProduct(Matrix(3) w, Matrix(3) v)

		Matrix(4,4) xForm(float alpha, float beta, float gamma, float x, float y, float z):

		