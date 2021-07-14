/*
 * This is the main.ino file for the Arduino. 
 * This file will instantiate an object of class Robot, that subsequently contains
 * the Eye, Shoulder and Neck subsystem classes. Communication with the arduino is
 * accomplished through a serial channel-> Serial1 that receives messages from a Windows API.
 */
 
#include "Robot.h"
#include "Function.h"

/*
 * Instantiating the Robot class object.
 */
Robot eyeTrackerRobot;

/*
 * Setup() establishes connection with the XBOX controller, Windows API serial comm line, 
 * Debugging serial comm line, and IMU if available.
 */
void setup() {  
  // Debugging Serial
  Serial.begin(115200); //max value
  
  // Windows API Serial
  Serial1.begin(38400); // should test 230400

  // checking for IMU connection
  while (!Serial) {
    ;
  }
  Serial.println("Serial On");
  if(!bno.begin())
  {
   Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
  }
  bno.setExtCrystalUse(true);   // prepping the IMU

  // Checking for XBOX connection
  while (!Serial); // Wait for serial port to connect - used on Leonardo, Teensy and other boards with built-in USB CDC serial connection
  if (Usb.Init() == -1) {
    Serial.print(F("\r\nOSC did not start"));
    while (1); //halt
  }
  Serial.print(F("\r\nXBOX USB Library Started"));
}

/*
 * Loop() function will constantly call RunState function from the Robot class.
 * This function performs all the decision making for the State Machine. 
 */
void loop() {
  eyeTrackerRobot.RunState();
}
