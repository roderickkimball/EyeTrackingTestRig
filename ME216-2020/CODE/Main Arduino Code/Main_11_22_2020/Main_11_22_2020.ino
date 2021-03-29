#include"functions.h"

void setup() {

//  Serial.begin(115200);
//  while (!Serial); // Wait for serial port to connect - used on Leonardo, Teensy and other boards with built-in USB CDC serial connection
//  if (Usb.Init() == -1) {
//    Serial.print(F("\r\nOSC did not start"));
//    while (1); //halt
//  }
 Serial.begin(115200);
  while (!Serial); // Wait for serial port to connect - used on Leonardo, Teensy and other boards with built-in USB CDC serial connection
  if (Usb.Init() == -1) {
    Serial.print(F("\r\nOSC did not start"));
    while (1); //halt
  }
  Serial.print(F("\r\nXBOX USB Library Started"));


  int xLPin=4;
  int yLPin=5;
  int xRPin=6;
  int yRPin=7;

  XservoL.attach(xLPin);
  YservoL.attach(yLPin);
  XservoR.attach(xRPin);
  YservoR.attach(yRPin);

  stepper.setMaxSpeed(5000); //SPEED = Steps / second  
  stepper.setAcceleration(1000); //ACCELERATION = Steps /(second)^2    
  delay(500);
  stepper2.setMaxSpeed(5000); //SPEED = Steps / second  
  stepper2.setAcceleration(1000); //ACCELERATION = Steps /(second)^2    
  delay(500);  
  stepper3.setMaxSpeed(5000); //SPEED = Steps / second  
  stepper3.setAcceleration(1000); //ACCELERATION = Steps /(second)^2    
  delay(500);  
  
InitialValues(); //averaging the values of the 3 analog pins (values from potmeters)


pinMode(23,INPUT);
pinMode(25,INPUT);
pinMode(27,INPUT);
  //----------------------------------------------------------------------------  
}



void loop() {

  
  Usb.Task();
 // limit=analogRead(A10);
 // Serial.println(limit);
  switch (state)
  {
    case (MENU):
      {
        runMenuState();
        break;
      }
    case (CAL):
      {
        runCalState();
        break;
      }
    case (MANUAL):
      {
        //Serial.println("Manual");
        runManualState();
        break;
      }
    case (AUTO):
      {
        runAutoState();
        break;
      }
    case (HOM):
      {
        runHomeState();
        break;
      }
    case (STEPPERS):
      {
        runSteppersState();
        break;
      }
  }
  if (state != MANUAL)
  {
    delay(1);
  }
}
