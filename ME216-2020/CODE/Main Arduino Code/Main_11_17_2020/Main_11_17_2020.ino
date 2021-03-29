
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


//MsTimer2::set(servoControl, 10); // 10ms period
//MsTimer2::start();

  int xLPin=4;
  int yLPin=5;
  int xRPin=6;
  int yRPin=3;

  XservoL.attach(xLPin);
  YservoL.attach(yLPin);
  XservoR.attach(xRPin);
  YservoR.attach(yRPin);
  
  pinMode(yLPin,OUTPUT);
  pinMode(xLPin,OUTPUT);
  pinMode(yRPin,OUTPUT);
  pinMode(xRPin,OUTPUT);


  
  Serial.print(F("\r\nXBOX USB Library Started"));

  pinMode(3, OUTPUT);
  pinMode(2, OUTPUT);
  pinMode(4, OUTPUT);
  pinMode(5, OUTPUT);
  pinMode(7, INPUT);
  pinMode(8, INPUT);
  pinMode(11, INPUT);
  digitalWrite(2, LOW);
  digitalWrite(4, LOW);

    stepperX.setMaxSpeed(200.0);
    stepperX.setAcceleration(100.0);
   
    stepperY.setMaxSpeed(300.0);
    stepperY.setAcceleration(100.0);
    
    stepperZ.setMaxSpeed(300.0);
    stepperZ.setAcceleration(100.0);

}

void loop() {
  Usb.Task();
  
 // Serial.println(state);
 //getButtons();
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
        Serial.println("Manual");
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
  }
  if (state != MANUAL)
  {
    delay(1);
  }
}
