// MultiStepper.pde
// -*- mode: C++ -*-
//
// Shows how to multiple simultaneous steppers
// Runs one stepper forwards and backwards, accelerating and decelerating
// at the limits. Runs other steppers at the same time
//
// Copyright (C) 2009 Mike McCauley
// $Id: MultiStepper.pde,v 1.1 2011/01/05 01:51:01 mikem Exp mikem $

#include <AccelStepper.h>
#include <XBOXONE.h>
#include <Servo.h>
#ifdef dobogusinclude
#include <spi4teensy3.h>
#endif
#include <SPI.h>

Servo Xservo;
Servo Yservo;

USB Usb;
XBOXONE Xbox(&Usb);

// Define some steppers and the pins the will use
//AccelStepper stepper1; // Defaults to AccelStepper::FULL4WIRE (4 pins) on 2, 3, 4, 5
AccelStepper stepper1(AccelStepper::DRIVER, 3, 2);
AccelStepper stepper2(AccelStepper::DRIVER, 5, 4);



void setup()
{  
    stepper1.setMaxSpeed(10000.0);
    stepper1.setAcceleration(5000.0);
    stepper1.moveTo(-500);
    
    stepper2.setMaxSpeed(10000.0);
    stepper2.setAcceleration(5000.0);
    stepper2.moveTo(-500);
    
//    stepper3.setMaxSpeed(3000.0);
//    stepper3.setAcceleration(400.0);
//    stepper3.moveTo(-300); 
Serial.begin(2000000);
while (!Serial); // Wait for serial port to connect - used on Leonardo, Teensy and other boards with built-in USB CDC serial connection
  if (Usb.Init() == -1) {
    Serial.print(F("\r\nOSC did not start"));
    while (1); //halt
  }
  Serial.print(F("\r\nXBOX USB Library Started"));
}
int target=500;
int target2=400;
int Target=0;
int Target2=0;


int del=0;
int joystick=0;
int joystick2=0;
int xStick=0;
int yStick=0;

int steps=0;
float LxScreen=0;
float LyScreen=0;
float dir=0;
float dir2=0;
bool limit=LOW;
bool Home=HIGH;
int i=0;

void loop()
{
 Usb.Task();
 xStick=Xbox.getAnalogHat(LeftHatX);
 yStick=Xbox.getAnalogHat(LeftHatY);
 
 
 joystick=yStick;
 joystick2=xStick;
 target = map(joystick,-32767,32767,-1000,1000);
 target2 = map(joystick2,-32767,32767,-1000,1000);

 if(target>100||target<-100)
 {
  Target+=target;
    stepper1.moveTo(Target);
 }
 if(target2>100 || target2<-100)   
 {
  Target2+=target2;
    stepper2.moveTo(Target2);
 }
    stepper1.run();
    stepper2.run();
    Serial.print(target);
    Serial.print("\t\t");
    Serial.println(target2);
}
