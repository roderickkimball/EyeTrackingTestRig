/*
 Example sketch for the Xbox ONE USB library - by guruthree, based on work by
 Kristian Lauszus.
 */

#include <XBOXONE.h>
#include <Servo.h>


// Satisfy the IDE, which needs to see the include statment in the ino too.
#ifdef dobogusinclude
#include <spi4teensy3.h>
#endif
#include <SPI.h>

Servo XservoL;
Servo YservoL;
Servo XservoR;
Servo YservoR;

USB Usb;
XBOXONE Xbox(&Usb);

void setup() {
  int xPinL=3;
  int yPinL=5;
  int xPinR=6;
  int yPinR=7;

  XservoL.attach(xPinL);
  YservoL.attach(yPinL);
  XservoR.attach(xPinR);
  YservoR.attach(yPinR);
  
  pinMode(yPinL,OUTPUT);
  pinMode(xPinL,OUTPUT);
  pinMode(yPinR,OUTPUT);
  pinMode(xPinR,OUTPUT);
  Serial.begin(115200);
  while (!Serial); // Wait for serial port to connect - used on Leonardo, Teensy and other boards with built-in USB CDC serial connection
  if (Usb.Init() == -1) {
    Serial.print(F("\r\nOSC did not start"));
    while (1); //halt
  }
  Serial.print(F("\r\nXBOX USB Library Started"));
}

double xStick=0;
double yStick=0;
int xVal=90;
int yVal=90;
int yVal2=90;
int lim=800;

int stepVal=90;
int stepValY=90;

void loop() {
  Usb.Task();

  xStick=Xbox.getAnalogHat(LeftHatX);
  yStick=Xbox.getAnalogHat(LeftHatY);
  Serial.print(xStick);
  Serial.print("\t");
  Serial.println(yStick);

 // xVal=map(xStick,-32768,32767,0,180);
 // yVal=map(yStick,-32768,32767,0,180);
  
  
if (Xbox.XboxOneConnected) {

    
    if (Xbox.getAnalogHat(LeftHatX) > lim || Xbox.getAnalogHat(LeftHatX) < -lim || Xbox.getAnalogHat(LeftHatY) > lim || Xbox.getAnalogHat(LeftHatY) < -lim || Xbox.getAnalogHat(RightHatX) > lim || Xbox.getAnalogHat(RightHatX) < -lim || Xbox.getAnalogHat(RightHatY) > lim || Xbox.getAnalogHat(RightHatY) < -lim) 
    {
delayMicroseconds(100);
      // Left or Right on Left Joystick
      if (Xbox.getAnalogHat(LeftHatX) > lim || Xbox.getAnalogHat(LeftHatX) < -lim) 
        {
        if (Xbox.getAnalogHat(LeftHatX) < -lim)
        {
        xVal=map(xStick,lim,32767,90,110);
        Serial.println(xVal);
        XservoL.write(xVal);
        XservoR.write(xVal);
        }
        if (Xbox.getAnalogHat(LeftHatX) > lim)
        {
        xVal=map(xStick,-32767,-lim,60,90);
        Serial.println(xVal);
        XservoL.write(xVal);
        XservoR.write(xVal);
        }
      }

      // Up or Down on Left Joystick
      if (Xbox.getAnalogHat(LeftHatY) > lim || Xbox.getAnalogHat(LeftHatY) < -lim) 
      {
        if (Xbox.getAnalogHat(LeftHatY) < -lim)
        {
        yVal=map(yStick,lim,32767,90,110);
        yVal2=map(yStick,-32767,-lim,110,90);
        Serial.println(yVal);
        YservoL.write(yVal);
        YservoR.write(yVal2);
        }
        if (Xbox.getAnalogHat(LeftHatY) > lim)
        {
        yVal=map(yStick,-32767,-lim,60,90);
        yVal2=map(yStick,lim,32767,90,60);
        Serial.println(yVal);
        YservoL.write(yVal);
        YservoR.write(yVal2);
        }
      }

      // Left or Right on Right Joystick
      if (Xbox.getAnalogHat(RightHatX) > lim || Xbox.getAnalogHat(RightHatX) < -lim) 
      {
       
      }

      // Up or Down on Right Joystick
      if (Xbox.getAnalogHat(RightHatY) > lim || Xbox.getAnalogHat(RightHatY) < -lim) 
      {
       
      }

      // 
      if (Xbox.getAnalogHat(LeftHatX) < lim && Xbox.getAnalogHat(LeftHatX) > -lim)
      {
        XservoL.write(stepVal);
      }
    
      Serial.println();
    }


    if (Xbox.getButtonPress(L2) > 0 || Xbox.getButtonPress(R2) > 0) 
    {
      if (Xbox.getButtonPress(L2) > 0) {
        Serial.print(F("L2: "));
        Serial.print(Xbox.getButtonPress(L2));
        Serial.print("\t");
      }
      if (Xbox.getButtonPress(R2) > 0) {
        Serial.print(F("R2: "));
        Serial.print(Xbox.getButtonPress(R2));
        Serial.print("\t");
      }
      Serial.println();
    }

    // Set rumble effect
    static uint16_t oldL2Value, oldR2Value;
    if (Xbox.getButtonPress(L2) != oldL2Value || Xbox.getButtonPress(R2) != oldR2Value) {
      oldL2Value = Xbox.getButtonPress(L2);
      oldR2Value = Xbox.getButtonPress(R2);
      uint8_t leftRumble = map(oldL2Value, 0, 1023, 0, 255); // Map the trigger values into a byte
      uint8_t rightRumble = map(oldR2Value, 0, 1023, 0, 255);
      if (leftRumble > 0 || rightRumble > 0)
        Xbox.setRumbleOn(leftRumble, rightRumble, leftRumble, rightRumble);
      else
        Xbox.setRumbleOff();
    }

    if (Xbox.getButtonClick(UP))
    {
      stepValY+=5;
      YservoL.write(stepValY);
      Serial.println(stepValY);
    }
    if (Xbox.getButtonClick(DOWN))
    {
      stepValY-=5;
      YservoL.write(stepValY);
      Serial.println(stepValY);
    }
    if (Xbox.getButtonClick(LEFT))
    {
      stepVal-=5;
      Serial.println(F("Left"));
      XservoL.write(stepVal);
      Serial.println(stepVal);
    }
    if (Xbox.getButtonClick(RIGHT))
    {
       stepVal+=5;
      Serial.println(F("Right"));
      XservoL.write(stepVal);
      Serial.println(stepVal);
    }

    if (Xbox.getButtonClick(START))
      Serial.println(F("Start"));
    if (Xbox.getButtonClick(BACK))
      Serial.println(F("Back"));
    if (Xbox.getButtonClick(XBOX))
      Serial.println(F("Xbox"));
    if (Xbox.getButtonClick(SYNC))
      Serial.println(F("Sync"));

    if (Xbox.getButtonClick(L1))
      Serial.println(F("L1"));
    if (Xbox.getButtonClick(R1))
      Serial.println(F("R1"));
    if (Xbox.getButtonClick(L2))
      Serial.println(F("L2"));
    if (Xbox.getButtonClick(R2))
      Serial.println(F("R2"));
    if (Xbox.getButtonClick(L3))
      Serial.println(F("L3"));
    if (Xbox.getButtonClick(R3))
      Serial.println(F("R3"));


    if (Xbox.getButtonClick(A))
      Serial.println(F("A"));
    if (Xbox.getButtonClick(B))
      Serial.println(F("B"));
    if (Xbox.getButtonClick(X))
      Serial.println(F("X"));
    if (Xbox.getButtonClick(Y))
      Serial.println(F("Y"));
  }
  delay(1);
}
