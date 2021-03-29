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


void setup() {
  // put your setup code here, to run once:
pinMode(3,OUTPUT);
pinMode(2,OUTPUT);
pinMode(4,OUTPUT);
pinMode(5,OUTPUT);
pinMode(7,INPUT);
pinMode(8,INPUT);
pinMode(11,INPUT);
digitalWrite(2,LOW);
digitalWrite(4,LOW);

Serial.begin(115200);
while (!Serial); // Wait for serial port to connect - used on Leonardo, Teensy and other boards with built-in USB CDC serial connection
  if (Usb.Init() == -1) {
    Serial.print(F("\r\nOSC did not start"));
    while (1); //halt
  }
  Serial.print(F("\r\nXBOX USB Library Started"));

}

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
void loop() {
  // put your main code here, to run repeatedly:
 Usb.Task();
 xStick=Xbox.getAnalogHat(LeftHatX);
 yStick=Xbox.getAnalogHat(LeftHatY);
 
 
 joystick=yStick;
 joystick2=xStick;
 LxScreen = map(joystick,-32767,32767,-1000,1000);
 LyScreen = map(joystick2,-32767,32767,-1000,1000);
 

 limit=digitalRead(7);

 if (Xbox.getButtonClick(A))
 {
 Home=LOW;
 }

 
  dir=LxScreen;
  dir2=LyScreen;
  
  
  if(dir<-100 && limit==HIGH) // Move left
  {
    del=(1100-(-1*(LxScreen-100)))/2;
    digitalWrite(2,LOW);
    digitalWrite(3,HIGH);
    delayMicroseconds((int)del);
    digitalWrite(3,LOW);
    delayMicroseconds((int)del);
    steps++;
  }
 
  
  if(dir>100 && limit==HIGH) // Move Right
  {
    del=(1100-(LxScreen+100))/2;
    digitalWrite(2,HIGH);
    digitalWrite(3,HIGH);
    delayMicroseconds((int)del);
    digitalWrite(3,LOW);
    steps--;
    
    delayMicroseconds((int)del);
  }

  if(dir2<-100 && limit==HIGH) // Move left
  {
    del=(1100-(-1*(LyScreen-100)))/2;
    digitalWrite(4,LOW);
    digitalWrite(5,HIGH);
    delayMicroseconds((int)del);
    digitalWrite(5,LOW);
    delayMicroseconds((int)del);
    //steps--;
  }
  
  if(dir2>100 && limit==HIGH) // Move Right
  {
    del=(1100-(LyScreen+100))/2;
    digitalWrite(4,HIGH);
    digitalWrite(5,HIGH);
    delayMicroseconds((int)del);
    digitalWrite(5,LOW);
    delayMicroseconds((int)del);
    //steps++;
  }   

if (Home==LOW)
{
  Homex();
}

}

int Homex()
{
  limit=digitalRead(7);
  if(limit==HIGH) //Moves slowly towards the limit switch
  {
  digitalWrite(2,HIGH);
  digitalWrite(3,HIGH);
  delayMicroseconds(200);
  digitalWrite(3,LOW);
  delayMicroseconds(200);
  //Serial.println("Homing");
  }
  else
  {
    i=0;
    digitalWrite(2,LOW);
    while(i<1000) // Back up 1000 steps
    {
    //digitalWrite(2,LOW);
    digitalWrite(3,HIGH);
    delayMicroseconds(400);
    digitalWrite(3,LOW);
    delayMicroseconds(400);
    //Serial.println(i);
    i++;
    }

    limit=digitalRead(7);
    while(limit==HIGH) // Move towards limit slowly until limit trips
    {
    digitalWrite(2,HIGH);
    digitalWrite(3,HIGH);
    delayMicroseconds(800);
    digitalWrite(3,LOW);
    delayMicroseconds(800);
    limit=digitalRead(7);
    }
    // Limit has tripped and steps can be zeroed
     steps=0;
     //Serial.print("Step ");
     //Serial.print(steps);
     digitalWrite(2,LOW);
     i=0;
    while(i<500) // Back up 500 steps
    {
    //digitalWrite(2,LOW);
    digitalWrite(3,HIGH);
    delayMicroseconds(800);
    digitalWrite(3,LOW);
    delayMicroseconds(800);
    //Serial.println(i);
    i++;
    steps++;
    Home=HIGH;
    }
  }
  
}

