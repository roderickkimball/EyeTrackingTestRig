import net.java.games.input.*;
import org.gamecontrolplus.*;
import org.gamecontrolplus.gui.*;

import cc.arduino.*;
import org.firmata.*;

import processing.serial.*;

ControlDevice cont;
ControlIO control;

Arduino arduino;

int A;
int B;
int X;
int Y;
int LB;
int RB;
int Select;
int Start;

float leftXangle;
float leftYangle;
float rightXangle;
float rightYangle;

float screenDistance = 65;
float LxScreen = 0;
float LyScreen = 0;

float RxScreen = 0;
float RyScreen = 0;

float xEye = 1920/2;
float eyeSpace = 6.5;
float yEye = 1080/2;

float dir=0;
float del=0;



float Trigger;

boolean lockEyes = true;

int buttonRelease = 1;


void setup() {
  size(360, 200);
  
  control = ControlIO.getInstance(this);
  cont = control.getMatchedDevice("XBox Controller");
  
  if (cont == null) {
    println("nothing");
    System.exit(-1);
  }
  
  //println(Arduino.list());
  
  arduino = new Arduino(this, Arduino.list()[0], 57600);
  arduino.pinMode(10, Arduino.SERVO);
  arduino.pinMode(11, Arduino.SERVO);
  arduino.pinMode(12, Arduino.SERVO);
  arduino.pinMode(13, Arduino.SERVO);
  arduino.pinMode(3, Arduino.OUTPUT);
}

public void getUserInput(){
  
 
  A = (int)cont.getButton("A").getValue();
  B = (int)cont.getButton("B").getValue();
  X = (int)cont.getButton("X").getValue();
  Y = (int)cont.getButton("Y").getValue();
  LB = (int)cont.getButton("LB").getValue();
  RB = (int)cont.getButton("RB").getValue();
  Select = (int)cont.getButton("Select").getValue();
  Start = (int)cont.getButton("Start").getValue();
  
  //LxScreen = map(cont.getSlider("Lx").getValue(),-1,1,0,1920);
  //LyScreen = map(cont.getSlider("Ly").getValue(),-1,1,0,1080);
  //RxScreen = map(cont.getSlider("Rx").getValue(),-1,1,0,1920);
  //RyScreen = map(cont.getSlider("Ry").getValue(),-1,1,0,1080);
  //Trigger = map(cont.getSlider("Trigger").getValue(),-1,1,0,180);
  LxScreen = map(cont.getSlider("Lx").getValue(),-1,1,-1000,1000);
  
}

void draw(){
  
  getUserInput();
  background(0,0,0);
  isButtonPressed();
  
  dir=LxScreen;
  
  
  if(dir<-100) // Move left
  {
    del=(1120-(-1*LxScreen))/2;
    arduino.digitalWrite(2,Arduino.LOW);
    arduino.digitalWrite(3,Arduino.HIGH);
    delay((int)del/1000);
    arduino.digitalWrite(3,Arduino.LOW);
    delay((int)del/1000);
  }
  
  if(dir>100) // Move left
  {
    del=(1120-(LxScreen))/2;
    arduino.digitalWrite(2,Arduino.HIGH);
    arduino.digitalWrite(3,Arduino.HIGH);
    delay((int)del/1000);
    arduino.digitalWrite(3,Arduino.LOW);
    delay((int)del/1000);
  }
  
  if (lockEyes)
  {
    RxScreen = LxScreen;
    RyScreen = LyScreen;
  }
  
  
  
  //leftXangle = atan((LxScreen-xEye+(eyeSpace/2))/screenDistance);
  //rightXangle = atan((RxScreen-xEye-(eyeSpace/2))/screenDistance);
  
  //leftYangle = atan((LyScreen-yEye)/screenDistance);
  //rightYangle = atan((RyScreen-yEye)/screenDistance);
  
  
  //arduino.servoWrite(10, (int)leftXangle);
  //arduino.servoWrite(11, (int)leftYangle);
  
  //arduino.servoWrite(12, (int)rightXangle);
  //arduino.servoWrite(13, (int)rightYangle);

  //arduino.digitalWrite(3, A);
  //arduino.digitalWrite(4, B);
  //arduino.digitalWrite(5, X);
  //arduino.digitalWrite(6, Y);
  //arduino.digitalWrite(7, (int)Trigger);

  
}

boolean isButtonPressed(){
  
  if ((A != 0) && (buttonRelease == 1))
  {
    lockEyes = !lockEyes;
    buttonRelease = 0;
  }
  else
  {
    buttonRelease = 1;
  }
  return lockEyes;
}
