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

float Lx;
float Ly;
float Rx;
float Ry;

float Trigger;


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
  
  Lx = map(cont.getSlider("Lx").getValue(),-1,1,0,180);
  Ly = map(cont.getSlider("Ly").getValue(),-1,1,-1,1);
  Rx = map(cont.getSlider("Rx").getValue(),-1,1,-1,1);
  Ry = map(cont.getSlider("Ry").getValue(),-1,1,-1,1);
  Trigger = map(cont.getSlider("Trigger").getValue(),-1,1,0,180);
  
  
}

void draw(){
  
  getUserInput();
  background(0,0,0);
  
  arduino.servoWrite(10, (int)Trigger);

  arduino.digitalWrite(3, A);
  arduino.digitalWrite(4, B);
  arduino.digitalWrite(5, X);
  arduino.digitalWrite(6, Y);
  //arduino.digitalWrite(7, (int)Trigger);

  
}
