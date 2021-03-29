void servoCalibration(){

   x = (0.5*IPD); //x = distance from left side of screen to eye
   z = (eyeZlocation); //distance between top of screen and eye  //eyeZlocation is the measured distance between chin laser and eye
   alphaLeft = asin(x/deltaY);
   betaLeft = asin(z/deltaY);
   alphaRight = asin(x/deltaY);
   betaRight = asin(z/deltaY);
  
  //Step 1: Point left eye and right eye at the chin laser

  
    i=0;
    while(i==0)
    {
      Usb.Task();
      if(Xbox.getButtonClick(B))
      {
        i=1;
      }
      GetPos();
      leftEye();
    }
leftXhome = XservoL.read() - alphaLeft;  //servo x angle for left eye center
leftZhome = ZservoL.read() - betaLeft;  //servo z angle for left eye center


delay(1000);


    i=0;
    while(i==0)
    {
      Usb.Task();
      if(Xbox.getButtonClick(B))
      {
        i=1;
      }
      GetPos();
      rightEye();
    }
rightXhome = XservoR.read() - alphaRight;  //servo x angle for right eye center
rightZhome = ZservoR.read() - betaRight;  //servo z angle for right eye center 
}

void stepperCalibration()
{
  //Step 1: Home the rig.
  zeroStepper(xpulse,xdir,xEnd);
  stepperX.setCurrentPosition(500);
  zeroStepper(ypulse,ydir,yEnd);
  stepperY.setCurrentPosition(500);
  zeroStepper(zpulse,zdir,zEnd);
  stepperZ.setCurrentPosition(500);
  
  //Step 2: Point chin laser at the bottom left corner of the screen @ 65cm.
  moveStepper_relative(zpulse,zdir,10188,HIGH);
  moveStepper_relative(ypulse,ydir,8335,LOW); // Last input parameter: Steps from home position to 65cm.

Usb.Task();

i=0;
while(i==0)
{
  
  Usb.Task();
  if(Xbox.getButtonClick(B))
  {
    i=1;
  }
  ReadAnalog(); 
  stepperX.run(); 
  stepperZ.run();
}

bottomScreen=stepperZ.currentPosition();
leftScreen=stepperX.currentPosition();
Serial.println(leftScreen);

delay(1000);
Serial.println("Bottom Left Corner");

i=0;
while(i==0)
{
  
  Usb.Task();
  if(Xbox.getButtonClick(B))
  {
    i=1;
  }
  ReadAnalog(); 
  stepperX.run(); 
  stepperZ.run();
}

topScreen=stepperZ.currentPosition();
rightScreen=stepperX.currentPosition();
Serial.println(rightScreen);

moveStepper_relative(xpulse,xdir,(leftScreen-rightScreen)/2,HIGH);
moveStepper_relative(zpulse,zdir,(topScreen-bottomScreen)/2,LOW);

}

