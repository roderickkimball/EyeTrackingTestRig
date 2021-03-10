void servoCalibration(){

   x = (0.5*IPD); //x = distance from left side of screen to eye
   z = (eyeZlocation); //distance between top of screen and eye  //eyeZlocation is the measured distance between chin laser and eye
   alphaLeft = atan(x/deltaY)*(180/3.14159);
   betaLeft = atan(z/deltaY)*(180/3.14159);
   alphaRight = atan(x/deltaY)*(180/3.14159);
   betaRight = atan(z/deltaY)*(180/3.14159);
  
  //Step 1: Point left eye and right eye at the chin laser
    i=0;
    while(i==0) //Waits for B to be pressed on Xbox controller
    {
      Usb.Task();
      if(Xbox.getButtonClick(B))
      {
        i=1;
      }
      GetPos();
      leftEye();
    }
leftXhome = XservoL.read() + alphaLeft;  //servo x angle for left eye center
leftZhome = ZservoL.read() - betaLeft;  //servo z angle for left eye center


delay(1000);

    i=0;
    while(i==0) //Waits for B to be pressed on Xbox controller
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

  DotPos[xpos]=screenWidth/2;
  DotPos[zpos]=screenHeight/2;

  Serial.print(leftXhome);
  Serial.print("\t");
  Serial.println(rightXhome);
}

void stepperCalibration()
{
  //Step 1: Home the rig.
  zeroStepper(xpulse,xdir,xEnd);
  stepperX.setCurrentPosition(-500);
  zeroStepper(ypulse,ydir,yEnd);
  stepperY.setCurrentPosition(500);
  zeroStepper(zpulse,zdir,zEnd);
  stepperZ.setCurrentPosition(500);
  
  //Step 2: Point chin laser at the bottom left corner of the screen @ 65cm.
  moveStepper_relative(zpulse,zdir,10188,HIGH);
  moveStepper_relative(ypulse,ydir,8335,LOW); // Last input parameter: Steps from home position to 65cm.

Usb.Task();

i=0;
while(i==0) //Waits for B to be pressed on Xbox controller
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

bottomScreen=abs(stepperZ.currentPosition());
leftScreen=abs(stepperX.currentPosition());
Serial.println("Bottom Left Corner");
Serial.print(bottomScreen);
Serial.print("\t");
Serial.println(leftScreen);

delay(1000);


i=0;
while(i==0) //Waits for B to be pressed on Xbox controller
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

topScreen=abs(stepperZ.currentPosition());
rightScreen=abs(stepperX.currentPosition());
Serial.println("Top Right Corner");
Serial.print(topScreen);
Serial.print("\t");
Serial.println(rightScreen);


moveStepper_relative(xpulse,xdir,(rightScreen-leftScreen)/2,HIGH);
moveStepper_relative(zpulse,zdir,(topScreen-bottomScreen)/2,LOW);


Serial.println("Current Position");
Serial.print(stepperZ.currentPosition());
Serial.print("\t");
Serial.println(abs(stepperX.currentPosition()));

  DotPos[xpos]=screenWidth/2;
  DotPos[zpos]=screenHeight/2;
}


void findCoordinates() // This function will be used whenever the Tobii Calibration coordinates need to be reset
{
   int j=0;
   while(j==0)
   {
    while(i==0&&j==0)
    {
      Usb.Task();
       GetPos();
       parallax();
       i=0;
      if(Xbox.getButtonClick(B))
      {
        i=1;
      }
      if(Xbox.getButtonClick(X))
      {
        j=1;
      }
     
    }
/* The X and Z coordinates will be displayed on the serial monitor one at a time. 
 *  These must be writeen down and entered into the calibrationCoords array. 
 */
Serial.print(DotPos[xpos]);
Serial.print("\t");
Serial.println(DotPos[zpos]);
i=0;
   }
   state=MENU;
}

void setPos() // This function will steer the gaze of the artificial eyes at the calibration points during the Tobii Calibration.
{
  for(int j=0;j<7;j++)
  {
    DotPos[0]=calibrationCoords[0][j]; // X Coordinate
    DotPos[1]=calibrationCoords[1][j]; // Z Coordinate
    parallax();
    i=0;
  while(i==0) // Waits for the user to advance the index by clicking X
    {
      Usb.Task();
       i=0;
      if(Xbox.getButtonClick(X))
      {
        i=1;
      } 
    }
    i=0;
 }
 state=MENU;
}

