void servoCalibration(){

  
  //Step 1: Point left eye and right eye at the chin laser


   x = (0.5*IPD); //x = distance from left side of screen to eye
   z = (eyeZlocation); //distance between top of screen and eye  //eyeZlocation is the measured distance between chin laser and eye
   alphaLeft = asin(x/deltaY);
   betaLeft = asin(z/deltaY);

   leftXhome = XservoL.read() - alphaLeft;  //servo x angle for left eye center
   leftZhome = YservoL.read() - betaLeft;  //servo z angle for left eye center

   alphaRight = asin(x/deltaY);
   betaRight = asin(z/deltaY);

   rightXhome = XservoR.read() - alphaRight;  //servo x angle for right eye center
   rightZhome = XservoR.read() - betaRight;  //servo z angle for right eye center

}

void stepperCalibration()
{
  //Step 1: Point chin laser at the bottom left corner of the screen @ 65cm.
//runHomeState();
//state=MENU;
  //Step 2: Hit A on the XBOX Controller. This will zero the stepper motors to the bottom left of the screen

  //Step 3: Point the chin laser at the top right corner of the sceen. 

  /*Step 4: Hit A on the XBOX Controller. The code will perform some calculations to find the screen dimentions. 
   *        The steppers will now move the chin laser to the center of the screen to prepare for the servo calibration.
   *
   */
}

