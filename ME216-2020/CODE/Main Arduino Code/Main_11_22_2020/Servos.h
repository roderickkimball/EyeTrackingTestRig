
float floatMap(float x, float in_min, float in_max, float out_min, float out_max) 
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

int GetPos()
{
  //Usb.Task();
    if (Xbox.XboxOneConnected) {
    if (Xbox.getAnalogHat(LeftHatX) > lim || Xbox.getAnalogHat(LeftHatX) < -lim || Xbox.getAnalogHat(LeftHatY) > lim || Xbox.getAnalogHat(LeftHatY) < -lim || Xbox.getAnalogHat(RightHatX) > lim || Xbox.getAnalogHat(RightHatX) < -lim || Xbox.getAnalogHat(RightHatY) > lim || Xbox.getAnalogHat(RightHatY) < -lim) {

       xStick=Xbox.getAnalogHat(LeftHatX);
       yStick=Xbox.getAnalogHat(LeftHatY);
      
      if (Xbox.getAnalogHat(LeftHatX) > lim || Xbox.getAnalogHat(LeftHatX) < -lim) {
        //Serial.print(F("LeftHatX: "));
        //Serial.print(Xbox.getAnalogHat(LeftHatX));
        //Serial.print("\t");
        //DotPos[xpos]+=map(xStick,-32767,32767,-1,1);
        DotPos[xpos]+=(xStick/32767);
////DotPos[xpos]+=floatMap(xStick,-32767,32767,-.1,.1);
//    if(DotPos[xpos]>292.1)
//    //DotPos[xpos]=292.1;
//    if(DotPos[xpos]<0)
//   // DotPos[xpos]=0;
      }
      if (Xbox.getAnalogHat(LeftHatY) > lim || Xbox.getAnalogHat(LeftHatY) < -lim) {
        //Serial.print(F("LeftHatY: "));
        //Serial.print(Xbox.getAnalogHat(LeftHatY));
        //Serial.print("\t");
        //DotPos[ypos]+=map(yStick,-32767,32767,-1,1);
        DotPos[ypos]+=(yStick/32767);
////DotPos[ypos]+=floatMap(yStick,-32767,32767,-.1,.1);
//    if(DotPos[ypos]>200.25)
////    DotPos[ypos]=200.25;
////DotPos[ypos]=500.25;
//    if(DotPos[ypos]<0)
////    DotPos[ypos]=0;
////DotPos[ypos]=-200;
        
      }
      if (Xbox.getAnalogHat(RightHatX) > lim || Xbox.getAnalogHat(RightHatX) < -lim) {
        //Serial.print(F("RightHatX: "));
       // Serial.print(Xbox.getAnalogHat(RightHatX));
       // Serial.print("\t");
      }
      if (Xbox.getAnalogHat(RightHatY) > lim || Xbox.getAnalogHat(RightHatY) < -lim) {
       // Serial.print(F("RightHatY: "));
       // Serial.print(Xbox.getAnalogHat(RightHatY));
      }
     // Serial.println();
     
    }
Serial.print("Xpos:  ");
Serial.print(DotPos[xpos]);
//Serial.println(xStick);
Serial.print("\t");
Serial.print("Ypos:  ");
Serial.println(DotPos[ypos]);
//Serial.println(yStick);
  
//  LxScreen = map(xStick,-32767,32767,-1000,1000);
//  LyScreen = map(yStick,-32767,32767,-1000,1000);
}
}

int MoveDot()
{
  // Moves the dot around the 2D screen. Larger stick inputs will result in a faster position change ie. distance increment per loop. 

  if(abs(xStick)>threshold) //Moves left or right
  {
    DotPos[xpos]+=map(xStick,-32767,32767,-1,1);
//DotPos[xpos]+=floatMap(xStick,-32767,32767,-.1,.1);
    if(DotPos[xpos]>292.1)
//    DotPos[xpos]=292.1;
DotPos[xpos]=592.1;
    if(DotPos[xpos]<0)
//    DotPos[xpos]=0;
DotPos[xpos]=-292;
//    Serial.print(xStick);
//    Serial.print("\t");
//    Serial.println(yStick);
  }

  if(abs(yStick)>threshold) //Moves up or down
  {
    DotPos[ypos]+=map(yStick,-32767,32767,-1,1);
//DotPos[ypos]+=floatMap(yStick,-32767,32767,-.1,.1);
    if(DotPos[ypos]>200.25)
    DotPos[ypos]=200.25;
    if(DotPos[ypos]<0)
    DotPos[ypos]=0;
//    Serial.print(xStick);
//    Serial.print("\t");
//    Serial.println(yStick);
  }
//Serial.print("Xpos:  ");
//Serial.print(DotPos[xpos]);
//Serial.print("\t");
//Serial.print("Ypos:  ");
//Serial.println(DotPos[ypos]);
}

void parallax(){

  //DotPos[2]={0,0}

  //Left Eye
  LxEye = xStep-(0.5*IPD);  
  LzEye = zStep+eyeZlocation;

  deltaX = DotPos[0]-LxEye;
  deltaZ = DotPos[1]-LzEye;

  leftLength = sqrt(deltaX*deltaX+deltaZ*deltaZ+deltaY*deltaY);

  alphaLeft = asin(deltaX/leftLength)*(180/3.14159);
  betaLeft = asin(deltaZ/leftLength)*(180/3.14159);

  //Right Eye
  RxEye = xStep+(0.5*IPD);  
  RzEye = zStep+eyeZlocation;

  deltaX = DotPos[0]-RxEye;
  deltaZ = DotPos[1]-RzEye;

  rightLength = sqrt(deltaX*deltaX+deltaZ*deltaZ+deltaY*deltaY);

  alphaRight = asin(deltaX/rightLength)*(180/3.14159);
  betaRight = -asin(deltaZ/rightLength)*(180/3.14159);


XservoL.write(alphaLeft+90);
YservoL.write(betaLeft+90);

XservoR.write(alphaRight+90);
YservoR.write(betaRight+90);


//Serial.print(alphaLeft+90);
//Serial.print("\t");
//Serial.println(betaLeft+90);

}





