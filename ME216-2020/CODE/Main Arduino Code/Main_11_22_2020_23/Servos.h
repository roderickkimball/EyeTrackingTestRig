
float floatMap(float x, float in_min, float in_max, float out_min, float out_max) 
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

int GetPos()
{
    if (Xbox.XboxOneConnected) {
    if (Xbox.getAnalogHat(LeftHatX) > lim || Xbox.getAnalogHat(LeftHatX) < -lim || Xbox.getAnalogHat(LeftHatY) > lim || Xbox.getAnalogHat(LeftHatY) < -lim || Xbox.getAnalogHat(RightHatX) > lim || Xbox.getAnalogHat(RightHatX) < -lim || Xbox.getAnalogHat(RightHatY) > lim || Xbox.getAnalogHat(RightHatY) < -lim) {

       xStick=Xbox.getAnalogHat(LeftHatX);
       zStick=Xbox.getAnalogHat(LeftHatY);
      
      if (Xbox.getAnalogHat(LeftHatX) > lim || Xbox.getAnalogHat(LeftHatX) < -lim) {
    
        DotPos[xpos]+=(xStick/32767);

      }
      if (Xbox.getAnalogHat(LeftHatY) > lim || Xbox.getAnalogHat(LeftHatY) < -lim) {
      
        DotPos[zpos]+=(zStick/32767);

        
      }
      if (Xbox.getAnalogHat(RightHatX) > lim || Xbox.getAnalogHat(RightHatX) < -lim) {
       
      }
      if (Xbox.getAnalogHat(RightHatY) > lim || Xbox.getAnalogHat(RightHatY) < -lim) {
      
     
    }


}
}
}

int MoveDot()
{
  // Moves the dot around the 2D screen. Larger stick inputs will result in a faster position change ie. distance increment per loop. 

  if(abs(xStick)>threshold) //Moves left or right
  {
    DotPos[xpos]+=map(xStick,-32767,32767,-1,1);
    if(DotPos[xpos]>292.1)
    DotPos[xpos]=592.1;
    if(DotPos[xpos]<0)
    DotPos[xpos]=-292;
  }

  if(abs(zStick)>threshold) //Moves up or down
  {
    DotPos[zpos]+=map(zStick,-32767,32767,-1,1);
    if(DotPos[zpos]>200.25)
    DotPos[zpos]=200.25;
    if(DotPos[zpos]<0)
    DotPos[zpos]=0;
}
}

void parallax(){

xStep=(stepperX.currentPosition()-leftScreen)*StepsPerMM;
zStep=(stepperZ.currentPosition()-bottomScreen)*StepsPerMM;

  //Left Eye
  LxEye = xStep-(0.5*IPD);  
  LzEye = zStep+eyeZlocation;

  deltaX = DotPos[0]-LxEye;
  deltaZ = DotPos[1]-LzEye;

  leftLength = sqrt(deltaX*deltaX+deltaZ*deltaZ+deltaY*deltaY);
  //leftLength=650;

//  alphaLeft = asin(deltaX/leftLength)*(180/3.14159);
//  betaLeft = asin(deltaZ/leftLength)*(180/3.14159);

//alphaLeft = asin(deltaX/leftLength);
//betaLeft = asin(deltaZ/leftLength);

i=((deltaX/leftLength)+.5)/(2/(lookupRes-1));
Serial.print(i);
i=constrain(i,12,260);
alphaLeft = Asin_Table[i]*(180/3.14159);


i=((deltaZ/leftLength)+.5)/(2/(lookupRes-1));
i=constrain(i,0,290);
betaLeft = Asin_Table[abs(i)]*(180/3.14159);

//Serial.print(i);
Serial.print("\t");
Serial.print(alphaLeft);
Serial.print("\t");

  //Right Eye
  RxEye = xStep+(0.5*IPD);  
  RzEye = zStep+eyeZlocation;

  deltaX = DotPos[0]-RxEye;
  deltaZ = DotPos[1]-RzEye;

  rightLength = sqrt(deltaX*deltaX+deltaZ*deltaZ+deltaY*deltaY);
  //rightLength=650;

//  alphaRight = asin(deltaX/rightLength)*(180/3.14159);
//  betaRight = -asin(deltaZ/rightLength)*(180/3.14159);
 
//  alphaRight = asin(deltaX/rightLength);
//  betaRight = -asin(deltaZ/rightLength);

i=((deltaX/rightLength)+.5)/(2/(lookupRes-1));
Serial.print(i);
i=constrain(i,12,260);
alphaRight = Asin_Table[i]*(180/3.14159);

i=((deltaZ/rightLength)+.5)/(2/(lookupRes-1));
i=constrain(i,0,290);
betaRight = -1*Asin_Table[abs(i)]*(180/3.14159);

//Serial.print(i);
Serial.print("\t");
Serial.print(alphaRight);
Serial.print("\t");

XservoL.write(leftXhome+alphaLeft);
ZservoL.write(leftZhome+betaLeft);

XservoR.write(rightXhome+alphaRight);
ZservoR.write(rightZhome+betaRight);

Serial.print(leftXhome+alphaLeft);
Serial.print("\t");
Serial.print(leftZhome+betaLeft);
Serial.print("\t");
Serial.print(rightXhome+alphaRight);
Serial.print("\t");
Serial.println(rightZhome+betaRight);

//Serial.print("\t");
//Serial.print(leftXhome);
//Serial.print("\t");
//Serial.print(leftZhome);
//Serial.print("\t");
//Serial.print(rightXhome);
//Serial.print("\t");
//Serial.println(rightZhome);

}

void leftEye()
{
  //Left Eye
  LxEye = xStep-(0.5*IPD);  
  LzEye = zStep+eyeZlocation;

  deltaX = DotPos[0]-LxEye;
  deltaZ = DotPos[1]-LzEye;

  leftLength = sqrt(deltaX*deltaX+deltaZ*deltaZ+deltaY*deltaY);

  alphaLeft = asin(deltaX/leftLength)*(180/3.14159);
  betaLeft = asin(deltaZ/leftLength)*(180/3.14159);

  XservoL.write(alphaLeft+90);
  ZservoL.write(betaLeft+90);
}

void rightEye()
{
  RxEye = xStep+(0.5*IPD);  
  RzEye = zStep+eyeZlocation;

  deltaX = DotPos[0]-RxEye;
  deltaZ = DotPos[1]-RzEye;

  rightLength = sqrt(deltaX*deltaX+deltaZ*deltaZ+deltaY*deltaY);

  alphaRight = asin(deltaX/rightLength)*(180/3.14159);
  betaRight = -asin(deltaZ/rightLength)*(180/3.14159);

  XservoR.write(alphaRight+90);
  ZservoR.write(betaRight+90);
}




