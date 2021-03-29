
int getMove()
{
  xStick=Xbox.getAnalogHat(LeftHatX);
  yStick=Xbox.getAnalogHat(LeftHatY);
  joystick=yStick;
  joystick2=xStick;
  LxScreen = map(joystick,-32767,32767,-1000,1000);
  LyScreen = map(joystick2,-32767,32767,-1000,1000);

  dir=LxScreen;
  //dir2=LyScreen;

  limit=digitalRead(7);
  if(dir<-100 && limit==HIGH && Coordinates[xsteps]<45800) // Move left
  {
    //Serial.println("left");
    //Coordinates[xsteps]-=(dir);
    //stepperX.moveTo(Coordinates[xsteps]);
    del=(1100-(-1*(LxScreen-100)))/2;
    digitalWrite(11,LOW);
    digitalWrite(8,HIGH);
    delayMicroseconds((int)del);
    digitalWrite(8,LOW);
    delayMicroseconds((int)del);
    Coordinates[xsteps]++;
  }
 
  
  if(dir>100 && limit==HIGH && Coordinates[xsteps]>0) // Move Right
  {
    //Serial.println("right");
    //Coordinates[xsteps]+=(dir);
    //stepperX.moveTo(Coordinates[xsteps]);
    del=(1100-(LxScreen+100))/2;
    digitalWrite(11,HIGH);
    digitalWrite(8,HIGH);
    delayMicroseconds((int)del);
    digitalWrite(8,LOW);
    delayMicroseconds((int)del);
    Coordinates[xsteps]--;
    
  }

  if(dir2<-100 && limit==HIGH) // Move left
  {
    del=(1100-(-1*(LyScreen-100)))/2;
    digitalWrite(12,LOW);
    digitalWrite(9,HIGH);
    delayMicroseconds((int)del);
    digitalWrite(9,LOW);
    delayMicroseconds((int)del);
    Coordinates[ysteps]--;
  }
  
  if(dir2>100 && limit==HIGH) // Move Right
  {
    del=(1100-(LyScreen+100))/2;
    digitalWrite(12,HIGH);
    digitalWrite(9,HIGH);
    delayMicroseconds((int)del);
    digitalWrite(9,LOW);
    delayMicroseconds((int)del);
    Coordinates[ysteps]++;
  }   
}

