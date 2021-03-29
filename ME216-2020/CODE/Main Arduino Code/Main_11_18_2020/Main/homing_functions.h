
int findLimit()
{
  limit=digitalRead(7);
  while(limit==HIGH) //Moves slowly towards the limit switch
  {
  limit=digitalRead(7);
  digitalWrite(2,HIGH);
  digitalWrite(3,HIGH);
  delayMicroseconds(200);
  digitalWrite(3,LOW);
  delayMicroseconds(200);
  }
  return 1;
}

int bounceBack()
{
    int i=0;
    digitalWrite(2,LOW);
    while(i<1000) // Back up 1000 steps
    {
    digitalWrite(3,HIGH);
    delayMicroseconds(400);
    digitalWrite(3,LOW);
    delayMicroseconds(400);
    i++;
    }
    return 1;
}

int smoothFind()
{
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
     Coordinates[xsteps]=0;
     digitalWrite(2,LOW);
    int i=0;
    while(i<500) // Back up 500 steps
    {
    digitalWrite(3,HIGH);
    delayMicroseconds(800);
    digitalWrite(3,LOW);
    delayMicroseconds(800);
    i++;
    Coordinates[xsteps]++;
    }
}

