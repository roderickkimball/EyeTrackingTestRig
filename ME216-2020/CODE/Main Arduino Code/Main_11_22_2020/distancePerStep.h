void stepsToInches(int dir, int pulse, int limm)//direction pin, pulse pin, limit pin
{
    digitalWrite(dir,LOW);
    //i=0;
    while(i<limm) // Back up 1000 steps
    {
    digitalWrite(pulse,HIGH);
    delayMicroseconds(400);
    digitalWrite(pulse,LOW);
    delayMicroseconds(400);
    i++;
    }
}


