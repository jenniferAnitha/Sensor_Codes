void setup()
{
//Pin 6 as Input
  pinMode(6,INPUT);
  //Initiate Serial Communication
  Serial.begin(9600);
// Pin 12 as Out put
  pinMode(12, OUTPUT);
}

void loop()
{
  Serial.print("IR Sensor Input ");
  Serial.println(digitalRead(6));
  if(digitalRead(6)== 0)
  {
   digitalWrite(12,HIGH);
  }
  else
  {
    digitalWrite(12,LOW); 
  }
}

