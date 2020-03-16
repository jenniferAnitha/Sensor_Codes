/*#define trigPin 13 //pin 13 as Trig pin

#define echoPin 12 //pin 12 as echo pin

#define led 11 //pin 11 as Led output*/
// defines pins number
const int trigPin=2;
const int echoPin=3;
const int led=10;




float distance=0;
unsigned long duration=0;
float error_Factor=0.65;

void setup()

{ 
Serial.begin (9600);// starts the serial communication

pinMode(trigPin, OUTPUT);//sets the trig pin as an output

pinMode(echoPin, INPUT);//sets the echo pin as output

pinMode(led, OUTPUT);

}

void loop()

{

digitalWrite(trigPin, LOW);

delayMicroseconds(2);
//Sets the trigPin on high state for 10 micro seconds

digitalWrite(trigPin, HIGH);
delayMicroseconds(10);

digitalWrite(trigPin, LOW);
delayMicroseconds(2);

//Reads the echoPin, returns the sound wave travel time in microseconds

duration = pulseIn(echoPin, HIGH);



//calculate the distance (CM and inches)
distance=((duration *0.0340)/2)- error_Factor;


if (distance <= 10)

{ 
  digitalWrite(led,HIGH);

}

else {

digitalWrite(led,LOW);

}

//prints the distance in serial monitor

Serial.print("Distance : ");

Serial.println(distance);

delay(300);

}
