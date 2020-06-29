#include <Servo.h>
Servo s1;
int servo_pin=9;


void setup() {
  s1.attach(servo_pin); 
}

void loop() {
  // put your main code here, to run repeatedly:
s1.write(0);//Moving the servo 0 degrees
delay(1000);
s1.write(90);//Moving the servo 90 degrees
delay(1000);

}
