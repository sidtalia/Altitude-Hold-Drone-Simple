#include<Servo.h>

Servo FL,FR,BL,BR;

void setup()
{
  FL.attach(3);
  FR.attach(4);
  BL.attach(5);
  BR.attach(6);
  FL.writeMicroseconds(2000);
  FR.writeMicroseconds(2000);
  BL.writeMicroseconds(2000);
  BR.writeMicroseconds(2000);
  delay(1000);//give 1 second delay
  FL.writeMicroseconds(1000);
  FR.writeMicroseconds(1000);
  BL.writeMicroseconds(1000);
  BR.writeMicroseconds(1000);
  
}

void loop()
{
}

