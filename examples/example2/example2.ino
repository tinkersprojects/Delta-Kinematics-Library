#include <DeltaInverseKinematics.h>
#include <Servo.h>

Servo servo3;
Servo servo2;
Servo servo1;

double angleB1;
double angleB2;
double angleB3;

DeltaInverseKinematics IPK(&angleB1,&angleB2,&angleB3,70,300,139,112);

void setup() 
{  
  Serial.begin(115200);

  servo1.attach(7);
  servo2.attach(6);
  servo3.attach(5);
}

void servo()
{
  //Serial.println(String(angleB1* 180 / 3.14)+","+String(angleB2* 180 / 3.14)+","+String(angleB3* 180 / 3.14));
/*
  servo1.write(0);
  servo2.write(0);
  servo3.write(0);*/
  servo1.write(angleB1* 180 / 3.14);
  servo2.write(angleB2* 180 / 3.14);
  servo3.write(angleB3* 180 / 3.14);
}

void loop() 
{
  IPK.set(0,0,-300);
  servo();
  delay(3000);

  IPK.set(0,0,-270);
  servo();
  delay(3000);

  IPK.set(100,100,-270);
  servo();
  delay(3000);
  /*
  IPK.set(-100,100,-270);
  servo();
  delay(3000);
  
  IPK.set(-100,-100,-270);
  servo();
  delay(3000);
  
  IPK.set(100,-100,-270);
  servo();
  delay(3000);
  
  IPK.set(100,100,-270);
  servo();
  delay(3000);
  
  IPK.set(0,0,-270);
  servo();
  delay(3000);*/
}