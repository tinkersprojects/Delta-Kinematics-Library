#include <DeltaKinematics.h>
#include <Servo.h>

Servo servo3;
Servo servo2;
Servo servo1;

DeltaKinematics DK(70,300,139,112);

void setup() 
{  
  Serial.begin(115200);

  servo1.attach(7);
  servo2.attach(6);
  servo3.attach(5);
}

void servo()
{
  Serial.println(String(DK.x)+","+String(DK.y)+","+String(DK.z));
  Serial.println(String(DK.a)+","+String(DK.b)+","+String(DK.c));
  Serial.println();
  servo1.write(DK.a);
  servo2.write(DK.b);
  servo3.write(DK.c);
}

void loop() 
{
  DK.x =  0;
  DK.y =  0;
  DK.z = -300;
  DK.inverse();
  // OR
  DK.inverse(0,0,-300);

  servo();
  delay(3000);



  // next position 
  
  DK.x =  0;
  DK.y =  0;
  DK.z = -270;
  DK.inverse();
  // OR
  DK.inverse(000,000,-270);

  servo();
  delay(3000);



  // next position 
  
  DK.x =  100;
  DK.y =  100;
  DK.z = -270;
  DK.inverse();
  // OR
  DK.inverse(100,100,-270);

  servo();
  delay(3000);
}