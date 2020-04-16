#include <DeltaKinematics.h>
#include <Servo.h>

Servo servo3;
Servo servo2;
Servo servo1;

DeltaKinematics DK(0.070,0.300,0.139,0.112);

void setup() 
{  
  Serial.begin(115200);

  servo1.attach(7);
  servo2.attach(6);
  servo3.attach(5);
}

void servo()
{
  Serial.println(String(DK.a)+","+String(DK.b)+","+String(DK.c));
  servo1.write(DK.a);
  servo2.write(DK.b);
  servo3.write(DK.c);
}

void loop() 
{
  DK.x =  0.000;
  DK.y =  0.000;
  DK.z = -0.300;
  DK.inverse();
  // OR
  DK.inverse(0.000,0.000,-0.300);

  servo();
  delay(3000);



  // next position 
  
  DK.x =  0.000;
  DK.y =  0.000;
  DK.z = -0.300;
  DK.inverse();
  // OR
  DK.inverse(0.000,0.000,-0.270);

  servo();
  delay(3000);



  // next position 
  
  DK.x =  0.000;
  DK.y =  0.000;
  DK.z = -0.300;
  DK.inverse();
  // OR
  DK.inverse(0.100,0.100,-0.270);

  servo();
  delay(3000);
}
