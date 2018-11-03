#include <DeltaInverseKinematics.h>
#include <Servo.h>

Servo servo3;
Servo servo2;
Servo servo1;

double B1;
double B2;
double B3;

DeltaInverseKinematics IPK(&B1,&B2,&B3,100,300,120,40);

void setup() 
{
  servo1.attach(9);
  servo2.attach(9);
  servo3.attach(9);
}

void loop() 
{
  IPK.set(0,0,0);
  servo1.write(map(B1, 0, 1023, 0, 180));
  servo2.write(map(B2, 0, 1023, 0, 180));
  servo3.write(map(B3, 0, 1023, 0, 180));
  delay(3000);

  IPK.set(0,0,30);
  servo1.write(map(B1, 0, 1023, 0, 180));
  servo2.write(map(B2, 0, 1023, 0, 180));
  servo3.write(map(B3, 0, 1023, 0, 180));
  delay(3000);

  IPK.set(100,100,30);
  servo1.write(map(B1, 0, 1023, 0, 180));
  servo2.write(map(B2, 0, 1023, 0, 180));
  servo3.write(map(B3, 0, 1023, 0, 180));
  delay(3000);

  IPK.set(-100,-100,30);
  servo1.write(map(B1, 0, 1023, 0, 180));
  servo2.write(map(B2, 0, 1023, 0, 180));
  servo3.write(map(B3, 0, 1023, 0, 180));
  delay(3000);
}