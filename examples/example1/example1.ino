#include <DeltaInverseKinematics.h>

double B1;
double B2;
double B3;

DeltaInverseKinematics IPK(&B1,&B2,&B3,100,300,120,40);

void setup() 
{
  Serial.begin(115200);
}

void loop() 
{
  IPK.set(0,0,0);
  Serial.println(B1+","+B2+","+B3);
  delay(3000);

  IPK.set(0,0,30);
  Serial.println(B1+","+B2+","+B3);
  delay(3000);

  IPK.set(100,100,30);
  Serial.println(B1+","+B2+","+B3);
  delay(3000);

  IPK.set(-100,-100,30);
  Serial.println(B1+","B2+","+B3);
  delay(3000);
}