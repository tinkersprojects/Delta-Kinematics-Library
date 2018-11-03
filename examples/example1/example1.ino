#include <DeltaInverseKinematics.h>

double B1angle;
double B2angle;
double B3angle;

DeltaInverseKinematics IPK(&B1angle,&B2angle,&B3angle,100,300,120,40);

void setup() 
{
  Serial.begin(115200);
}

void loop() 
{
  IPK.set(0,0,0);
  Serial.println(String(B1angle)+","+String(B2angle)+","+String(B3angle));
  delay(3000);

  IPK.set(0,0,30);
  Serial.println(String(B1angle)+","+String(B2angle)+","+String(B3angle));
  delay(3000);

  IPK.set(100,100,30);
  Serial.println(String(B1angle)+","+String(B2angle)+","+String(B3angle));
  delay(3000);

  IPK.set(-100,-100,30);
  Serial.println(String(B1angle)+","+String(B2angle)+","+String(B3angle));
  delay(3000);
}