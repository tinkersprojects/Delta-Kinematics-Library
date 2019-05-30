#include <DeltaInverseKinematics.h>

double angleB1;
double angleB2;
double angleB3;

DeltaInverseKinematics IPK(&angleB1,&angleB2,&angleB3,0.070,0.300,0.139,0.112);

void setup() 
{  
  Serial.begin(115200);
}

void loop() 
{
  IPK.set(0,0,-0.300);  
  Serial.println(String(angleB1)+","+String(angleB2* 180 / 3.14)+","+String(angleB3* 180 / 3.14));
  delay(3000);

  IPK.set(0.100,0.100,-0.270);
  Serial.println(String(angleB1)+","+String(angleB2* 180 / 3.14)+","+String(angleB3* 180 / 3.14));
  delay(3000);

  IPK.set(-0.100,-0.100,-0.270);
  Serial.println(String(angleB1)+","+String(angleB2* 180 / 3.14)+","+String(angleB3* 180 / 3.14));
  delay(3000);
}
