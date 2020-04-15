#include <DeltaKinematics.h>

DeltaKinematics DK(0.070,0.300,0.139,0.112);

void setup() 
{  
  Serial.begin(115200);
}

void loop() 
{
  DK.x =  0.000;
  DK.y =  0.000;
  DK.z = -0.300;
  DK.inverse();
  Serial.println(String(DK.a)+","+String(DK.b* 180 / 3.14)+","+String(DK.c* 180 / 3.14));
  delay(3000);

  DK.x =  0.100;
  DK.y =  0.100;
  DK.z = -0.270;
  DK.inverse();
  Serial.println(String(DK.a)+","+String(DK.b* 180 / 3.14)+","+String(DK.c* 180 / 3.14));
  delay(3000);

  DK.x = -0.100;
  DK.y = -0.100;
  DK.z = -0.100;
  DK.inverse();
  Serial.println(String(DK.a)+","+String(DK.b* 180 / 3.14)+","+String(DK.c* 180 / 3.14));
  delay(3000);
}