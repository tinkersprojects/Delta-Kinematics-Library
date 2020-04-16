#include <DeltaKinematics.h>

DeltaKinematics DK(70,300,139,112);

void setup() 
{  
  Serial.begin(115200);
}

void loop() 
{
  DK.x =  0;
  DK.y =  0;
  DK.z = -300;
  DK.inverse();
  // OR
  DK.inverse(0,0,-300);

  Serial.println(String(DK.x)+","+String(DK.y)+","+String(DK.z));
  Serial.println(String(DK.a)+","+String(DK.b)+","+String(DK.c));
  Serial.println();
  delay(3000);



  // next position 
  
  DK.x =  0;
  DK.y =  0;
  DK.z = -270;
  DK.inverse();
  // OR
  DK.inverse(000,000,-270);

  Serial.println(String(DK.x)+","+String(DK.y)+","+String(DK.z));
  Serial.println(String(DK.a)+","+String(DK.b)+","+String(DK.c));
  Serial.println();
  delay(3000);



  // next position 
  
  DK.x =  100;
  DK.y =  100;
  DK.z = -270;
  DK.inverse();
  // OR
  DK.inverse(100,100,-270);

  Serial.println(String(DK.x)+","+String(DK.y)+","+String(DK.z));
  Serial.println(String(DK.a)+","+String(DK.b)+","+String(DK.c));
  Serial.println();
  delay(3000);
}