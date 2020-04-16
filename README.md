# Delta-Kinematics-Library

[https://tinkersprojects.com/](https://tinkersprojects.com/)

Forward and Inverse Kinematics Library for Delta robot.

<a href="http://www.youtube.com/watch?feature=player_embedded&v=sGBrFT5dmzU
" target="_blank"><img src="http://img.youtube.com/vi/sGBrFT5dmzU/0.jpg" 
alt="IMAGE ALT TEXT HERE" width="240" height="180" border="10" /></a> 


### Thank you to:
- http://forums.trossenrobotics.com/tutorials/introduction-129/delta-robot-kinematics-3276/
- https://www.marginallyclever.com/other/samples/fk-ik-test.html
- https://github.com/Tomdf/Delta_Robots/blob/master/Diagrams/Delta%20Robot%20Kinematics%20-%20Trossen%20Robotics.pdf
- http://hypertriangle.com/~alex/delta-robot-tutorial/





## Waiting for Image




## Functions
### SETUP
#### DeltaKinematics(double ArmLength,double RodLength,double BassTri,double PlatformTri)
This Fuction is set up the class to calulate the forward and inverse kinematics for the delta. ArmLength, RodLength, BassTri and PlatformTri are the set values from the Delta robot. See image above to see how to take the measurement.

Millimetre, meters, inches or any other lenth measurement can be used for ArmLength, RodLength, BassTri and PlatformTri. The measurement will need to be the same for x, y and z.

**It is recommended to use Millimetre for ArmLength, RodLength, BassTri and PlatformTri**


### SETUP
#### int forward()
#### int forward(double thetaA, double thetaB, double thetaC)
This functions is to calulate the forward kinematics of for the delta. The calulations can be set using the variables a, b and c or in the function with ThetaA, thetaB and thetaC.

ThetaA, thetaB and thetaC valuse are in degrees.
a, b and c valuse are in degrees (see Variables below). See image above.



#### int inverse()
#### int inverse(double x0, double y0, double z0)
This functions is to calulate the inverse kinematics of for the delta. The calulations can be set using the variables x, y and z or in the function with x0, y0 and z0.

**It is recommended to use Millimetre for x, y and z**


## Variables
#### double x, double y, double z
These variables (x, y and z) and be read and written. They are used to set the position of the Delta robot's platform for Kinematics. 
If forward kinematics are calulated, the return values are placed in x, y and z.

**It is recommended to use Millimetre for x, y and z**


#### double a, double b, double c
These variables (a, b and c) and be read and written. They are used to set the angle of the Delta robot's motors for Kinematics. 
If inverse kinematics are calulated, the return values are placed in a, b and c.

a, b and c valuse are in degrees. See image above.





## Example
### Example 1:

```c++
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
```

### Example 2:

```c++
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
```

