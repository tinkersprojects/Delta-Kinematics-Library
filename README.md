# Delta-Kinematics-Library (still working on some of the library)
Forward and Inverse Kinematics Library for Delta robot.

<a href="http://www.youtube.com/watch?feature=player_embedded&v=sGBrFT5dmzU
" target="_blank"><img src="http://img.youtube.com/vi/sGBrFT5dmzU/0.jpg" 
alt="IMAGE ALT TEXT HERE" width="240" height="180" border="10" /></a> 


### Thank you to:
R.L. Williams II, “The Delta Parallel Robot: Kinematics Solutions”, Internet Publication, www.ohio.edu/people/williar4/html/pdf/DeltaKin.pdf, January 2016.

## Functions
### SETUP
#### DeltaInverseKinematics(double *angleB1,double *angleB2,double *angleB3,double ArmLength,double RodLength,double BassTri,double PlatformTri);
This Fuction is set up the variables that will be return for the motors. ArmLength, RodLength, BassTri and PlatformTri are the set values from the Delta robot. 
        
### SET 
#### void set(double x,double y,double z);
This function to calculate the angle of each motor given the position X, Y and Z. The motor angles will be return throug the variables declared in the setup;angleB1, angleB2 and angleB3. the values of the angles is in radens.

#### void setOffsets(double X, double Y, double Z);
This fuctions is used to set the Offsets of X, Y and Z.

#### void setLimits(double upperX, double upperY, double upperZ, double lowerX, double lowerY, double lowerZ);
This fuctions is used to set the angle limits of each motor.

## Example
### Example 1:

```c++
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
```

### Example 2:

```c++
#include <DeltaInverseKinematics.h>
#include <Servo.h>

Servo servo3;
Servo servo2;
Servo servo1;

double angleB1;
double angleB2;
double angleB3;

DeltaInverseKinematics IPK(&angleB1,&angleB2,&angleB3,0.070,0.300,0.139,0.112);

void setup() 
{  
  Serial.begin(115200);

  servo1.attach(7);
  servo2.attach(6);
  servo3.attach(5);
}

void servo()
{
  Serial.println(String(angleB1* 180 / 3.14)+","+String(angleB2* 180 / 3.14)+","+String(angleB3* 180 / 3.14));
  servo1.write(angleB1* 180 / 3.14);
  servo2.write(angleB2* 180 / 3.14);
  servo3.write(angleB3* 180 / 3.14);
}

void loop() 
{
  IPK.set(0,0,-0.300);
  servo();
  delay(3000);

  IPK.set(0,0,-0.270);
  servo();
  delay(3000);

  IPK.set(0.100,0.100,-0.270);
  servo();
  delay(3000);
}
```

