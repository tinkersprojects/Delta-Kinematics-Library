/**********************************************************************************************
 * Arduino Delta Kinematics- Version 1.0
 * by William Bailes <williambailes@gmail.com> http://tinkersprojects.com/
 *
 * This Library is licensed under a GPLv3 License
 * 
 * made with infomation from the following documents
 * - http://forums.trossenrobotics.com/tutorials/introduction-129/delta-robot-kinematics-3276/
 * - https://www.marginallyclever.com/other/samples/fk-ik-test.html
 * - https://github.com/Tomdf/Delta_Robots/blob/master/Diagrams/Delta%20Robot%20Kinematics%20-%20Trossen%20Robotics.pdf
 * - http://hypertriangle.com/~alex/delta-robot-tutorial/
 **********************************************************************************************/

#include "DeltaKinematics.h"

#if ARDUINO >= 100
  #include "Arduino.h"
#else
  #include "WProgram.h"
#endif

#include <math.h>

/******************* SETUP *******************/

DeltaKinematics::DeltaKinematics(double _ArmLength,double _RodLength,double _BassTri,double _PlatformTri)
{
  PlatformTri = _PlatformTri;            // end effector
  BassTri = _BassTri;            // base
  RodLength = _RodLength;
  ArmLength = _ArmLength;
}











// forward kinematics: (thetaA, thetaB, thetaC) -> (x0, y0, z0)
int DeltaKinematics::forward() 
{
  return forward(a, b, c);
}

int DeltaKinematics::forward(double thetaA, double thetaB, double thetaC) 
{
  x=0.0;
  y=0.0;
  z=0.0;
  
  double t = (BassTri-PlatformTri)*tan30/2.0;
  double dtr = pi/180.0;

  thetaA *= dtr;
  thetaB *= dtr;
  thetaC *= dtr;

  double y1 = -(t + ArmLength*cos(thetaA));
  double z1 = -ArmLength*sin(thetaA);

  double y2 = (t + ArmLength*cos(thetaB))*sin30;
  double x2 = y2*tan60;
  double z2 = -ArmLength*sin(thetaB);

  double y3 = (t + ArmLength*cos(thetaC))*sin30;
  double x3 = -y3*tan60;
  double z3 = -ArmLength*sin(thetaC);

  double dnm = (y2-y1)*x3-(y3-y1)*x2;

  double w1 = y1*y1 + z1*z1;
  double w2 = x2*x2 + y2*y2 + z2*z2;
  double w3 = x3*x3 + y3*y3 + z3*z3;

  // x = (a1*z + b1)/dnm
  double a1 = (z2-z1)*(y3-y1)-(z3-z1)*(y2-y1);
  double b1 = -((w2-w1)*(y3-y1)-(w3-w1)*(y2-y1))/2.0;

  // y = (a2*z + b2)/dnm;
  double a2 = -(z2-z1)*x3+(z3-z1)*x2;
  double b2 = ((w2-w1)*x3 - (w3-w1)*x2)/2.0;

  // a*z^2 + b*z + c = 0
  double aV = a1*a1 + a2*a2 + dnm*dnm;
  double bV = 2.0*(a1*b1 + a2*(b2-y1*dnm) - z1*dnm*dnm);
  double cV = (b2-y1*dnm)*(b2-y1*dnm) + b1*b1 + dnm*dnm*(z1*z1 - RodLength*RodLength);

  // discriminant
  double dV = bV*bV - 4.0*aV*cV;
  if (dV < 0.0)
  {
    return non_existing_povar_error; // non-existing povar. return error,x,y,z
  }
  
  z = -0.5*(bV+sqrt(dV))/aV;
  x = (a1*z + b1)/dnm;
  y = (a2*z + b2)/dnm;

  return no_error;
}












// inverse kinematics
// helper functions, calculates angle thetaA (for YZ-pane)
int DeltaKinematics::delta_calcAngleYZ(double *Angle, double x0, double y0, double z0)
{
  double y1 = -0.5 * 0.57735 * BassTri;  // f/2 * tan(30 deg)
      y0 -= 0.5 * 0.57735 * PlatformTri;  // shift center to edge

  // z = a + b*y
  double aV = (x0*x0 + y0*y0 + z0*z0 +ArmLength*ArmLength - RodLength*RodLength - y1*y1)/(2.0*z0);
  double bV = (y1-y0)/z0;

  // discriminant
  double dV = -(aV+bV*y1)*(aV+bV*y1)+ArmLength*(bV*bV*ArmLength+ArmLength); 
  if (dV < 0)
  {
    return non_existing_povar_error; // non-existing povar.  return error, theta
  }

  double yj = (y1 - aV*bV - sqrt(dV))/(bV*bV + 1); // choosing outer povar
  double zj = aV + bV*yj;
  *Angle = atan2(-zj,(y1 - yj)) * 180.0/pi + ((yj>y1)?180.0:0.0);

  return no_error;  // return error, theta
}











// inverse kinematics: (x0, y0, z0) -> (thetaA, thetaB, thetaC)
int DeltaKinematics::inverse() 
{
  inverse(x, y, z);
}

int DeltaKinematics::inverse(double x0, double y0, double z0) 
{
  a = 0;
  b = 0;
  c = 0;
  int error = delta_calcAngleYZ(&a, x0, y0, z0);
  if(error != no_error)
    return no_error;
  error = delta_calcAngleYZ(&b, x0*cos120 + y0*sin120, y0*cos120-x0*sin120, z0);
  if(error != no_error)
    return no_error;
  error = delta_calcAngleYZ(&c, x0*cos120 - y0*sin120, y0*cos120+x0*sin120, z0);

  return no_error;
}
