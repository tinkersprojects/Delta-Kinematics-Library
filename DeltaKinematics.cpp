/**********************************************************************************************
 * Arduino Delta Kinematics - Version 1.0
 * by William Bailes <williambailes@gmail.com> http://tinkersprojects.com/
 *
 * This Library is licensed under a GPLv3 License
 **********************************************************************************************/

#include "DeltaKinematics.h"
#include <math.h>

/******************* SETUP *******************/

DeltaKinematics::DeltaKinematics(double *xtemp,double *ytemp,double *ztemp,double ArmLength,double RodLength,double BassTri,double PlatformTri)
{
    X = xtemp;
    Y = ytemp;
    Z = ztemp; 

    L = ArmLength;
    l = RodLength;

    Sb = BassTri;
    Sp = PlatformTri;

    Ub = Sb * 0.57735026919;
    Wb = Sb * 0.28867513459;

    Up = Sp * 0.57735026919;
    Wp = Sp * 0.28867513459;
}
        
/******************* SET *******************/

void DeltaKinematics::setLimits(double upperB1, double upperB2, double upperB3, double lowerB1, double lowerB2, double lowerB3)
{
  limitUpperB1 = upperB1;
  limitUpperB2 = upperB2;
  limitUpperB3 = upperB3;
  limitLowerB1 = lowerB1;
  limitLowerB2 = lowerB2;
  limitLowerB3 = lowerB3;
}


void DeltaKinematics::setOffset(double x,double y,double z)
{
    offsetX = x;
    offsetY = y;
    offsetZ = z;
}


void DeltaKinematics::set(double a,double b,double c)
{
  double xa;
  double ya;
  double za;
  double xb;
  double yb;
  double zb;

  DeltaKinematics::calulate(&xa,&ya,&za,&xb,&yb,&zb,a,b,c);
  
}

        
/******************* TEST *******************/

bool DeltaKinematics::test(double a,double b,double c)
{
  double xa;
  double ya;
  double za;
  double xb;
  double yb;
  double zb;

  DeltaKinematics::calulate(&xa,&ya,&za,&xb,&yb,&zb,a,b,c);

  return true;
}


        
/******************* FUNCTIONS *******************/

void DeltaKinematics::calulate(double *xa, double *ya,double *za, double *xb, double *yb,double *zb, double B1angle,double B2angle,double B3angle)
{
	double B1x = Wb+sin(B1angle)*L;
	double B1y = 0;
	double B1z = cos(B1angle)*L;

	double B2x = (Wb+sin(B2angle)*L)*cos(2.09333333333);
	double B2y = (Wb+sin(B2angle)*L)*sin(2.09333333333);
	double B2z = cos(B2angle)*L;

	double B3x = (Wb+sin(B3angle)*L)*cos(4.18666666667);
	double B3y = (Wb+sin(B3angle)*L)*sin(4.18666666667);
	double B3z = cos(B3angle)*L;

    double r1 = l;
    double r2 = l;
    double r3 = l;


    double a11 = 2*(B3x-B1x);
    double a12 = 2*(B3y-B1y);
    double a13 = 2*(B3z-B1z);

    double a21 = 2*(B3x-B2x);
    double a22 = 2*(B3y-B2y);
    double a23 = 2*(B3z-B2z);

    double b1 = r1*r1-r3*r3-B1x*B1x-B1y*B1y-B1z*B1z+B3x*B3x+B3y*B3y+B3z*B3z;
    double b2 = r2*r2-r3*r3-B2x*B2x-B2y*B2y-B2z*B2z+B3x*B3x+B3y*B3y+B3z*B3z;


    *xa = B1x;
    *ya = B3y;
    *za = B3z;
return;

    double a1 = a11/a13-a21/a23;
    double a2 = a12/a13-a22/a23;
    double a3 = b2/a23-b1/a13;
    double a4 = -a2/a1;
    double a5 = -a3/a1;
    double a6 = (-a21*a4-a22)/a23;
    double a7 = (b2-a21*a5)/a23;

    double a = a4*a4+1+a6*a6;
    double b = 2*a4*(a5-B1x)-2*B1y+2*a6*(a7-B1z);
    double c = a5*(a5-2*B1x)+a7*(a7-2*B1z)+B1x*B1x+B1y*B1y+B1z*B1z-r1*r1;
    double w = b*b-4*a*c;

    *ya = (-b+sqrt(w))/(2*a);
    *yb = (-b-sqrt(w))/(2*a);

    *xa = a4*(*ya)+a5;
    *xb = a4*(*yb)+a5;

    *xa = a6*(*ya)+a7;
    *xb = a6*(*yb)+a7;
}

double DeltaKinematics::straightArms(double base,double platform,double armL1, double armL2)
{

  return 0;
}

