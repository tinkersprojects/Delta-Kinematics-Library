/**********************************************************************************************
 * Arduino Delta Kinematics- Version 1.0
 * by William Bailes <williambailes@gmail.com> http://tinkersprojects.com/
 *
 * This Library is licensed under a GPLv3 License
 * 
 * made with infomation from the following documents
 * - R.L. Williams II, “The Delta Parallel Robot: Kinematics Solutions”, Internet Publication, www.ohio.edu/people/williar4/html/pdf/DeltaKin.pdf, January 2016. 
 **********************************************************************************************/

#include "DeltaInverseKinematics.h"

/******************* SETUP *******************/

DeltaInverseKinematics::DeltaInverseKinematics(double *B1,double *B2,double *B3,double ArmLength,double RodLength,double BassTri,double PlatformTri)
{
  B1angle = B1;
  B2angle = B2;
  B3angle = B3;

  L = ArmLength;
  l = RodLength;

  Sb = BassTri;
  Ub = sqrt(3)/3*Sb;
  Wb = sqrt(3)/6*Sb;

  Sp = PlatformTri;
  Up = sqrt(3)/6*Sp;
  Wp = sqrt(3)/3*Sp;

  a = Wb - Up;
  b = Sp/2-sqrt(3)/2*Wb;
  c = Wp-Wb/2;
}

/******************* SET *******************/

void DeltaInverseKinematics::set(double x,double y,double z)
{
    double E1 = 2*L*(y+a);
    double F1 = 2*z*L;
    double G1 = x*x+y*y+z*z+a*a+L*L+2*y*a-l*l;

    double E2 = -L*(sqrt(3)*(x+b)+y+c);
    double F2 = 2*z*L;
    double G2 = x*x+y*y+z*z+b*b+c*c+L*L+2*(x*b+y*c)-l*l;

    double E3 = L*(sqrt(3)*(x-b)-y-c);
    double F3 = 2*z*L;
    double G3 = x*x+y*y+z*z+b*b+c*c+L*L+2*(-x*b+y*c)-l*l;

    double T1 = E1*E1+F1+F1-G1+G1;
    double TH1a = 2*atan2((-F1+sqrt(T1)),(G1-E1));
    double TH1b = 2*atan2((-F1-sqrt(T1)),(G1-E1));

    double T2 = E2*E2+F2+F2-G2+G2;
    double TH2a = 2*atan2((-F2+sqrt(T2)),(G2-E2));
    double TH2b = 2*atan2((-F2-sqrt(T2)),(G2-E2));

    double T3 = E3*E3+F3+F3-G3+G3;
    double TH3a = 2*atan2(-F3+sqrt(T3)),(G3-E3));
    double TH3b = 2*atan2((-F3-sqrt(T3)),(G3-E3));
}
