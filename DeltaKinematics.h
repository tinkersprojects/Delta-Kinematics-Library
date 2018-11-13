
#ifndef DeltaKinematics_h
#define DeltaKinematics_h
/*
#if ARDUINO >= 100
#include <Arduino.h>
#else
#include <WProgram.h>
#include <wiring.h>
#endif*/


class DeltaKinematics
{
    public:
        // SETUP
        DeltaKinematics::DeltaKinematics(double *x,double *y,double *z,double ArmLength,double RodLength,double TopTri,double BottomTri);

        // SET 
        void set(double a,double b,double c);
        void setLimits(double upperB1, double upperB2, double upperB3, double lowerB1, double lowerB2, double lowerB3);
        void setOffset(double x,double y,double z);

        // TEST
        bool test(double a,double b,double c);

        void calulate(double *xa, double *ya,double *za, double *xb, double *yb,double *zb, double B1angle,double B2angle,double B3angle);
        double straightArms(double base,double platform,double armL1, double armL2);
        
    private:
        double *X;
        double *Y;
        double *Z;

        double Sb;
        double Ub;
        double Wb;

        double Sp;
        double Up;
        double Wp;

        double L;
        double l;

        double limitUpperB1 = 0;
        double limitUpperB2 = 0;
        double limitUpperB3 = 0;
        double limitLowerB1 = 180;
        double limitLowerB2 = 180;
        double limitLowerB3 = 180;

        double offsetX = 0;
        double offsetY = 0;
        double offsetZ = 0;
        
};

#endif 
