
#ifndef DeltaInverseKinematics_h
#define DeltaInverseKinematics_h
/*
#if ARDUINO >= 100
#include <Arduino.h>
#else
#include <WProgram.h>
#include <wiring.h>
#endif*/


class DeltaInverseKinematics
{
    public:
        // SETUP 
        DeltaInverseKinematics(double *B1temp,double *B2temp,double *B3temp,double ArmLength,double RodLength,double BassTri,double PlatformTri);
        
        // SET 
        void set(double x,double y,double z);
        void setOffsets(double upperX, double upperY, double upperZ, double lowerX, double lowerY, double lowerZ);
        
        // TEST
        bool test(double x,double y,double z);
        
        void calulate(double x,double y,double z, double *B1a,double *B2a,double *B3a,double *B1b,double *B2b,double *B3b);
        double straightArms(double base,double platform,double armL1, double armL2);

        
    private:
        double *B1angle;
        double *B2angle;
        double *B3angle;

        double a;
        double b;
        double c;

        double Sb;
        double Ub;
        double Wb;

        double Sp;
        double Up;
        double Wp;

        double L;
        double l;

        double offsetUpperB1 = 0;
        double offsetUpperB2 = 0;
        double offsetUpperB3 = 0;
        double offsetLowerB1 = 180;
        double offsetLowerB2 = 180;
        double offsetLowerB3 = 180;
        
       
};

#endif 
