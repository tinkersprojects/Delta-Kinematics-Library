
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
        void setOffsets(double X, double Y, double Z);
        void setLimits(double upperX, double upperY, double upperZ, double lowerX, double lowerY, double lowerZ);
        
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

        double offsetX = 0;
        double offsetY = 0;
        double offsetZ = 0;
        double LimitsUpperB1 = 0;
        double LimitsUpperB2 = 0;
        double LimitsUpperB3 = 0;
        double LimitsLowerB1 = 3.14;
        double LimitsLowerB2 = 3.14;
        double LimitsLowerB3 = 3.14;
        
       
};

#endif 
