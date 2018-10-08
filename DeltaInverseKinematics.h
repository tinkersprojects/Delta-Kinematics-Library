
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
        DeltaInverseKinematics(double *B1,double *B2,double *B3,double ArmLength,double RodLength,double BassTri,double PlatformTri)

        // SET 
        void set(double x,double y,double z);
        
        
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
        
};

#endif 
