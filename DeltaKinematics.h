
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
        void DeltaKinematics::set(double a,double b,double c);
        
    private:
        
};

#endif 
