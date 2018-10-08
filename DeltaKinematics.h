
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
        DeltaKinematics(byte pinR,byte pinG,byte pinB);
        DeltaKinematics(bool inverted, byte pinR,byte pinG,byte pinB);
        void setCallback(float (*CallBack)(float x));

        // SET 
        void set(byte Rvalue,byte Gvalue,byte Bvalue);
        
    private:
        byte R_Pin;
        byte G_Pin;
        byte B_Pin;
        byte R_Last_value;
        byte G_Last_value;
        byte B_Last_value;
        byte R_Current_value;
        byte G_Current_value;
        byte B_Current_value;
        byte R_Future_value;
        byte G_Future_value;
        byte B_Future_value;

        unsigned long Speed = 2000;
        unsigned long starting_time;
        int function=0;
        int count;
        bool invertedPins=false;
        
        float (*FadeFunctionCallBack) (float x);

        void writeOutput();
        void FunctionsFinished();
        void FadeFunction();
        void FadeRandomFunction();
        void StepRGBWFunction();
        void StepAllFunction();
        void StepRandomFunction();
};

#endif 
