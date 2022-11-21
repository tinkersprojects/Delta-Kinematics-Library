
#ifndef DeltaKinematics_h
#define DeltaKinematics_h

#define sqrt3 1.7320508075688772935274463415059
#define pi  3.1415926535897932384626433832795     // PI
#define sin120  sqrt3/2.0   
#define cos120  -0.5        
#define tan60  sqrt3
#define sin30  0.5
#define tan30  1.0/sqrt3

#define non_existing_povar_error -2
#define no_error 1

class DeltaKinematics
{
    public:
        // SETUP 
        DeltaKinematics(double _ArmLength,double _RodLength,double _BassTri,double _PlatformTri);
        
        int forward();
        int forward(double thetaA, double thetaB, double thetaC);
        int inverse();
        int inverse(double x0, double y0, double z0);

        double x;
        double y;
        double z;

        double a;
        double b;
        double c;
        
    private:
    
        int delta_calcAngleYZ(double *Angle, double x0, double y0, double z0);

        double ArmLength;
        double RodLength;
        double BassTri;
        double PlatformTri;
       
};

#endif 



