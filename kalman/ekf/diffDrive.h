#ifndef __DIFF_DRIVE_CPP
#define __DIFF_DRIVE_CPP

#include "./eigen/Eigen/Dense"

/*
 * Namespaces/Classes: Lowercase, Underscore separated
 * Functions/Variables: lowerCamelCase, Compacted
 * Private Member Variables: Underscore prefixed, lowerCamelCase
 */ 

namespace y2017{
namespace akf_sim{
 class differential_drive{
   public:
     differential_drive(const double dT, const double length,
                        const double kV, const double kT, const double mR, const double mJ,
                        const double mass = 0, const double momentOfInertia = 0);

     void setInputs(const double leftVoltage, const double rightVoltage);
     void driveStraight(const double voltage);
     void quickTurn(const double voltage); //Assigned to left motor; positive turns clockwise, negative turns counterclockwise.

     void stepDrive();
     
     enum states{
       xPos,
       yPos,
       theta,
       omega,
       radius,
       Sl,
       Sr,
       Stot
     };

   private: 
     double _dT; //Milliseconds

     /* Drive Characteristics */
     double _chassisLength; //Distance between left wheel and right wheel across the axle
     double _mass; //UNUSED; Total mass of robot, Centre of Gravity assumed to be at the midpoint of the axle
     double _J; //UNUSED; Moment of inertia of the robot around the centre axle

     /* Motor Characteristics */
     //We assume both the left and the right motors are identical; if they are put through a gearbox, account for that in kV and kT :^)
     double _kV; //Motor Voltage Constant
     double _kT; //Motor Torque Constant
     double _mR; //Motor internal electrical resistance
     double _mJ; //Motor driven wheel moment of Inertia

     /* Dynamic System Information */
     Eigen::Vector2d _inputs; //Left Motor Voltage, Right Motor Voltage
     Eigen::Vector8d _state; //X, Y, θ, ω, Radius, Speed of Left Wheel, Speed of Right Wheel, Speed Total Forward
     Eigen::Vector8d _statePrev;
     double _leftWheelDistance;
     double _rightWheelDistance; //Continously integrated; not a state variable, so not included in _state, but used in simulating Encoder readings.

     /* State Transition Functions */
     void updateAbsolutePosition();
     void updateAngularStatistics();
     void updateForwardVelocities();
 
 }
}
}

#endif//__DIFF_DRIVE_CPP
