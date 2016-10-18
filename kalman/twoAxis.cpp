#include "./eigen/Eigen/Dense"
#include <iostream>
#include "genSys.h"
#include "kalmanFilter.h"
#include <string>
#include <math.h>

using namespace std;
using namespace Eigen;


double dSgn(double d){ //Returns sign number for double
    double epsilon = pow(10, -5); //Arbitary small number
    if(d > epsilon){ return 1; }
    else if(d < -epsilon){ return -1; }
    else{ return 0; }
}
VectorXd controlInput(double time){
    VectorXd ret(2); ret << 0, 0;
    return ret;
}
VectorXd airMovement(VectorXd state, VectorXd statePrev, double dT, double mass){
    double dragCoefficient = -1/5;
    double xAir = abs(state(0)) > 10 ? pow((state(0)*dragCoefficient),2)*dSgn(state(0)) : state(0)*dragCoefficient;
    double yAir = abs(state(1)) > 10 ? pow((state(1)*dragCoefficient),2)*dSgn(state(1)) : state(1)*dragCoefficient;
    
    double aC = dT*dT/(2*mass); double vC = dT/mass;
    VectorXd ret(4); ret << aC*xAir, vC*xAir, aC*yAir, vC*yAir;

    return ret;
}

int main(){
    double dT = 0.01;
    double mu = pow(10, -6); //Coefficient for the covariance matrices
    double mass = 1;
    
    /*
    * State Space Matrices
    */
    VectorXd initialState(4); initialState << 0, 10, 0, 10;
    MatrixXd trueTransition(4, 4); trueTransition << 1, dT, 0, 0,
						     0, 1,  0, 0,
						     0, 0,  1, dT,
						     0, 0,  0, 1;
    double aC = dT*dT / (2*mass); //Acceleration coefficient; F/m * dT^2 / 2
    double vC = dT / mass; //Velocity coefficient
    MatrixXd trueControlMap(4, 2); trueControlMap << aC, 0, 
						     vC, 0,
						     0, aC,
						     0, vC;
    MatrixXd trueSensorMap(2, 4); trueSensorMap << 1, 0, 0, 0,
						   0, 0, 1, 0;
    VectorXd sensorNoiseSd(2); sensorNoiseSd << 3*mu, 3*mu;//, 3*mu, 3*mu;
    VectorXd processNoiseSd(4); processNoiseSd << 5*mu, 5*mu, 5*mu, 5*mu;

    GenSys system(initialState, trueTransition, processNoiseSd, trueControlMap, trueSensorMap, sensorNoiseSd, &airMovement, dT, mass);

    double timeLimit = 10;
    double interval = 0.5;
    for(int i = 0; i < timeLimit/dT; i++){
        system.updateFilter(controlInput(i*dT));
	if(i%((int)(0.5/dT)) == 0){
	    cout<<"Position: "<<system.getMeasurement()(0)<<", "<<system.getMeasurement()(1)<<".\n";
	//    cout<<"Velocity: "<<system.getMeasurement()(1)<<", "<<system.getMeasurement()(4)<<".\n";
	}
    }
    cout<<"Position: "<<system.getMeasurement()(0)<<", "<<system.getMeasurement()(1)<<".\n";
    cout<<system.getMeasurement()<<endl;
    //cout<<"Velocity: "<<system.getMeasurement()(1)<<", "<<system.getMeasurement()(4)<<".\n";
}
