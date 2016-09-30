#include "kalmanFilter.h"
#include <string>
#include <iostream>
#include "./eigen/Eigen/Dense"
#include <stdlib.h> //rand
#include <time.h> //rand
#include <math.h> //erf
#include <float.h> //DBL_MIN 
#include <random>
/*
 * Just a flywheel spinning at some random speed, blank test for kalman.
 */
double trueX = 0;
double trueV = 0;
double trueA = 0;
//Assume m = 1kg, accel. = force

double dT = 0.01;
double timeElapsed = 0;

using namespace std;

string RED= "\033[31m";
string CLOSE = "\e[0m";

double previousTrueV = 0;

void updateSystem(double dT, double accel){
    std::default_random_engine generator;
    normal_distribution<double> airResistance(abs(trueV)>10 ? pow(trueV,2)/25 : abs(trueV/5), abs(trueV)/10);

    trueX = trueX + trueV*dT + trueA*dT*dT/2;
    trueV = trueV + trueA*dT - abs(airResistance(generator)*dT)*abs(trueV)/(trueV+DBL_MIN);
    trueA = accel; //Not actually the full acceleration; JUST that provided by the input. AirResistance provides a dV we don't measure.
}
VectorXd measureSystem() 
{
    double dV = trueV - previousTrueV;
    previousTrueV = trueV;
    normal_distribution<double> sensorNoise(0, 0.3);
    std::default_random_engine generator;
    VectorXd ret(3); ret << 0, 0, dV/dT+sensorNoise(generator)*dT; //We use kinematics since trueA =/= actual acceleration
    return ret;
}
double returnInput(double timeElapsed){
    return timeElapsed < 5 ? 3 : 0;
}
int main(){

    MatrixXd stateTransition(3,3);
    stateTransition << 1, dT, dT*dT/2,
                       0,  1, dT,
		       0,  0, 1;
    VectorXd stateVec(3);
    stateVec << trueX, trueV/dT, trueA/dT; //Mantain proper rate vs. time

    VectorXd k_u(1); k_u << 0;
    MatrixXd k_B(3, 1); k_B << 0, 0, 0;

    MatrixXd k_P(3, 3); 
    k_P << 0.0000001, 0.000000, 0.000000,
           0.0000000, 0.000001, 0.000000,
	   0.0000000, 0.000000, 0.000001;
    MatrixXd k_Q(3,3);
    k_Q << 0.0000001, 0.000000, 0.000000,
           0.0000000, 0.000001, 0.000000,
	   0.0000000, 0.000000, 0.000001;
    MatrixXd k_R(3,3);
    k_R << 0.0000001, 0.000000, 0.000000,
           0.0000000, 0.000001, 0.000000,
	   0.0000000, 0.000000, 0.000001;
    
    MatrixXd k_H(3, 3); k_H << 0, 0, 0,
                               0, 0, 0,
			       0, 0, 1;
   
    KalmanFilter kalman(stateVec, k_P, stateTransition, k_H, k_B, k_Q, k_R, dT);

    double errorX = 1;
    double errorV = 1;
    int i; 
    int stX = 0;
    int stXn = 0;
    int stV = 0;
    int stVn = 0;
    int stA = 0;
    int stAn = 0;

    for(i = 0; stX == 0 || stV == 0; i++){
        k_u << returnInput(timeElapsed);
        updateSystem(dT, k_u(0));
        kalman.updateFilter(measureSystem(), k_u);
	timeElapsed+=dT;
	if(i%25 == 0){
		std::cout<<"Time: "<<i*dT<<endl;
		std::cout<<"Estimates: Position: "<<RED<<kalman.getCurrentEstimate()(0)<<CLOSE<<" Velocity: "<<RED<<kalman.getCurrentEstimate()(1)<<CLOSE<<" Acceleration: "<<RED<<kalman.getCurrentEstimate()(2)<<CLOSE<<endl;
		std::cout<<"Actuals: Position: "<<RED<<trueX<<CLOSE<<" Velocity: "<<RED<<trueV<<CLOSE<<" Acceleration: "<<RED<<trueA<<CLOSE<<endl;
		std::cout<<endl;
	}
	if(i==3000){break;}
    }
}
	
