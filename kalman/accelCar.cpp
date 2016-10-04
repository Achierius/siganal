#include "kalmanFilter.h"
#include <string>
#include <iostream>
#include "./eigen/Eigen/Dense"
#include <stdlib.h> //rand
#include <time.h> //rand
#include <math.h> //erf
#include <float.h> //DBL_MIN 
#include <random> //normal_distribution
#include <chrono> //seed
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

double thisAirResistance = 0;

double dSgn(double d){
    double epsilon = pow(10, -5);
    if(d > epsilon){ return 1; }
    else if(d < -epsilon){ return -1; }
    else{ return 0; }
}
void updateSystem(double dT, double accel){
    unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
    std::default_random_engine generator(seed);
    double mean;
    if(abs(trueV) > 10){mean = pow(trueV,2)/25;}
    else{mean = abs(trueV/5);}
    normal_distribution<double> airResistance(mean, abs(trueV)/10);
    double sign = dSgn(trueV);
    trueX = trueX + trueV*dT + trueA*dT*dT/2;
    trueV = trueV + trueA*dT;
    thisAirResistance = abs(airResistance(generator))*sign;
    trueA = accel - thisAirResistance; //abs(airResistance(generator))*sign;
    thisAirResistance *= sign;
}
VectorXd measureSystem() 
{
    double dV = trueV - previousTrueV;
    previousTrueV = trueV;
    normal_distribution<double> sensorNoise(0, 0.5);
    std::default_random_engine generator;
    VectorXd ret(3); ret << trueX+3*sensorNoise(generator)*dT, trueV+2*sensorNoise(generator)*dT, trueA+sensorNoise(generator)*dT;
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
    stateVec << 0/*trueX*/, 0/*trueV/dT*/, trueA/dT; //Mantain proper rate vs. time

    VectorXd k_u(1); k_u << 0;
    MatrixXd k_B(3, 1); k_B << 0, 0, 0;

    MatrixXd k_P(3, 3); 
    k_P << 0.0000001, 0.000000, 0.000000,
           0.0000000, 0.000001, 0.000000,
	   0.0000000, 0.000000, 0.000001;
    MatrixXd k_Q(3,3);
    k_Q << 0.0000001, 0.000000, 0.000000,
           0.0000000, 0.000001, 0.000000,
	   0.0000000, 0.000000, 0.000100;
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
		std::cout<<"Actuals: Position: "<<RED<<trueX<<CLOSE<<" Velocity: "<<RED<<trueV<<CLOSE<<" Acceleration: "<<RED<<trueA<<CLOSE<<" ARes: "<<RED<<thisAirResistance<<CLOSE<<endl;
		std::cout<<endl;
	}
	if(abs(kalman.getCurrentEstimate()(0)-trueX)/trueX < 0.001){
	    stXn++;
	    if(stXn>=10){stX = i-stXn;}
	}
	else{stXn = 0;}
	if(abs(kalman.getCurrentEstimate()(1)-trueV)/trueV < 0.001){
	    stVn++;
	    if(stVn>=10){stV = i-stVn;}
	}
	else{stVn = 0;}
    }
	std::cout<<"Settling Time: X: "<<RED<<stX<<CLOSE<<" V: "<<RED<<stV<<CLOSE<<".\n";
}
	
