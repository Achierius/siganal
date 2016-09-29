#include "kalmanFilter.h"
#include <string>
#include <iostream>
#include "./eigen/Eigen/Dense"
#include <stdlib.h> //rand
#include <time.h> //rand

/*
 * Just a flywheel spinning at some random speed, blank test for kalman.
 */
double trueX = 0;
double trueV = 10;
double trueA = 0;
//Assume m = 1kg, accel. = force

double dT = 0.01;
double timeElapsed = 0;

using namespace std;

string RED= "\033[31m";
string CLOSE = "\e[0m";

double erf(double x){ //Error Function!
    return 1-1/(pow((1+0.278393*pow(x,1)+0.230389*pow(x,2)+0.000972*pow(x,3)+0.078108*pow(x,4)),4));
}
double cdf(double x, double mean, double stdev){
    return 0.5*(1+erf((x-mean)/(1.41421356*stdev)));
}
double lazyGauss(double mean, double stdev){ //Whooo cental limit theorem
    srand(time(NULL));
    /*double destination = (rand()%99+1)/100;
    if(destination > 0.5){
	for(double i = 0; true; i+=0.005){
	    if(abs(cdf(i, mean, stdev)-destination)<0.01){return i;}
	}
    }
    else if(destination < 0.5){
	for(double i = 0; true; i-=0.005){
	    if(abs(cdf(i, mean, stdev)-destination)<0.01){return i;}
	}
    }
    else if(destination = 0.5){
	return 0;
    }*/    
    return mean;
}
void updateSystem(double dT, double accel){
    trueX = trueX + trueV*dT + trueA*dT*dT/2 + lazyGauss(0, 0.75)*dT;
    trueV = trueV + trueA*dT + lazyGauss(0, 1.50)*dT - lazyGauss(0-pow(trueV,2)/100, trueV/10)*dT;
    trueA = accel;
}
VectorXd measureSystem() 
{
    VectorXd ret(3); ret << 0, 0, trueA+lazyGauss(0, 1);
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
   
	cout<<"Pre-Kalman\n";
    KalmanFilter kalman(stateVec, k_P, stateTransition, k_H, k_B, k_Q, k_R, dT);
	cout<<"Post-Kalman\n";
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
	cout<<"StartLoop\n";
        k_u << returnInput(timeElapsed);
	cout<<"ReturnInput\n";
        updateSystem(dT, k_u(0));
	cout<<"UpdatedSystem\n";
        kalman.updateFilter(measureSystem(), k_u);
	cout<<"UpdatedFilter\n";
	timeElapsed+=dT;
	std::cout<<"Time: "<<i*dT<<endl;
	std::cout<<"Estimates: Position: "<<RED<<kalman.getCurrentEstimate()(0)<<CLOSE<<" Velocity: "<<RED<<kalman.getCurrentEstimate()(1)<<CLOSE<<" Acceleration: "<<RED<<kalman.getCurrentEstimate()(2)<<CLOSE<<endl;
	std::cout<<"Actuals: Position: "<<RED<<trueX<<CLOSE<<" Velocity: "<<RED<<trueV<<CLOSE<<" Acceleration: "<<RED<<trueA<<CLOSE<<endl;
	std::cout<<endl;
    }
}
	
