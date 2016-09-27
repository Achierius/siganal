#include "kalmanFilter.h"
#include <string>
#include <iostream>
#include "./eigen/Eigen/Dense"
#include <stdlib.h> //rand
#include <time.h> //rand

/*
 * Just a flywheel spinning at some random speed, blank test for kalman.
 */
double trueTheta = 0;
double trueW = 10;
double dT = 0.01;

using namespace std;

string RED= "\033[31m";
string CLOSE = "\e[0m";

double lazyGauss(double mean, double max){ //Whooo cental limit theorem
    srand(time(NULL));
    double accum = 0;
    for(int i = 0; i < 100; i++){
        accum += rand() % 10 * (rand() % 2 -1);
    }
    accum /= 1000; //Normalize to -1 : 1
    accum *= max;
    accum += mean;

    return accum;
}
void updateSystem(double dT){
    trueW = trueW += lazyGauss(0, 0.5*dT); //Speed randomly varies by at most half a degree per second
    trueTheta = (trueTheta + dT*trueW + lazyGauss(0, 1*dT)); //More variance in the position whoo
}
double oldTheta = 0;
VectorXd measureSystem(bool measureWAccurately = false) //We have an encoder~
{
    VectorXd ret(2); ret << trueTheta + lazyGauss(0, 0.1), 0; //At most 1 tenth a degree of error
    if(measureWAccurately){ret(1) = trueW + lazyGauss(0, 0.1);}
    else{ret(1) = (ret(0)-oldTheta)/dT;}
    oldTheta = ret(0);
    return ret;
}
int main(){
    double w = 10; //Degrees / sec
    double theta = 0;

    MatrixXd stateTransition(2,2);
    stateTransition << 1, dT,
                       0, 1;
    VectorXd stateVec(2);
    stateVec << theta, w/dT; //Mantain proper rate vs. time

    VectorXd k_u(1); k_u << 0; //No input acceleration
    MatrixXd k_B(2, 1); k_B << 0, 0;

    MatrixXd k_P(2, 2); 
    k_P << 0.000001, 0,
           0, 0.000001;
    MatrixXd k_Q(2,2);
    k_Q << 0.0000005, 0.000001,
           0.000001, 0.00005;
    MatrixXd k_R(2,2);
    k_R << 0.000001, 0,
           0, 0.00001;
    
    MatrixXd k_H(2, 2); k_H << 1, 0,
                                0, 1;
   

    KalmanFilter kalman(stateVec, k_P, stateTransition, k_H, k_B, k_Q, k_R, dT);

    double errorX = 1;
    double errorV = 1;
    int i; 
    int stX = 0;
    int stXn = 0;
    int stV = 0;
    int stVn = 0;
    for(i = 0; stX == 0 || stV == 0; i++){
        updateSystem(dT);
        kalman.updateFilter(measureSystem(false), k_u);
        cout<<"System: Position "<<RED<<trueTheta<<CLOSE<<", Velocity "<<RED<<trueW<<CLOSE<<".\n";
        cout<<"Kalman: Position "<<RED<<kalman.getCurrentEstimate()(0)<<CLOSE<<", Velocity "<<RED<<kalman.getCurrentEstimate()(1)<<CLOSE<<".\n";
        cout<<"Percent Error Position: "<<RED<<(kalman.getCurrentEstimate()(0)-trueTheta)/trueTheta<<CLOSE<<", Velocity: "<<RED<<(kalman.getCurrentEstimate()(1)-trueW)/trueW<<CLOSE<<".\n\n";
	errorX = (kalman.getCurrentEstimate()(0)-trueTheta)/trueTheta;
	errorV = (kalman.getCurrentEstimate()(1)-trueW)/trueW;
	if(abs(errorX)<0.1 && stX==0){stXn++;} else{stXn=0;} if(stXn >= 5){stX=i-stXn;}
	if(abs(errorV)<0.1 && stV==0){stVn++;} else{stVn=0;} if(stVn >= 5){stV=i-stVn;}
    }
    cout<<"Settling time V: "<<RED<<stV*dT<<CLOSE<<" X: "<<RED<<stX*dT<<CLOSE<<".\n";
    cout<<"Covariance of the Process Noise: "<<RED<<endl<<k_Q<<CLOSE<<"\n";
    cout<<"Covariance of the Sensor Noise: "<<RED<<endl<<k_R<<CLOSE<<"\n";
}
