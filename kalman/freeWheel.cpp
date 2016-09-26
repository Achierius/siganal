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
VectorXd measureSystem() //We have an encoder~
{
    VectorXd ret(2); ret << trueTheta + lazyGauss(0, 0.1), 0; //At most 1 tenth a degree of error
    return ret;
}
int main(){
    double dT = 0.01;
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
    k_Q << 0.000001, 0,
           0, 0.000001;
    MatrixXd k_R(2,2);
    k_R << 0.000001, 0,
           0, 0.000001;
    
    MatrixXd k_H(2, 2); k_H << 1, 0,
                                0, 0;
   

    KalmanFilter kalman(stateVec, k_P, stateTransition, k_H, k_B, k_Q, k_R, dT);


    char check = '9';

    while(check != '0'){
        updateSystem(dT);
        kalman.updateFilter(measureSystem(), k_u);
        cout<<"System: Position "<<trueTheta<<", Velocity "<<trueW<<".\n";
        cout<<"Kalman: Position "<<kalman.getCurrentEstimate()(0)<<", Velocity "<<kalman.getCurrentEstimate()(0)<<".\n";
//        cin>>check;
    }
}
