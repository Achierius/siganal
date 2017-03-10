#include "kalmanFilter.h"
#include <iostream>
#include <stdlib.h>

//#include <./eigen/Eigen/LU> //Inverse of Matrix


KalmanFilter::KalmanFilter(){
    MatrixXd nullMat(1,1); nullMat << 0;

    this->initialize(nullMat, nullMat, nullMat, nullMat, nullMat, nullMat, nullMat, 0.01);
}

KalmanFilter::KalmanFilter(VectorXd initialState, MatrixXd initialStateCV, MatrixXd stateTransition, MatrixXd sensorMap, MatrixXd controlMap, MatrixXd noiseCV, MatrixXd sensorCV, double timeStep){
    initialize(initialState, initialStateCV, stateTransition, sensorMap, controlMap, noiseCV, sensorCV, timeStep);
}

void KalmanFilter::initialize(VectorXd initialState, MatrixXd initialStateCV, MatrixXd stateTransition, MatrixXd sensorMap, MatrixXd controlMap, MatrixXd processNoiseCV, MatrixXd sensorNoiseCV, double timeStep){
    /*
     * TODO: Add checks to make sure the various matrices will play nicely.
     */

    this->k_xcur = initialState;
    this->k_xpre = initialState; //Unneccessary, but tidy. The Update function will override this with the value of k_xcur anyways, but I don't like leaving things undefined, y'now?
    this->k_Pcur = initialStateCV;
    this->k_Ppre = initialStateCV; //Same thing.
    this->k_F = stateTransition;
    this->k_H = sensorMap;
    this->k_B = controlMap;
    this->k_R = sensorNoiseCV;
    this->k_Q = processNoiseCV;
    this->timeStep = timeStep;
}

void KalmanFilter::setTimeStep(double newTimeStep){ //Seconds
    this->timeStep = newTimeStep == 0 ? 0.0000001 : abs(newTimeStep); //Lazy Safety check
}
double KalmanFilter::getTimeStep(){
    return timeStep;
}
double KalmanFilter::getTimeElapsed(){
    return timeElapsed;
}

VectorXd KalmanFilter::getCurrentEstimate(){
    return k_xcur;
}
MatrixXd KalmanFilter::getCurrentCovariance(){
    return k_Pcur;
}


//y'all
void KalmanFilter::updateFilter(VectorXd sensorInput, VectorXd controlInput){
    timeElapsed += timeStep;

    k_u = controlInput;
    k_z = sensorInput;

    k_xpre = k_xcur;
    k_Ppre = k_Pcur;

    /*
     * Prediction Equations
     */
    k_xcur = k_F*k_xpre + k_B*k_u;
    k_Pcur = (k_F*k_Ppre)*(k_F.transpose()) + k_Q;

    /*
     * Update Equations
     */
    k_S = k_H*k_Ppre*(k_H.transpose())+k_R;
    k_K = k_Ppre*(k_H.transpose())*k_S.inverse();
    k_xcur = k_xcur + k_K*(k_z-k_H*k_xcur);
    k_Pcur = k_Pcur - k_K*k_H*k_Pcur;
}
