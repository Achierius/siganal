#include "kalmanFilter.h"
#include <stdlib.h>

KalmanFilter::KalmanFilter(){
    MatrixXd nullMat(1); nullMat << 0;

    this->initialize(nullMat, nullMat, nullMat, nullmat, nullmat, nullMat, nullMat, 0.01);
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
void KalmanFilter::getTimeStep(){
    return timeStep;
}
void KalmanFilter::getTimeElapsed(){
    return timeElapsed;
}

VectorXd KalmanFilter::getCurrentEstimate(){
    return k_xcur;
}
MatrixXd KalmanFilter::getCurrentCovariance(){
    return k_Pcur;
}




