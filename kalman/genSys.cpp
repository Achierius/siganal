#include "genSys.h"
#include <random>
#include <chrono>
#include <iostream>

GenSys::GenSys(VectorXd initState, MatrixXd stateTran, VectorXd sNoise, MatrixXd controlTran, MatrixXd measTran, VectorXd mNoise,  VectorXd (*nonLinCom)(VectorXd, VectorXd, double, double), double dT, double mass){
    setState(initState);
    statePrev = initState;
    setStateTransition(stateTran);
    setSNoiseSD(sNoise);
    setMNoiseSD(mNoise);
    setControlTranslation(controlTran);
    setMeasurementTranslation(measTran);
    nonlinearStateComponent = nonLinCom;
    setdT(dT);
    setMass(mass);
}
void GenSys::setMass(double mass){
    mass = mass;
}
double GenSys::genGauss(double mean, double sd){
    unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
    std::default_random_engine generator(seed);
    std::normal_distribution<double> distribution (mean, sd);
    return distribution(generator);
}
void GenSys::setState(VectorXd s){
    state = s;
    states = s.rows();
}
void GenSys::setStateTransition(MatrixXd sT){
    stateTransition = sT;
}

void GenSys::setSNoiseSD(VectorXd q){
    sNoiseSD = q;
}
void GenSys::setMNoiseSD(VectorXd r){
    mNoiseSD = r;
}

void GenSys::setControlTranslation(MatrixXd b){
    controlTranslation = b;
}
void GenSys::setMeasurementTranslation(MatrixXd h){
    measurementTranslation = h;
} 

VectorXd GenSys::getState(){
    return state;
}
VectorXd GenSys::getMeasurement(){
    return measurement;
}

void GenSys::setdT(double dT){
    this->dT = dT;
}

void GenSys::updateFilter(VectorXd controlInput){
    statePrev = state;
    state = stateTransition*state + controlTranslation*controlInput + nonlinearStateComponent(state, statePrev, dT, mass);
    VectorXd noise(states);
    for(int i = 0; i < states; i++){
        noise[i] = genGauss(0, sNoiseSD(i));
    }
    state += noise;

    measurement = measurementTranslation*state;
    for(int i = 0; i < states; i++){
        noise[i] = genGauss(0, mNoiseSD(i));
    }
    measurement += noise;
}
