#include "genSys.h"
#include <random>
#include <chrono>

GenSys::GenSys(VectorXd initState, stateTran, sNoise, controlTran, measTran, VectorXd (*nonLinCom)(VectorXd, VectorXd), double DT){
    setState(initState);
    statePrev = initState;
    setStateTransition(stateTran);
    setSNoiseSD(sNoise);
    setControlTranslation(controlTran);
    setMeasurementTranslation(measTran);
    nonlinearStateComponent = nonLinCom;
    setdT(DT);
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
}
