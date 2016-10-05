#ifndef __SIGANAL_GENSYS_H
#define __SIGANAL_GENSYS_H

#include "./eigen/Eigen/Dense"

class GenSys{
using namespace Eigen;

public:
    GenSys(VectorXd initState, stateTran, sNoise, controlTran, measTran, VectorXd (*nonLinCom)(VectorXd, VectorXd));

    double genGauss(double mean, double sd);

    void setState(VectorXd s);
    void setStateTransition(MatrixXd sT);

    void setSNoiseSD(VectorXd q);
    void setMNoiseSD(VectorXd r);

    void setControlTranslation(MatrixXd b);
    void setMeasurementTranslation(MatrixXd h); 

    VectorXd getState();
    VectorXd getMeasurement();

    void setdT(double dT);

    void updateFilter(VectorXd controlInput);
private:
    MatrixXd stateTransition;
    VectorXd state;
    VectorXd statePrev; //In case the nonlinear function component needs this
    unsigned int states;
    VectorXd sNoiseSD;

    VectorXd (*nonlinearStateComponent)(VectorXd, VectorXd); //State, previous state

    MatrixXd controlTranslation;
    VectorXd control;

    MatrixXd measurementTranslation;
    VectorXd measurement;
    VectorXd mNoiseSD;

    double dT;
};

#endif//__SIGANAL_GENSYS_H
