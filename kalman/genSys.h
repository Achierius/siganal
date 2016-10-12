#ifndef __SIGANAL_GENSYS_H
#define __SIGANAL_GENSYS_H

#include "./eigen/Eigen/Dense"

using namespace Eigen;

class GenSys{

public:
    GenSys(VectorXd initState, MatrixXd stateTran, VectorXd sNoise, MatrixXd controlTran, MatrixXd measTran, VectorXd mNoise, VectorXd (*nonLinCom)(VectorXd, VectorXd, double, double), double DT);

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

    VectorXd (*nonlinearStateComponent)(VectorXd, VectorXd, double, double); //State, previous state, dT, mass

    MatrixXd controlTranslation;
    VectorXd control;

    MatrixXd measurementTranslation;
    VectorXd measurement;
    VectorXd mNoiseSD;

    double dT;
};

#endif//__SIGANAL_GENSYS_H
