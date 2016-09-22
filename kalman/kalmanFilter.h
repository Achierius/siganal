#ifndef __SIGANAL_KALMAN_FILTER_H
#define __SIGANAL_KALMAN_FILTER_H

#include "./eigen/Eigen/Dense"

class KalmanFilter{
    using Eigen::VectorXd;
    using Eigen::MatrixXd;

public:
    KalmanFilter();
    KalmanFilter(VectorXd initialState, MatrixXd initialStateCV, MatrixXd stateTransition, MatrixXd sensorMap, MatrixXd controlMap, MatrixXd noiseCV, MatrixXd sensorCV, double timeStep);

    void initialize(VectorXd initialState, MatrixXd initialStateCV, MatrixXd stateTransition, MatrixXd sensorMap, MatrixXd controlMap, MatrixXd processNoiseCV, MatrixXd sensorNoiseCV, double timeStep);

    /*void setQ(MatrixXd newProcessNoiseCV);
    void setR(MatrixXd newSensorNoiseCV);
    void setF(MatrixXd newF);
    void setB(MatrixXd newB);
    void setH(MatrixXd newH);*/

    void setTimeStep(double seconds);
    double getTimeStep();
    double getTimeElapsed();

    void updateFilter(VectorXd sensorInput, VectorXd controlInput);

    VectorXd getCurrentEstimate();
    MatrixXd getCurrentCovariance();
private:
    MatrixXd k_K; //OPTIMAL Kalman Gain

    MatrixXd k_F; //State Transition Matrix
    MatrixXd k_B; //Control Input Translation Matrix
    MatrixXd k_H; //Observation Space Translation Matrix

    MatrixXd k_Pcur; //Current Covariance of the State Estimate
    MatrixXd k_Ppre; //Previous ''
    MatrixXd k_R; //Covariance of the Observation Noise
    MatrixXd k_Q; //Covariance of the Process Noise

    VectorXd k_xcur; //Current State Estimate
    VectorXd k_xpre; //Previous State Estimate
    VectorXd k_u; //Current control input
    VectorXd k_z; //Current Observation


    double timeStep;
    double timeElapsed;
};

#endif//__SIGANAL_KALMAN_FILTER_H
