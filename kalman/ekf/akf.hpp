#ifndef __MASS_AKF_HPP
#define __MASS_AKF_HPP

#include <iostream>
#include <cmath>
#include <ctime>
#include <cstdio>
#include "./eigen/Eigen/Dense"

namespace akf_mass{

  class akf{
    public:
    private:
      int _states;
      int _measurements;

      Eigen::VectorXd _state;
      Eigen::VectorXd _prevState;

      Eigen::MatrixXd _stateCovariance;
      Eigen::MatrixXd _prevStateCovariance;

      Eigen::MatrixXd _sensorCovariance;
      Eigen::MatrixXd _processCovariance;

      Eigen::MatrixXd _K;
      Eigen::MatrixXd _H;
      Eigen::MatrixXd _



  }

}

#endif//__MASS_AKF_HPP
