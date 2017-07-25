#ifndef __ACCEL_TRAIN_HPP
#define __ACCEL_TRAIN_HPP

#include "../sim/genSim.hpp"
#include <iostream>
#include <ctime>
#include <cmath>
#include <eigen/Eigen/Dense>
#include <random>

class AccelTrain : GenSim {
  public:
    AccelTrain(double initState, std::chrono::milliseconds dT);
    
    void resetDT(std::chrono::milliseconds newDT);

    void setInput(double input);

    static Eigen::VectorXd unpare(double input); //Returns a 3-vector containing the given double in the initial cell and 0's in the following 2

  private:
    state _measurement(state input, std::chrono::milliseconds dt) override;
    state _transition(state input, std::chrono::milliseconds dt) override;
    
    state _pNoise(state state) override;
    state _mNoise(state state) override;

    std::default_random_engine _generator;
    std::normal_distribution<double> _noise;

    void _fixTransition();
    Eigen::MatrixXd _transition;

    double _input;

    //Form of State:
    /*   x
    /*   x'
     */  x''
}

#endif//__ACCEL_TRAIN_HPP
