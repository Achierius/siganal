#ifndef __ACCEL_TRAIN_HPP
#define __ACCEL_TRAIN_HPP

#include "../sim/genSim.hpp"
#include <iostream>
#include <ctime>
#include <cmath>
#include <eigen/Eigen/Dense>
#include <random>

/*
 * Simple physical system based off of the GenSim framework
 * Model: Cart on single-dimensioned track. Posesses position and velocity. Physical noise affects velocity. No internal system acceleration. Input: acceleration.
 * Mass not needed. All units in feet/seconds derivatives. 
 * Sensor reading: encoder on wheel, 360 degrees per foot.
 * @author Marcus Plutowski <achierius@gmail.com>
 */

class AccelTrain : public GenSim {
  public:
    AccelTrain(double initState);

    static Eigen::VectorXd unpare(double input); //Returns a 3-vector containing the given double in the initial cell and 0's in the following 2
 
  private:
    state _measurement(state input, std::chrono::milliseconds dt) override;
    state _transition(state input, std::chrono::milliseconds dt) override;
    
    state _pNoise(GenSim::state currentState) override;
    state _mNoise(GenSim::state measurement, GenSim::state currentState) override;

    std::default_random_engine _generatorM;
    std::normal_distribution<double> _mNoiseD;
    
    std::default_random_engine _generatorP;
    std::normal_distribution<double> _pNoiseD;

    Eigen::MatrixXd _transitionMat;

    double _input;

    //Form of State:
    /*   x    -- feet
     *   x'   -- feet/second
     *   x''  -- feet/second^2
     */
};

#endif//__ACCEL_TRAIN_HPP
