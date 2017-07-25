#include "../sim/genSim.hpp"
#include <iostream>
#include <ctime>
#include <cmath>
#include <eigen/Eigen/Dense>
#include <random>

class AccelTrain : GenSim {
  public:
    AccelTrain(double initState, std::chrono::milliseconds dT);

  private:
    state _measurement(state input, std::chrono::milliseconds dt) override;
    state _transition(state input, std::chrono::milliseconds dt) override;
    
    state _pNoise(state state) override;
    state _mNoise(state state) override;

    std::default_random_engine _generator;
    std::normal_distribution<double> _noise;

    Eigen::MatrixXd _transition;

    //Form of State:
    /*   x
    /*   x'
     */  x''
}
