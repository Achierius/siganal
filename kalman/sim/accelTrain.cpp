#include "accelTrain.hpp"

AccelTrain::AccelTrain(double initState) : GenSim::GenSim(3, 1, 1, std::chrono::milliseconds(1), AccelTrain::unpare(initState)) {

  double dT = 0.001;                //Seconds per millisecond
  _transitionMat = Eigen::Matrix3d();
  _transitionMat << 1,  dT, dT*dT/2,   //Standard state transition matrix
                    0,  1,  dT,
                    0,  0,  0;         //Acceleration nulled at each time step
  
  _mNoiseD.param(std::normal_distribution<double>::param_type(0, 0.250)); //stddev = 3 inches
  _pNoiseD.param(std::normal_distribution<double>::param_type(0, 0.083)); //stddev = 1 inch
}

Eigen::VectorXd AccelTrain::unpare(double input){
  Eigen::VectorXd n(3);
  n << input, 0, 0;
  return n;
}

GenSim::state AccelTrain::_transition(GenSim::state input, std::chrono::milliseconds dt) {
  //We're going to be propogating things with two derivatives of precision.
  //Thus, literally just Position, dPosition/dT, d^2 Position/ dT^2
 
  //Model:
  //Position/Velocity propogate as expected; acceleration is presumed 0
  
  this->_setStateIndex(2, input[0]); //Adding input to current state vector, as acceleration will not come from any other source
     
  return _transitionMat*_getCurState(); //Standard Kalman-esque linear state transition
}  

GenSim::state AccelTrain::_measurement(GenSim::state input, std::chrono::milliseconds dt) { //Let's say there's an encoder on one of the wheels: 
  Eigen::Matrix<double, 1, 1> measurement;
  measurement[0] = input[0]*360;
  return measurement;
}

							
GenSim::state AccelTrain::_pNoise(GenSim::state state) {       //Process noise: Applies to velocity. Mean 0, Gaussian, StDev = 1 inches (1/12 ft).
  state[1] += _pNoiseD(_generatorP);
  return state;
}
GenSim::state AccelTrain::_mNoise(GenSim::state measurement, GenSim::state currentState) {       //Measure noise: Applies to position. Mean 0, Gaussian, StDev = 3 inches (1/4 ft)
  measurement[0] += _mNoiseD(_generatorM);
  return measurement;
}
