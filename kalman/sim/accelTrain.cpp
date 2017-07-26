#include "accelTrain.hpp"

AccelTrain::AccelTrain(double initState) : GenSim::GenSim(3, 1, 1, std::chrono::ms(1), AccelTrain::unpare(initState)) {

  double dT = 0.001;                //Seconds per millisecond
  _transition << 1,  dT, dT*dT/2;   //Standard state transition matrix
                 0,  1,  dt;
                 0,  0,  0;         //Acceleration nulled at each time step
  
  _mNoise.param(std::normal_distribution<double>::param_type(0, 0.250); //stddev = 3 inches
  _pNoise.param(std::normal_distrbiution<double>::param_type(0, 0.083); //stddev = 1 inch
}

static Eigen::VectorXd unpare(double input){
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
     
  return _transition*_getCurState(); //Standard Kalman-esque linear state transition
}  

GenSim::state AccelTrain::_measurement(GenSim::state input, std::chrono::milliseconds dt) { //Let's say there's an encoder on one of the wheels: 
  Eigen::Matrix<double, 1, 1> measurement;
  measurement[0] = input[0]*360;
  return measurement;
}

							
GenSim::state GenSim::_pNoise(GenSim::state state) {       //Process noise: Applies to velocity. Mean 0, Gaussian, StDev = 1 inches (1/12 ft).
  state[1] += _pNoise(_generatorP);
  return state;
}
GenSim::state GenSim::_mNoise(GenSim::state measurement) {       //Measure noise: Applies to position. Mean 0, Gaussian, StDev = 3 inches (1/4 ft)
  measurement[0] += _mNoise(_generatorM);
  return measurement;
}
