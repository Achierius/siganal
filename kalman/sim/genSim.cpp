#include "genSim.hpp"
#include <cmath>

namespace ukf_mass{


GenSim::GenSim(){

}

GenSim::GenSim(const GenSim& copy){

}
GenSim& GenSim::operator= (const GenSim& copy){

}
~GenSim(){

}

GenSim::state GenSim::getCurrentState(bool measure, bool mNoise){
  if(measure){
    if(mNoise){
      return _mNoise(_measurement(this->_state, this->_dT), this->_state);    //TODO: It might be better to have a measurement stored per iteration to avoid being able to repeatedly sample
    }
    else{
      return _measurement(this->_state, this->_dT);
    }
  }
  else{
    return this->_state;
  }
}

void GenSim::updateCurrentState(GenSim::state controlInput, std::chrono::milliseconds duration, bool pNoise){
  for(std::chrono::milliseconds i = std::chrono::milliseconds(0); i < duration; i += std::min(this->_dT, duration-i)){
    Eigen::VectorXd temp = _transition(controlInput, std::min(this->_dT, duration-i));                               
    if(temp.cols() == 1 && temp.rows() == _output){      //I don't use _setState() because I shouldn't be resetting _states here.
    }
    else{
      std::cerr<<"Invalid state size in GenSim::updateCurrentState(~) from GenSim::_transitition(~). Control Input: "<<controlInput<<std::endl;
    }

    if(pNoise){ _state = _pNoise(_state); }
  }
  if(duration <= std::chrono::milliseconds(0)){
    std::cerr<<"Invalid duration in GenSim::updateCurrentState(~). Duration: "<<duration<<std::endl;
    _state = _state; 
    /* Invalid 
  }
}


} //anmespace ukf_mass
