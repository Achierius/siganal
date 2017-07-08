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
  void GenSim::updateCurrentState(GenSim::state controlInput, GenSim::ms duration, bool pNoise){
    for(GenSim::ms i = std::chrono::milliseconds(0); i < duration; i += std::min(this->_dT, duration-i)){
      _state = _transition(controlInput, std::min(this->_dT, duration-i));
      if(pNoise){ _state = _pNoise(_state); }
    }
    else{
      std::cerr<<"Invalid duration in GenSim::updateCurrentState(~)"
      _state = _state; 
     /* Invalid 
    }
  }

}
