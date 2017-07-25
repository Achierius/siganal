#include "genSim.hpp"
#include <cmath>


GenSim::GenSim() noexcept{
  _outputs = measurements;
  _states = stateVariables;
  _inputs = inputs;  
  _dt = internalDt;
  
  if(initialState.cols() == 1 && initialState.rows() == _states){
    _state = initialState;
  }
  else{
    std::cerr<<"Invalid state size in GenSim::GenSim(~) from parameter. State size: "<<initialState.size()<<" Expected: "<<_states<<"x1"<<std::endl;
  }
}
GenSim::GenSim(const GenSim& copy) noexcept{
  this->_outputs = copy.measurements;
  this->_states = copy.stateVariables;
  this->_inputs = copy.inputs;  
  this->_dt = copy.internalDt;
  this->_state = copy.initialState;
}
GenSim& GenSim::operator= (const GenSim& copy) noexcept{
  this->_outputs = copy.measurements;
  this->_states = copy.stateVariables;
  this->_inputs = copy.inputs;  
  this->_dt = copy.internalDt;
  this->_state = copy.initialState;
}
~GenSim() noexcept{
  //We aint found shit
}


void GenSim::_hardSetState(state _newState){
  if(_newState.cols() != 1){
    std::cerr<</*   TODO    */<<std::endl;
    return;
  }
  _states = _newState.rows();
  _state = _newState;
}
void GenSim::_hardSetInput(state _newInput){
  if(_newInput.cols() != 1){
    std::cerr<</*   TODO    */<<std::endl;
    return;
  }
  _inputs = _newInput.rows();
  _input = _newInput;
}
void GenSim::_softSetState(state _newState){
  if(_newState.cols() == 1 && _newState.rows() == _states){
    _state = _newState;
    return;
  }
  else{
    std::cerr<</*    TODO    */<<std::endl;
    return;
  }
}
void GenSim::_softSetInput(state _newInput){
  if(_newInput.cols() == 1 && _newInput.rows() == _inputs){
    _input= _newInput;
    return;
  }
  else{
    std::cerr<</*    TODO    */<<std::endl;
    return;
  }
}
void GenSim::_setOutputLength(int _newOutputs){
  if(_newOutputs > 0){
    _outputs = _newOutputs;
  }
  else{
    std::cerr<</*    TODO    */<<std::endl;
  }
}
void GenSim::_setInputLength(int _newInputs){  //You can indeed have 0 inputs to a system
  if(_newInputs >= 0){
    _inputs= _newInputs;
  }
  else{
    std::cerr<</*    TODO    */<<std::endl;
  }
}
void GenSim::_setStateLength(int _newStates){
  if(_newStates > 0){
    _states= _newStates;
  }
  else{
    std::cerr<</*    TODO    */<<std::endl;
  }
}

void GenSim::_setStateIndex(int index, double newValue){
  if(index >= _states){
    std::cerr<<"In GenSim::_setStateIndex(~), given index "<<index<<" beyond the size of the state vector, "<<_states<<std::endl;
  }
  else{
    _state[index] = newValue;
  }
}

void GenSim::_setDT(std::chrono::milliseconds _newDT){
  if(_newDT <= std::chrono::ms(0)){
    std::cerr<<"In GenSim::_setDT(~), _newDT "<<_newDT<<" is equal to or less than the minimum, 0ms"<<std::endl;
  }
  else{
    _dt = _newDT;
  }
}


GenSim::state GenSim::getCurrentState(bool measure, bool mNoise){ //TODO: _outputs based bounds checking
  if(measure){
    if(mNoise){
      return _mNoise(_measurement(this->_state, this->_dT), this->_state);    //TODO: It might be better to have a measurement stored per iteration to avoid being able to repeatedly sample
    }
    else{
      return _measurement(this->_state, this->_dT);
    }
  }
  else{
    return this->_getCurState();
  }
}

void GenSim::updateCurrentState(GenSim::state controlInput, std::chrono::milliseconds duration, bool pNoise){
  for(std::chrono::milliseconds i = std::chrono::milliseconds(0); i < duration; i += std::min(this->_dT, duration-i)){
      Eigen::VectorXd temp = _transition(controlInput, std::min(this->_dT, duration-i));                               

    if(temp.cols() == 1 && temp.rows() == _states){      //I don't use _setState() because I shouldn't be resetting _states here.
      _state = temp;
    }
    else{
      std::cerr<<"Invalid state size in GenSim::updateCurrentState(~) from GenSim::_transitition(~). State size: "<<temp.size()<<" Expected: "<<_states<<"x1"<<std::endl;
    }

    if(pNoise){ _state = _pNoise(_state); }
  }
  if(duration <= std::chrono::milliseconds(0)){
    std::cerr<<"Invalid duration in GenSim::updateCurrentState(~). Duration: "<<duration<<std::endl;
    _state = _state; 
    /* Invalid 
  }
}
