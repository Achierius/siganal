#include "vFunc.hpp"
#include <iostream>

ukf_mass::vFunc::vFunc() {
  _vector = new _vf[1];
  _states = 1;
}

ukf_mass::vFunc::vFunc(unsigned int states) {
  _vector = new _vf[states];
  _states = states;
}

ukf_mass::vFunc::vFunc(const ukf_mass::vFunc& object) {
  _vector = new _vf[object.getStates()];
  _states = object.getStates();
  for(int i = 0; i < _states; i++) {
    _vector[i] = object[i];
  }
}

/*ukf_mass::vFunc::vFunc(unsigned int states, ... ) {
  va_start;
  
}*/


ukf_mass::vFunc::~vFunc() {
  delete [] _vector;
}

ukf_mass::vFunc& ukf_mass::vFunc::operator = (const ukf_mass::vFunc& object) {
  this->resize(object.getStates());
  for(int i = 0; i < _states; i++) {
    _vector[i] = object[i];
  }
}


void ukf_mass::vFunc::resize(unsigned int states) {
  delete [] _vector;
  _vector = new _vf[states];
  _states = states;
}


Eigen::VectorXd ukf_mass::vFunc::operator()(Eigen::VectorXd vector, double scalar) {
  if(vector.size() != _states) {
    std::cerr<<"vFunc application call failed; parameter vector of unmatched size "<<vector.size()<<"; desired rows "<<_states<<std::endl;
  }
  Eigen::VectorXd _m(_states);
  for(int i = 0; i < _states; i++) {
    _m(i) = (_vector[i])(vector, scalar);
  }
 return _m;
}

void ukf_mass::vFunc::setParameter(unsigned int index, double (*function)(Eigen::VectorXd, double)) {
  if(index < _states && index >= 0) {
    _vector[index] = function;
  }
  else {
    std::cerr<<"vFunc setParameter() call index out of bounds; _states = "<<_states<<std::endl;
  }
}
