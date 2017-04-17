#include "vFunc.hpp"
#include <iostream>

ekf_mass::vFunc::vFunc(unsigned int states) {
  _vector = new _vf[states];
  _states = states;
}

ekf_mass::vFunc::~vFunc() {
  delete [] _vector;
}

Eigen::VectorXd ekf_mass::vFunc::operator()(Eigen::VectorXd parameter) {
  if(parameter.size() != _states) {
    std::cerr<<"vFunc application call failed; parameter vector of unmatched size "<<parameter.size()<<"; desired rows "<<_states<<std::endl;
  }
  Eigen::VectorXd _m(_states);
  for(int i = 0; i < _states; i++) {
    _m(i) = (_vector[i])(parameter);
  }
 return _m;
}

void ekf_mass::vFunc::setParameter(unsigned int index, double (*function)(Eigen::VectorXd)) {
  if(index < _states && index >= 0) {
    _vector[index] = function;
  }
  else {
    std::cerr<<"vFunc setParameter() call index out of bounds; _states = "<<_states<<std::endl;
  }
}