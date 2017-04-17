#include "vFunc.hpp"
#include <iostream>

vFunc::vFunc(unsigned int states) {
  _vector = new _vf[states];
  _states = states;
}
vFunc::~vFunc() {
  delete _vf;
}

Eigen::VectorXd vFunc::operator()(Eigen::VectorXd parameter) {
  Eigen::VectorXd _m(_states);
  for(int i = 0; i < _states; i++) {
    _m(i, 1) = (_vector[i])(parameter);
  }
 return _m;
}

void vFunc::setParameter(unsigned int index, double (*function)(Eigen::VectorXd)) {
  if(index < _states && index > 0) {
    _m[index] = function;
  }
  else {
    cerr<<"vFunc setParameter() call index out of bounds; _states = "<<_states<<std::endl;
  }
}
