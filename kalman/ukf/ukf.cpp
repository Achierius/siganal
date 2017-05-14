#include "../ukf/ukf.hpp"
#include <iostream>
#include <math.h>
#include <algorithm>
#include <chrono>
#include "./eigen/unsupported/Eigen/MatrixFunctions"



namespace ukf_mass{


ukf::ukf(int states, int measurements, int controls, double alpha, double beta, double kappa,   //Full definition of the UKF
    std::chrono::milliseconds dT,
    Eigen::VectorXd initState, Eigen::MatrixXd initCovar, vFunc transform, vFunc measure,
    Eigen::MatrixXd Q, Eigen::MatrixXd R){

  if(states > 0){ _states = states; }
  else{
    std::cerr<<"In ukf(~): <1 value of given states\n"; states = 1;
  }
  if(measurements > 0){ _measurements = measurements; }
  else{
    std::cerr<<"In ukf(~): <1 value of given measurements\n"; states = 1;
  }
  if(controls > 0){ _controls = controls; }
  else{
    std::cerr<<"In ukf(~): <1 value of given controls\n"; controls = 1;
  }

  this->setQ(Q);
  this->setR(R);

  this->setMeasure(measure);
  this->setTransform(transform);

  _alpha = 0; _beta = 0; _kappa = 0;
  this->setAlpha(alpha);
  this->setBeta(beta);
  this->setKappa(kappa);

  if(initCovar.cols() == initCovar.rows() && initCovar.rows() == _states){
    _prevCovar = initCovar;
    _covar = initCovar;
  }
  else{
    std::cerr<<"In ukf(~): incorrect size of initial covariance matrix\n";
    _prevCovar = Eigen::MatrixXd::Zero(_states, _states);
    _covar = Eigen::MatrixXd::Zero(_states, _states);
  }
  if(initState.cols() == 1 && initState.rows() == _states){
    _prevState = initState;
    _state = initState;
  }
  else{
    std::cerr<<"In ukf(~): incorrect size of initial state vector\n";
    _prevState = Eigen::VectorXd::Zero(_states); //_prevState.resize(_states, 1); _prevState.zero();
    _state = Eigen::VectorXd::Zero(_states); //_state.resize(_states, 1); _state.zero();
  }

  _sigmaPoints = Eigen::MatrixXd::Constant(_states, _states*2+1, 0);
  _measureSigmaPoints = Eigen::MatrixXd::Constant(_states, _states*2+1, 0);

}

ukf::ukf(int states, int measurements, int controls, double alpha, double beta,                 //Assumes that Kappa=0
    std::chrono::milliseconds dT,
    Eigen::VectorXd initState, Eigen::MatrixXd initCovar, vFunc transform, vFunc measure,
    Eigen::MatrixXd Q, Eigen::MatrixXd R){

  if(states > 0){ _states = states; }
  else{
    std::cerr<<"In ukf(~): <1 value of given states\n"; states = 1;
  }
  if(measurements > 0){ _measurements = measurements; }
  else{
    std::cerr<<"In ukf(~): <1 value of given measurements\n"; states = 1;
  }
  if(controls > 0){ _controls = controls; }
  else{
    std::cerr<<"In ukf(~): <1 value of given controls\n"; controls = 1;
  }

  this->setQ(Q);
  this->setR(R);

  this->setMeasure(measure);
  this->setTransform(transform);

  _alpha = 0; _beta = 0; _kappa = 0;
  this->setAlpha(alpha);
  this->setBeta(beta);
  this->setKappa(0);

  if(initCovar.cols() == initCovar.rows() && initCovar.rows() == _states){
    _prevCovar = initCovar;
    _covar = initCovar;
  }
  else{
    std::cerr<<"In ukf(~): incorrect size of initial covariance matrix\n";
    _prevCovar = Eigen::MatrixXd::Zero(_states, _states); //_prevCovar.resize(_states, _states); _prevCovar.zero();
    _covar = Eigen::MatrixXd::Zero(_states, _states); //_covar.resize(_states, _states); _covar.zero();
  }
  if(initState.cols() == 1 && initState.rows() == _states){
    _prevState = initState;
    _state = initState;
  }
  else{
    std::cerr<<"In ukf(~): incorrect size of initial state vector\n";
    _prevState = Eigen::VectorXd::Zero(_states); //_prevState.resize(_states, 1); _prevState.zero();
    _state = Eigen::VectorXd::Zero(_states); //_state.resize(_states, 1); _state.zero();
  }

  _sigmaPoints = Eigen::MatrixXd::Constant(_states, _states*2+1, 0);
  _measureSigmaPoints = Eigen::MatrixXd::Constant(_states, _states*2+1, 0);

}

ukf::ukf(int states, int measurements, int controls, double alpha, double beta,                 //Assumes that Covar = 0, initial state -> [0,...,0]^T
    std::chrono::milliseconds dT,
    vFunc transform, vFunc measure,
    Eigen::MatrixXd Q, Eigen::MatrixXd R){

  if(states > 0){ _states = states; }
  else{
    std::cerr<<"In ukf(~): <1 value of given states\n"; states = 1;
  }
  if(measurements > 0){ _measurements = measurements; }
  else{
    std::cerr<<"In ukf(~): <1 value of given measurements\n"; states = 1;
  }
  if(controls > 0){ _controls = controls; }
  else{
    std::cerr<<"In ukf(~): <1 value of given controls\n"; controls = 1;
  }

  this->setQ(Q);
  this->setR(R);

  this->setMeasure(measure);
  this->setTransform(transform);

  _alpha = 0; _beta = 0; _kappa = 0;
  this->setAlpha(alpha);
  this->setBeta(beta);
  this->setKappa(0);

  _prevCovar = Eigen::MatrixXd::Zero(_states, _states); //_prevCovar.resize(_states, _states); _prevCovar.zero();
  _covar = Eigen::MatrixXd::Zero(_states, _states); //_covar.resize(_states, _states); _covar.zero();

  _prevState = Eigen::VectorXd::Zero(_states); //_prevState.resize(_states, 1); _prevState.zero();
  _state = Eigen::VectorXd::Zero(_states); //_state.resize(_states, 1); _state.zero();

  _sigmaPoints = Eigen::MatrixXd::Constant(_states, _states*2+1, 0);
  _measureSigmaPoints = Eigen::MatrixXd::Constant(_states, _states*2+1, 0);

}

ukf::~ukf(){

}

void ukf::setQ(Eigen::MatrixXd Q){
  if(Q.rows() == Q.cols() && Q.rows() == _states){
    _Q = Q;
    return;
  }
  std::cerr<<"In setQ: given Q Matrix dimensionality does not correspond to state vector size\n";
  return;

}

void ukf::setR(Eigen::MatrixXd R){
  if(R.rows() == R.cols() && R.rows() == _measurements){
    _R = R;
    return;
  }
  std::cerr<<"In setR: given R Matrix dimensionality does not correspond to measurement vector size\n";
  return; 
}


void ukf::setTransform(vFunc transform){
  _transform = transform;
}

void ukf::setMeasure(vFunc measure){
  _measure = measure;
}

void ukf::stepUKF(std::chrono::milliseconds dT){
  this->stepUKF(dT, Eigen::VectorXd::Zero(_measurements), Eigen::VectorXd::Zero(_controls));
}

//Actual Kalman Filter equations
void ukf::stepUKF(std::chrono::milliseconds dT, Eigen::VectorXd measurement, Eigen::VectorXd control){
  for(auto j = std::chrono::milliseconds(0); j < (dT>_dT ? dT : _dT); j+=_dT){
   
    _prevState= _state;
    _prevCovar= _covar; 

    _statistic = 1/(2*(_states+_lambda));
    _sqrtP = _covar.sqrt();
    double coeff = std::sqrt(_states+_lambda);
    
    _sigmaPoints.col(0) = _prevState;
    _sigmaPoints.block(0, 1, _states, _states) = coeff*_sqrtP;
    _sigmaPoints.block(0, 1+_states, _states, _states) = -1*coeff*_sqrtP;
    for(auto i = 1; i < 2*_states+1; i++){
      _sigmaPoints.col(i) += _prevState;
    }
    
    for(auto i = 0; i < 2*_states+1; i++){
      _sigmaPoints.col(i) = _transform(_sigmaPoints.col(i), static_cast<double>(std::min(_dT, dT).count()));
    } 
    
    _state = _sigmaPoints.col(0);
    for(auto i = 1; i < 2*_states+1; i++){
      _state+= _sigmaPoints.col(i);
    }
    _state *= _statistic;
 
    _covar = _Q;
    for(auto i = 0; i < 2*_states+1; i++){
      _covar += (_sigmaPoints.col(i) - _prevState)*(static_cast<Eigen::MatrixXd>(_sigmaPoints.col(i) - _prevState)).transpose();
    }
    _state *= _statistic;
    
    for(auto i = 0; i < 2*_states+1; i++){
      _measureSigmaPoints.col(i) = _measure(_sigmaPoints.col(i), static_cast<double>(std::min(_dT, dT).count()));
    } 
  }
}

void ukf::setKappa(double kappa){ 
  _kappa = kappa;
  setLambda();
}

void ukf::setAlpha(double alpha){
  if(alpha > 1){
    ukf::COOB("alpha", "setAlpha(double alpha)", 1, true);
    _alpha = 1;
    setLambda();
    return;
  }
  if(alpha < 0.0004){
    ukf::COOB("alpha", "setAlpha(double alpha)", 0.0001);
    _alpha = 0.0004;
    setLambda();
    return;
  }
  _alpha = alpha;
  setLambda();
  return;
}

void ukf::setBeta(double beta){
  _beta = beta;
  setLambda();
}

//Constant out of bounds
void ukf::COOB(std::string name, std::string function, double minmax, bool min){
  if(min){ std::cerr<<"In "<<function<<" "<<name<<" is less than the minimum "<<minmax<<"\n"; }
  else{ std::cerr<<"In "<<function<<" "<<name<<" is greater than the maximum "<<minmax<<"\n"; }
} 

Eigen::VectorXd ukf::currentState(){
  return _state;
}

inline void ukf::setLambda(){
  _lambda = _alpha*_alpha*(_states + _kappa) - _states;
  _mean0 = _lambda/(_states+_lambda);
  _covar0 = _mean0 + 1 - _alpha*_alpha + _beta;
}
inline void ukf::setDT(std::chrono::milliseconds dT){
  if(dT <= std::chrono::milliseconds(0)){ 
    ukf::COOB("the given dT", "setDT", 0);
    _dT = std::chrono::milliseconds(1);
  }
  else{ _dT = dT; }
}

} //namespace ukf_mass
