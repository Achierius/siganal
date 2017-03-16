#include <iostream>
#include "diffDrive.h"
#include <cmath>


y2017::akf_sim::differential_drive::differential_drive(const double dT, const double length,
                                                       const double kV, const double kT, const double mR, const double mJ, const double mr,
                                                       const double mass, const double momentOfInertia,
                                                       const Eigen::VectorXd& initialStates){
  _dT = dT;
  _chassisLength = length;
  _kV = kV;
  _kT = kT;
  _mR = mR;
  _mJ = mJ;
  _mr = mr;
  _mass = mass;
  _J = momentOfInertia;

  _state.resize(8);
  if(initialStates.size() != 8){
    for(int i = 0; i < 8; i++){
      _state[i] = 0;
    }
  } 
  else{
    for(int i = 0; i < 8; i++){
      _state[i] = initialStates[i];
    }
  }
}

void y2017::akf_sim::differential_drive::setInputs(const double leftVoltage, const double rightVoltage){
  _inputs[0] = std::max(std::min(static_cast<int>(leftVoltage), 12), -12);
  _inputs[1] = std::max(std::min(static_cast<int>(rightVoltage), 12), -12); //Assuming FRC-standard motor -- min/max of 12 Volts.
}
void y2017::akf_sim::differential_drive::driveStraight(const double voltage){
  this->setInputs(voltage, voltage); //Positive -> Forward, Negative -> Backward
}
void y2017::akf_sim::differential_drive::quickTurn(const double voltage){
  this->setInputs(voltage, -voltage); //Positive voltage turns clockwise, negative counter-clockwise
}

double y2017::akf_sim::differential_drive::getValue(const double index) const{
  if(index > Stot || index < xPos) { return 0; }
  return _state[index];
}

void y2017::akf_sim::differential_drive::stepDrive(){
  _statePrev = _state; //Is deep copy default or?
  this->updateAbsolutePosition();
  this->updateForwardVelocities();
  this->updateAngularStatistics(); //Order matters! UAS requires the current Stot and doesn't want to use a previous value.
  _leftWheelDistance += _statePrev[Sl]*_dT;
  _rightWheelDistance += _statePrev[Sr]*_dT; //For the sake of simulating encoder readings.
}

void y2017::akf_sim::differential_drive::updateAbsolutePosition(){
   _state[xPos] = _statePrev[xPos] + _statePrev[Stot]*std::cos(_statePrev[theta]+_statePrev[omega]*_dT/2);
   _state[yPos] = _statePrev[yPos] + _statePrev[Stot]*std::sin(_statePrev[theta]+_statePrev[omega]*_dT/2); //The w*t/2 is just mini-euler'ing the linearization
}
void y2017::akf_sim::differential_drive::updateAngularStatistics(){ //All of the following relations are derived from the base relation of a differential drive-train: omega of the left wheel = omega of 
                                                //the right wheel, assuming that the entire body will drive in a perfect circle with any differential in wheel-driving-voltages.
  _state[omega] = (_statePrev[Sr]-_statePrev[Sl])/_chassisLength;
  _state[r] = _chassisLength/2 * (_statePrev[Sr] + _statePrev[Sl])/(_statePrev[Sr] - _statePrev[Sl]); //Notice that Omega and R are calculated instantaneously, rather than via
  _state[theta] = _statePrev[theta] + _statePrev[omega]*_dT;                                          //integration. 
}
void y2017::akf_sim::differential_drive::updateForwardVelocities(){ //Below equations are derived from DC Motor equations, assuming left/right identical motor constants and L~0.
  _state[Sr] = _statePrev[Sr] + _dT*(-1*_kT/_kV * 1/(_mR*_mJ)*_statePrev[Sr] + _kT/(_mr*_mR*_mJ)*(_inputs[1]));
  _state[Sl] = _statePrev[Sl] + _dT*(-1*_kT/_kV * 1/(_mR*_mJ)*_statePrev[Sl] + _kT/(_mr*_mR*_mJ)*(_inputs[0]));
}

