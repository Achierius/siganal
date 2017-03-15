#include <iostream>
#include "diffDrive.h"
#include <cmath>

y2017::akf_sim::differential_drive(){
}

void y2017::akf_sim::setInputs(const double leftVoltage, const double rightVoltage){
  _inputs[0] = max(min(leftVoltage, 12), -12);
  _inputs[1] = max(min(rightVoltage, 12), -12);
}
void y2017::akf_sim::driveStraight(const double voltage){
  this->setInputs(voltage, voltage);
}
void y2017::akf_sim::quickTurn(const double voltage){
  this->setInputs(voltage, -voltage);
}

void stepDrive(){
  _statePrev = _state; //Is deep copy default or?
  this->updateAbsolutePosition();
  this->updateForwardVelocities();
  this->updateAngularStatistics(); //Order matters!
  _leftWheelDistance += _statePrev[5]*_dT;
  _rightWheelDistance += _statePrev[6]*_dT;
}

