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
void y2017::akf_sim::differential_drive::updateAngularStatistics(){ //All of the following relations are derived from the base relation of a differential drive-train: omega of the left
                                                                    //wheel = omega of the right wheel, assuming that the entire body will drive in a perfect circle with
                                                                    //any differential in wheel-driving-voltages.
  double NO_DIVIDE_BY_ZERO = 0.000001;
  _state[omega] = (_statePrev[Sr]-_statePrev[Sl])/_chassisLength;
  _state[r] = _chassisLength/2 * (_statePrev[Sr] + _statePrev[Sl])/(_statePrev[Sr] - _statePrev[Sl] + NO_DIVIDE_BY_ZERO); //Notice that Omega and R are calculated instantaneously, 
  _state[theta] = _statePrev[theta] + _statePrev[omega]*_dT;                                                              //rather than via integration. 
  _state[theta] = regulateTheta(_state[theta]);
}
void y2017::akf_sim::differential_drive::updateForwardVelocities(){ //Below equations are derived from DC Motor equations, assuming left/right identical motor constants and L~0.
  double CONSOLIDATED_MOTOR_CONSTANT = _kT/_kV * 1/(_mR*_mJ);
  double CMC = CONSOLIDATED_MOTOR_CONSTANT;
  double LOOP_DIFFERENCE = 0.001; //0.1 MS

  double sr = _statePrev[Sr];
  double sl = _statePrev[Sl];
  for(double i = 0; i < _dT/LOOP_DIFFERENCE; i++){
    sr += LOOP_DIFFERENCE*CMC*(_kV*_inputs[1]-_statePrev[Sr]);
    sl += _statePrev[Sl] + LOOP_DIFFERENCE*CMC*(_kV*_inputs[0]-_statePrev[Sl]);
  }
  _state[Sr] = sr;
  _state[Sl] = sl;

  _state[Stot] = _state[Sr]/2 + _state[Sl]/2;
}

double y2017::akf_sim::differential_drive::regulateTheta(const double& inputTheta){
  const double ANGULAR_LIMIT = 3.1415926535897932384626433832; //Thanks Chris; It's in Radians, 180 degrees = pi
  double outputTheta = inputTheta; 

  int moduloFactor = std::floor(std::abs(outputTheta)/(2*ANGULAR_LIMIT));
  int signNum = std::abs(outputTheta)/outputTheta;
  outputTheta += -1*signNum*moduloFactor*(2*ANGULAR_LIMIT); //Limits the range to -360 to 360 degrees, -2pi to 2pi.
  
  if(std::abs(outputTheta) > ANGULAR_LIMIT){ //Limits the range to -180 to 180, -pi to pi.
    outputTheta = (outputTheta-signNum*ANGULAR_LIMIT)*-1;
  }

  return outputTheta;
}

