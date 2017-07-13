#include <iostream>
#include "./eigen/Eigen/Dense"
#include <string>
#include <chrono>
#include <cmath>
#include <ctime>


/*
 ***** GenSim.hpp *****
 *
 * General System Representation
 * Assumptions: Time invariant system transition
 *
 * Represents the system with a single state vector. Transitions consist of
 * application by both the previous state and a control input vector 
 * simultaneously. Transition may be non-linear.
 *
 * Two stages of the transition:
 *  -Physical Transition
 *  -Measurement
 * The latter of which represents the measured output of
 * the system ('y' vector). Measurement may be nonlinear.
 * 
 * System noise applied in 'measurement' and 'process' noise segments.
 * Both types of noise may be Non-Gaussian/White/Zero Mean, and may
 * vary with the system state.
 *
 * @author Marcus Plutowski <achierius@gmail.com>
 *
 ***** GenSim.hpp *****
 */

namespace ukf_mass{

class GenSim{
public:
  GenSim() noexcept;
 
  GenSim(const GenSim& copy) noexcept; 
  GenSim & operator= (const GenSim & copy) noexcept;
  ~GenSim() noexcept;

  using state Eigen::VectorXd;

  state getCurrentState(bool measure, bool mNoise);
  void  updateState(state controlInput, ms duration, bool pNoise);

protected:
  void _setState(state _newState);
  void _setInput(state _newInput);
  void _setOutputSize(int _newOutputs);
  void _setDT(std::chrono::milliseconds _newDt);

  virtual state _measurement(state input, std::chrono::milliseconds dT) = 0;    //
  virtual state _transition(state input, std::chrono::milliseconds dT) = 0;     //
 										//dt: duration of sub-partitions of total time elapsed in update step; used to minimize propogation error
										//dT: duration of total update step, may be much longer than a reasonable single physical update

  virtual state _pNoise(state currentState) = 0;                                                     //Applies noise to state provided. Returns _states x 1 vector.
  virtual state _mNoise(state measurement, state currentState) = 0;                                  //Applies noise to the measurement provided, presumably from the _measurement function.

private:
  int   _states;  //Number of state variables [size of [_state]
  int   _outputs; //Number of output variables [size of the returned variable from _measurement(~)]; output is defined as basically a system measurement
  int   _inputs;  //Number of input variables [size of _input]
 
  state _state;   //Current system state
  state _input;   //Most recent control input: user inputs to system
 
  std::chrono::milliseconds _dt; //See above

};

}

