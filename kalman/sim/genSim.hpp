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
  GenSim(int stateVariables, int measurements, int inputs, std::chrono::milliseconds internalDt, Eigen::VectorXd initialState) noexcept;
 
  GenSim(const GenSim& copy) noexcept; 
  GenSim & operator= (const GenSim & copy) noexcept;
  ~GenSim() noexcept;

  using state Eigen::VectorXd;

  state getCurrentState(bool measure, bool mNoise);
  void  updateState(state controlInput, std::chrono::milliseconds duration, bool pNoise);

protected:
  void _hardSetState(state _newState);  //Note: These are meant as hard overrides! They _will_ write in a new state, however it may break the system in the next update if it has the wrong size
  void _hardSetInput(state _newInput);
  void _softSetState(state _newState);  //Setting a state via these two is subject to bounds checking via _states or _outputs
  void _softSetInput(state _newInput);

  void _setStateLength(int _newStates);
  void _setInputLength(int _newInputs);
  void _setOutputLength(int _newOutputs);
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

