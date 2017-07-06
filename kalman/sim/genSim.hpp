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
 ***** GenSim.hpp *****
 */

namespace ukf_mass{

class GenSim{
public:
  GenSim();
 
  GenSim(const GenSim& copy); 
  GenSim & operator= (const GenSim & copy);
  ~GenSim();

  using state Eigen::VectorXd;
  using ms std::chrono::milliseconds;

  state getCurrentState(bool measure, bool mNoise);
  void  updateState(state controlInput, ms duration, bool pNoise);
  
protected:
  int   _states;  //Number of state variables
  int   _outputs; //Number of output variables [from _measurement(~)]
  state _state;   //Current system state
  state _input;   //Most recent control input
  ms    _dt;      //

  virtual state _measurement(state input, ms dT) = 0;    //dT represents the individual time-gap between sub-partitions of the overal update interval.
  virtual state _transition(state input, ms dT) = 0;                       //

  virtual state _pNoise(state state) = 0;                                                     //Applies noise to state provided. Returns _states x 1 vector.
  virtual state _mNoise(state measurement, state state) = 0;                                  //Applies noise to the measurement provided, presumably from the _measurement function.
                                                                                      //cont. Returns `_outputs x 1` size vector. May vary noise with actual system state.
  
};

}

