#ifndef __MASS_EKF_VFUNC_HPP
#define __MASS_EKF_VFUNC_HPP

#include <cstdarg>
#include <vector>
#include "./eigen/Eigen/Dense"

namespace ekf_mass{

class vFunc {
public:
  vFunc();
  vFunc(unsigned int states);
  vFunc(const vFunc& object);
//vFunc(unsigned int states, ... );  /* TODO */
  ~vFunc();
  vFunc& operator = (const vFunc& object);

  void resize(unsigned int states);

  typedef double (*_vf)(Eigen::VectorXd);
  
  Eigen::VectorXd operator()(Eigen::VectorXd parameter);
  const _vf operator[](unsigned int i) const { return _vector[i]; }

  void setParameter(unsigned int index, double (*function)(Eigen::VectorXd));
  unsigned int getStates() const { return _states; }
  
  static _vf zero(){ return &f_zero; }
  static _vf one(){ return &f_one; }
private:
  _vf* _vector;
  unsigned int _states;

  static double f_zero(Eigen::VectorXd parameter) { return 0; }
  static double f_one(Eigen::VectorXd parameter) { return 1; }
};

}

#endif//__MASS_EKF_VFUNC_HPP
