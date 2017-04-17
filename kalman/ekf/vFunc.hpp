#ifndef __MASS_EKF_HPP
#define __MASS_EKF_HPP

#include <cstdarg>
#include <vector>

namespace ekf_mass{

class vFunc {
public:
  vFunc(unsigned int states);
//vFunc(unsigned int states, ... );  /* TODO */
  ~vFunc();

  Eigen::VectorXd operator()(Eigen::VectorXd parameter);
  const std::function<double(Eigen::VectorXd)>
               operator[](unsigned int i) const { return _vf[i]; }
  void setParameter(unsigned int index, double (*function)(Eigen::VectorXd));
private:
  using double (*)(Eigen::VectorXd) _vf; 
  _vf* _vector;
  unsigned int _states;
};

}

#endif//__MASS_EKF_HPP
