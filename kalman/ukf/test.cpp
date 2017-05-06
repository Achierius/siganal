#include <iostream>
#include "./eigen/Eigen/Dense"
#include "vFunc.hpp"

using namespace std;
using namespace ekf_mass;

double func(Eigen::VectorXd input){
  return input(1) + input(0);
}
double prod(Eigen::VectorXd input){
  return input(0)*input(1);
}
int main(){
  vFunc vec(2);
  Eigen::Vector2d input(5, 5);
  vec.setParameter(0, func);
  vec.setParameter(1, ekf_mass::vFunc::zero());
  cout<<vec(input)<<endl; //2, 2
}
  
