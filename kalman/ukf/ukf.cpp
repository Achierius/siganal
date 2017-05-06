#include "./ukf/ukf.hpp"


namespace ukf_mass{

~ukf(){

}

void ukf::setQ(Eigen::MatrixXd Q){
  if(Q.rows() == Q.cols() && Q.rows() = _states){
    _Q = Q;
    return;
  }
  cerr<<"In setQ: given Q Matrix dimensionality does not correspond to state vector size\n";
  return;
}

void ukf::setR(Eigen::MatrixXd R){
  if(R.rows() == R.cols() && R.rows() = _measurements){
    _R = R;
    return;
  }
  cerr<<"In setR: given R Matrix dimensionality does not correspond to measurement vector size\n";
  return;
}

