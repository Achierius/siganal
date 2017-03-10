#include "./eigen/Eigen/Dense"
#include <iostream>

using namespace Eigen;

int main(int argn, char* args[]){
  
  double r = 0; double p = 0; double h = 0;
  
  if(argn>1){
    r+=char(args[1][0])-'0';
    if(argn>2){
      p+=char(args[2][0])-'0';
      if(argn>3){
        h+=char(args[3][0])-'0';
      }
      else{ h+=1; }
    }
    else{ h+=1; p+=1; }
  }
  else{ h+=1; p+=1; r+=1; }
  
  double scale = 0.1;//0.000001;
  double e = scale;
 
  
  Matrix3d k_H; k_H << 1,  0,  0,
                       0,  1,  0,
                       0,  0,  1;
  
  Matrix3d k_P; k_P << 10*e,  1*e,  1*e,
                       1*e,  10*e,  1*e,
                       1*e,   1*e, 10*e;
  
  Matrix3d k_R; k_R << 10*e,  1*e,  1*e,
                       1*e,  10*e,  1*e,
                       1*e,   1*e, 10*e;
 
  k_R = k_R*r;
  k_P = k_P*p;
  k_H = k_H*h;
  
  MatrixXd k_K;
  
  k_K = k_P*k_H.transpose()*(k_H*k_P*k_H.transpose()+k_R).inverse();
  //k_K = k_P*((k_P+k_R).inverse());
  std::cout<<k_K<<std::endl;
}