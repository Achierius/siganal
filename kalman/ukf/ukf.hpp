#ifndef __MASS_UKF_HPP
#define __MASS_UKF_HPP

#include <iostream>
#include <cmath>
#include <ctime>
#include <cstdio>
#include <string>
#include <chrono>
#include "./eigen/Eigen/Dense"
#include "./vFunc/vFunc.hpp"

namespace ukf_mass{

  class ukf{
    public:
      ukf(int states, int measurements, int controls, double alpha, double beta, double kappa,   //Full definition of the UKF
          std::chrono::milliseconds dT,
          Eigen::VectorXd initState, Eigen::MatrixXd initCovar, vFunc transform, vFunc measure,
          Eigen::MatrixXd Q, Eigen::MatrixXd R);
      ukf(int states, int measurements, int controls, double alpha, double beta,                 //Assumes that Kappa=0
          std::chrono::milliseconds dT,
          Eigen::VectorXd initState, Eigen::MatrixXd initCovar, vFunc transform, vFunc measure,
          Eigen::MatrixXd Q, Eigen::MatrixXd R);
      ukf(int states, int measurements, int controls, double alpha, double beta,                 //Assumes that Covar = I, initial state -> [0,...,0]^T
          std::chrono::milliseconds dT, 
          vFunc transform, vFunc measure,
          Eigen::MatrixXd Q, Eigen::MatrixXd R);

      ~ukf();
      ukf(const ukf&) = delete;
      ukf& operator = (const ukf&) = delete; //Laziest rule of three ever

      void stepUKF(std::chrono::milliseconds dT, Eigen::VectorXd measurement, Eigen::VectorXd control);
      void stepUKF(std::chrono::milliseconds dT);								      //Assumes measuremnt && control == [0...0]^T

      void setQ(Eigen::MatrixXd Q);
      void setR(Eigen::MatrixXd R);

      Eigen::MatrixXd Q(){ return _Q; }
      Eigen::MatrixXd R(){ return _R; }

      void setKappa(double kappa);
      void setAlpha(double alpha);
      void setBeta(double beta);

      double kappa(){ return _kappa; }
      double alpha(){ return _alpha; }
      double beta(){ return _beta; }

      void setTransform(vFunc transform);
      void setMeasure(vFunc measure);

      Eigen::VectorXd currentState();

      static void COOB(std::string name, std::string function, double minmax, bool min = true);

    private:

      inline void setLambda();
      inline void setDT(std::chrono::milliseconds dT);

      int _states;
      int _controls;
      int _measurements;
      
      std::chrono::milliseconds _dT;

      double _alpha; //Spread of sigma-points
      double _beta;  //Information about prior distribution; gaussian : = 2
      double _kappa; //Uh, usually 0

      double _lambda;           //alpha^s * (states + kappa) - L
      double _mean0;            //lambda / (L+lambda)
      double _covar0;           // lambda / (L + lambda)  + 1 - alpha^2 + beta
      double _statistic;        //Mean at t =/= 0  =  covar at t =/= 0  =  1/(2*(L+lambda))

      Eigen::MatrixXd _sigmaPoints; //Yeah, see the paper lmao
      Eigen::MatrixXd _measureSigmaPoints;
      ukf_mass::vFunc _transform;   //Moves from time k-1 to time k across a given dT
      ukf_mass::vFunc _measure;

      Eigen::MatrixXd _sqrtP;   //sqrt of a matrix, in this case, sqrt*sqrt^T
      Eigen::MatrixXd _Pkyy;
      Eigen::MatrixXd _Pkxy;    //lolwtf
      Eigen::MatrixXd _Kk;	//Kalman Gain

      Eigen::MatrixXd _Q;       //Covariance of the process noise
      Eigen::MatrixXd _R;       //Covariance of the measurement noise
      
      Eigen::MatrixXd _covar;
      Eigen::MatrixXd _prevCovar;
      Eigen::VectorXd _state;
      Eigen::VectorXd _prevState;

  };

}

#endif//__MASS_UKF_HPP
