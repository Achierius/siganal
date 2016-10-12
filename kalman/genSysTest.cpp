#include "genSys.h"
#include <iostream>
#include <string>
#include "./eigen/Eigen/Dense"
#include <math.h>

using namespace Eigen;

VectorXd airResistance(VectorXd state, VectorXd prevState, double dT){
	double a = state(1);
	if(abs(a) > 10) a*=a/25;
	else a*=0.2;
	VectorXd ret(2); ret << -a*dT*dT/2, -a*dT;
	return ret;
}
VectorXd controlInput(double time){
	VectorXd ret(1); ret<<0;
	//if(time<3) ret(1) = 1;
	return ret;
}
int main(){
	double dT = 0.01;
	VectorXd state(2); state << 0, 1; //x, x'
	MatrixXd st(2,2); st << 1, dT, 0, 1;
	VectorXd q(2); q << 0.0001, 0.0001;
	VectorXd r(2); r = q;
        MatrixXd h(2,2); h << 1, 0, 0, 1;
	MatrixXd b(2,1); b << dT*dT/2, dT;
	
	GenSys sys(state, st, q, b, h, r, &airResistance, dT, 1);

	for(int i = 0; i < 10000; i++){
		sys.updateFilter(controlInput(i*dT));
		if(i%500==0) std::cout<<"Position: "<<sys.getMeasurement()(0)<<" Velocity: "<<sys.getMeasurement()(1)<<std::endl;
	}
}
