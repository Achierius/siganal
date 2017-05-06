#include "diffDrive.h"
#include <iostream>
#include <cmath>
#include <iomanip>

using namespace std;
using namespace y2017::akf_sim;

int main(){
  Eigen::VectorXd init(8);
  init << 0, 0, 0, 0, 0, 0, 0, 0;
  differential_drive robot(0.01, 1, 1, 1, 2, 2, 0.5, 0.1, 0.1, init);  
 
  cout<<robot.regulateTheta(-4.5*3.141592)<<"\n";
  double ms = 1000;
  double time = 0; //Milliseconds

  cout<<fixed<<std::setprecision(3);
  while(time <= 10*ms){
    if(time < 2*ms){
      robot.setInputs(1,1);
    }
    else if(time > 2*ms && time < 5*ms){
      //robot.quickTurn(1);//driveStraight(-5);
    }
    else{
      robot.driveStraight(0);
    } 
    if(int(time)%500 == 0){
      cout<<"Position at time "<<time/ms<<": "<<robot.getValue(6)<<".\n";
    }
    robot.stepDrive();
    time+=10.0;
  }
}
