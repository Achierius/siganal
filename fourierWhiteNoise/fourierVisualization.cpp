#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/core/mat.hpp"
#include <iostream>
#include <string>
#include "dftMod.h"
using namespace std;
using namespace cv;



int main(int argc, char** argv){

    string IM_P; //Path to the image

    if(argc > 1){
        IM_P = argv[1];
    }
    else{
        IM_P = "./duck.jpg";
    }

    Mat image = imread(IM_P, 1);

    siganal::visualizeFDomain(image);

    namedWindow("shaped");

    int c;
    while(true){
        imshow("shaped", image);
        c = waitKey(10);
   }
   
   return 0;
}
