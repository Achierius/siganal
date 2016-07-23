#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/core/mat.hpp"
#include <iostream>
#include <string>
using namespace std;
using namespace cv;

int main(int argc, char** argv){

    string IM_P; //Path to the image
    string IM_D; //Path to the result

    if(argc > 2){
        IM_P = argv[1];
        IM_D = argv[2];
    }
    else if(argc > 1){
        IM_P = argv[1];
        IM_D = "";
    }
    else{
        IM_P = "./duck.jpg";
    }

    Mat image = imread(IM_P, 1);
    cvtColor(image, image, CV_BGR2GRAY);
    int sz[] = {image.rows, image.cols};
    Mat noise(2, sz, CV_8U, Scalar::all(0));
    randn(noise, 120, 30);

    image = image*0.7 + noise*0.3;

    int c;

    namedWindow("white noise");
    imshow("white noise", image);
    
    c = waitKey(0);
}
