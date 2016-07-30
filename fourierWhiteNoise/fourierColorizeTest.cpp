#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/core/mat.hpp"
#include <iostream>
#include <string>
#include <sstream>
#include <cmath>
#include <complex>
using namespace std;
using namespace cv;

int main(int argc, char** argv){
   
    int c;

    Mat image = imread("./duck.jpg", 1);
    cvtColor(image, image, CV_BGR2GRAY);

    int optW = getOptimalDFTSize(image.cols); 
    int optH = getOptimalDFTSize(image.rows);
    copyMakeBorder(image, image, 0, optH-image.rows, 0, optW-image.cols, BORDER_CONSTANT, Scalar::all(0));

    Mat planes[] = {Mat_<float>(image), Mat::zeros(image.size(), CV_32F)};
    Mat_<float> complexIm(image);
    //merge(planes, 2, complexIm); //2-channel image time

    dft(complexIm, complexIm); //Uh... I think because the second channel is all zeroes, it doesn't affect the DFT?
    cout<<complexIm.at<float>(2,2)<<endl;


    Mat complexIm2;


    merge(planes, 2, complexIm2);

    dft(complexIm2, complexIm2);

    split(complexIm2, planes);
    cout<<planes[1].at<float>(2,1)<<endl;
  return 0;
}
