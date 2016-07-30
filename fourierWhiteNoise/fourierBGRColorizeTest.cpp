#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/core/mat.hpp"
#include <iostream>
#include <string>
#include <sstream>
#include <cmath>
#include <complex>
#include "dftMod.h"
using namespace std;
using namespace cv;

int main(int argc, char** argv){
   
    int c = 1;

    Mat image = imread("./duck.jpg", 1);
    cout<<siganal::typeID(image.type())<<"\n";
    cvtColor(image, image, CV_BGR2GRAY);

    int optW = getOptimalDFTSize(image.cols); 
    int optH = getOptimalDFTSize(image.rows);
    copyMakeBorder(image, image, 0, optH-image.rows, 0, optW-image.cols, BORDER_CONSTANT, Scalar::all(0));

    Mat planes[] = {Mat_<float>(image), Mat::zeros(image.size(), CV_32F)};
    Mat complexIm;

    merge(planes, 2, complexIm);
    dft(complexIm, complexIm);
    split(complexIm, planes);

    Mat magnitudeMat;
    Mat phaseMat;
    magnitude(planes[0], planes[1], magnitudeMat);
    phase(planes[0], planes[1], phaseMat);

    magnitudeMat += Scalar::all(1);
    phaseMat += Scalar::all(1);
    log(magnitudeMat, magnitudeMat);
    log(phaseMat, phaseMat);
    normalize(magnitudeMat, magnitudeMat, 0, 1, CV_MINMAX);
    normalize(phaseMat, phaseMat, 0, 1, CV_MINMAX);

    Mat outputImage;
    Mat imArray[] = {Mat_<float>(phaseMat), Mat::zeros(magnitudeMat.size(), CV_32F), Mat_<float>(magnitudeMat)};
    /*cout
        <<"MatCharImag: "<<imArray[0].size<<" "<<imArray[0].depth()<<", \n"
        <<"MatCharReal: "<<imArray[2].size<<" "<<imArray[2].depth()<<", \n"
        <<"MatCharZeros: "<<imArray[1].size<<" "<<imArray[1].depth()<<".\n";
    cout<<"CV8U: "<<CV_8U<<", \n"
        <<"CV8S: "<<CV_8S<<", \n"
        <<"CV16U: "<<CV_16U<<", \n"
        <<"CV16S: "<<CV_16S<<", \n";*/
    merge(imArray, 3, outputImage);
     
    namedWindow("output");

    cout<<siganal::typeID(outputImage.type())<<"\n";
    while(c != 0){
       imshow("output", outputImage);
        c = waitKey(0);
    }

    return 0;
}
