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

    if(argc > 1){
        IM_P = argv[1];
    }
    else{
        IM_P = "./duck.jpg";
    }

    Mat im = imread(IM_P, 1);
    cvtColor(im, im, CV_BGR2GRAY); //Only want to deal with one channel

    int optW = getOptimalDFTSize(im.cols); //DFT works best on certain image sizes; this calculates the optimal number closest to a given integer representing width or height
    int optH = getOptimalDFTSize(im.rows);
    copyMakeBorder(im, im, 0, optH-im.rows, 0, optW-im.cols, BORDER_CONSTANT, Scalar::all(0)); //Buffering the border on the bottom and right

    Mat planes[] = {Mat_<float>(im), Mat::zeros(im.size(), CV_32F)}; //Two matrices: A copy of the image, but in float form (?), and one filled with zeroes.
    Mat complexIm;
    merge(planes, 2, complexIm); //2-channel image time

    dft(complexIm, complexIm); //Uh... I think because the second channel is all zeroes, it doesn't affect the DFT?

    Mat magn; //Magnitude vs. Phase

    split(complexIm, planes);

    magnitude(planes[0], planes[1], magn);

    magn += Scalar::all(1); //Can't log 0
    log(magn, magn); //Log scale to reduce from basically all 0/1

    normalize(magn, magn, 0, 1, CV_MINMAX); //Still too big to visualize without normalization

    namedWindow("shaped");

    int c;
    while(true){
        imshow("shaped", magn);
        c = waitKey(10);
   }
  return 0;
}
