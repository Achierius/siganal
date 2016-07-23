#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/core/mat.hpp"
#include <iostream>
#include <string>
#include <sstream>
#include <cmath>
using namespace std;
using namespace cv;

int main(int argc, char** argv){
    int rows = 1000;
    int cols = 1000;
    int stDev = 1;
    int mean = 120;

    stringstream s;

    if(argc > 2){
        stDev = stoi(argv[1]);
        mean = stoi(argv[2]);
    }
    else if(argc == 2){
        stDev = stoi(argv[1]);
    }

    int sz[] = {rows, cols};
    Mat im(2, sz, CV_8U, Scalar::all(0));
    randn(im, mean, stDev);

    int c;

    namedWindow("white noise");
    imshow("white noise", im);
    
    c = waitKey(0);


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

    while(true){
        imshow("shaped", magn);
        c = waitKey(10);
   }
  return 0;
}
