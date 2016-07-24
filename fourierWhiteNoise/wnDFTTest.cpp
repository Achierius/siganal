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

    Mat image = imread("./duck.jpg", 1);
    Mat imageNoised;
    Mat imageDFTMag;
    Mat imageNoisedDFTMag;

    
    cvtColor(image, image, CV_BGR2GRAY); //Only want to deal with one channel

    /*
     * Generate White Noise
     */
    int rows = image.rows;
    int cols = image.cols;
    int stDev = 40;
    int mean = 120;

    int sz[] = {rows, cols};
    Mat noise(2, sz, CV_8U, Scalar::all(0));
    randn(noise, mean, stDev);


    /*
     * Combine with target image
     */
    
    imageNoised = image+noise*0.25;

    /*
     * Generate DFT of target image before noise
     */


    int optW = getOptimalDFTSize(image.cols); //DFT works best on certain image sizes; this calculates the optimal number closest to a given integer representing width or height
    int optH = getOptimalDFTSize(image.rows);
    copyMakeBorder(image, image, 0, optH-image.rows, 0, optW-image.cols, BORDER_CONSTANT, Scalar::all(0)); //Buffering the border on the bottom and right

    Mat planes[] = {Mat_<float>(image), Mat::zeros(image.size(), CV_32F)}; //Two matrices: A copy of the image, but in float form (?), and one filled with zeroes.
    Mat complexIm;
    merge(planes, 2, complexIm); //2-channel image time

    dft(complexIm, complexIm); //Uh... I think because the second channel is all zeroes, it doesn't affect the DFT?

    split(complexIm, planes);

    magnitude(planes[0], planes[1], imageDFTMag);

    imageDFTMag += Scalar::all(1); //Can't log 0
    log(imageDFTMag, imageDFTMag); //Log scale to reduce from basically all 0/1

    normalize(imageDFTMag, imageDFTMag, 0, 1, CV_MINMAX); //Still too big to visualize without normalization

    /*
     * Generate DFT of noised target
     */
    copyMakeBorder(imageNoised, imageNoised, 0, getOptimalDFTSize(imageNoised.rows)-image.rows, 0, getOptimalDFTSize(imageNoised.cols)-imageNoised.cols, BORDER_CONSTANT, Scalar::all(0));

    planes[0] = Mat_<float>(imageNoised);
    planes[1] = Mat::zeros(imageNoised.size(), CV_32F);
    merge(planes, 2, complexIm);

    dft(complexIm, complexIm);

    split(complexIm, planes);

    magnitude(planes[0], planes[1], imageNoisedDFTMag);

    imageNoisedDFTMag += Scalar::all(1);
    log(imageNoisedDFTMag, imageNoisedDFTMag);

    normalize(imageNoisedDFTMag, imageNoisedDFTMag, 0, 1, CV_MINMAX);

    /*
     * Display
     */

    namedWindow("Noise-DFT");
    namedWindow("Noise");
    namedWindow("Image-DFT");
    namedWindow("Image");

    int c = 1;
    while(c!=0){
        imshow("Noise-DFT", imageNoisedDFTMag);
        imshow("Image-DFT", imageDFTMag);
        imshow("Noise", imageNoised);
        imshow("Image", image);
        c = waitKey(10);
    }
    return 0;
}
