#include "dftMod.h"



void sigAnal::visualizeFDomain(cv::Mat& targetMatrix){
    cvtColor(targetMatrix, targetMatrix, CV_BGR2GRAY) ;

    copyMakeBorder(targetMatrix, targetMatrix, 0, getOptimalDFTSize(targetMatrix.rows)-targetMatrix.rows, 0, getOptimalDFTSize(targetMatrix.cols)-targetMatrix.cols, BORDER_CONSTANT, Scalar::all(0));

    Mat planes[] = {Mat_<float>(im), Mat::zeros(im.size(), CV_32F)}; 
    Mat complexIm;
    merge(planes, 2, complexIm);

    dft(complexI, complexIm);

    split(complexIm, planes);
    magnitude(planes[0], planes[1], targetMatrix);

    targetMatrix += Scalar::all(1);
    log(targetMatrix, targetMatrix);
    normalize(targetMatrix, targetMatrix, 0, 1, CV_MINMAX);
}
