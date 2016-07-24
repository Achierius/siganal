#include "dftMod.h"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/core/mat.hpp"


void siganal::visualizeFDomain(cv::Mat& targetMatrix){
    cv::cvtColor(targetMatrix, targetMatrix, CV_BGR2GRAY) ;

    cv::copyMakeBorder(targetMatrix, targetMatrix, 0, cv::getOptimalDFTSize(targetMatrix.rows)-targetMatrix.rows, 0, cv::getOptimalDFTSize(targetMatrix.cols)-targetMatrix.cols, cv::BORDER_CONSTANT, cv::Scalar::all(0));

    cv::Mat planes[] = {cv::Mat_<float>(targetMatrix), cv::Mat::zeros(targetMatrix.size(), CV_32F)}; 
    cv::Mat complexIm;
    cv::merge(planes, 2, complexIm);

    cv::dft(complexIm, complexIm);

    cv::split(complexIm, planes);
    cv::magnitude(planes[0], planes[1], targetMatrix);

    targetMatrix += cv::Scalar::all(1);
    cv::log(targetMatrix, targetMatrix);
    cv::normalize(targetMatrix, targetMatrix, 0, 1, CV_MINMAX);
}
