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

std::string siganal::typeID(int type){
    std::string r;

  uchar depth = type & CV_MAT_DEPTH_MASK;
  uchar chans = 1 + (type >> CV_CN_SHIFT);

  switch ( depth ) {
      case CV_8U:  r = "8U"; break;
      case CV_8S:  r = "8S"; break;
      case CV_16U: r = "16U"; break;
      case CV_16S: r = "16S"; break;
      case CV_32S: r = "32S"; break;
      case CV_32F: r = "32F"; break;
      case CV_64F: r = "64F"; break;
      default:     r = "User"; break;
    }

  r += "C";
  r += (chans+'0');

  return r;
}
