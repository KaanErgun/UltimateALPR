#ifndef PLATERECOGNIZER_DEFINES
#define PLATERECOGNIZER_DEFINES

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#define CV_CHAIN_APPROX_NONE cv::CHAIN_APPROX_NONE
#define CV_CHAIN_APPROX_SIMPLE cv::CHAIN_APPROX_SIMPLE 
#define CV_RETR_LIST cv::RETR_LIST
#define CV_THRESH_BINARY cv::THRESH_BINARY
#define CV_THRESH_BINARY_INV cv::THRESH_BINARY_INV
#define CV_THRESH_OTSU cv::THRESH_OTSU 
#define CV_GRAY2BGR cv::COLOR_GRAY2BGR
#define CV_BGR2HSV cv::COLOR_BGR2HSV
#define CV_ADAPTIVE_THRESH_GAUSSIAN_C cv::ADAPTIVE_THRESH_GAUSSIAN_C
//#define SHOW_STEPS

namespace PlateRecognizer {
	const cv::Scalar SCALAR_BLACK = cv::Scalar(0.0, 0.0, 0.0);
	const cv::Scalar SCALAR_WHITE = cv::Scalar(255.0, 255.0, 255.0);
	const cv::Scalar SCALAR_YELLOW = cv::Scalar(0.0, 255.0, 255.0);
	const cv::Scalar SCALAR_GREEN = cv::Scalar(0.0, 255.0, 0.0);
	const cv::Scalar SCALAR_RED = cv::Scalar(0.0, 0.0, 255.0);
}

#endif // PLATERECOGNIZER_DEFINES
