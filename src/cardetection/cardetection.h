#ifndef CAR_DETECTION
#define CAR_DETECTION

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "blob.h"

class CarDetection
{
public:
    enum VerticalMotionDirection {
        Top2Bottom = 1,
        Bottom2Top = 2 
    };
    static cv::Scalar SCALAR_BLACK;
    static cv::Scalar SCALAR_WHITE;
    static cv::Scalar SCALAR_YELLOW;
    static cv::Scalar SCALAR_GREEN;
    static cv::Scalar SCALAR_RED;

    static std::vector<cv::Point> CAMERA_1_SEARCH_AREA_POLYGON;
    static double CAMERA_1_HORIZONTAL_LINE_POSITION;
    static VerticalMotionDirection CAMERA_1_VERTICAL_MOTION_DIRECTON;
    static int CAMERA_1_MIN_CAR_AREA;

    static std::vector<cv::Point> CAMERA_2_SEARCH_AREA_POLYGON;
    static double CAMERA_2_HORIZONTAL_LINE_POSITION;
    static VerticalMotionDirection CAMERA_2_VERTICAL_MOTION_DIRECTON;
    static int CAMERA_2_MIN_CAR_AREA;

    static void matchCurrentFrameBlobsToExistingBlobs(std::vector<Blob> &existingBlobs, std::vector<Blob> &currentFrameBlobs);
    static void addBlobToExistingBlobs(Blob &currentFrameBlob, std::vector<Blob> &existingBlobs, int &intIndex);
    static void addNewBlob(Blob &currentFrameBlob, std::vector<Blob> &existingBlobs);
    static double distanceBetweenPoints(cv::Point point1, cv::Point point2);
    static void drawAndShowContours(cv::Size imageSize, std::vector<std::vector<cv::Point> > contours, std::string strImageName);
    static void drawAndShowContours(cv::Size imageSize, std::vector<Blob> blobs, std::string strImageName);
    static bool checkIfBlobsCrossedTheLine(std::vector<Blob> &blobs, int &intHorizontalLinePosition, int &carCount, VerticalMotionDirection direction = VerticalMotionDirection::Bottom2Top);
    static bool checkIfBlobsCrossedTheLine2(std::vector<Blob> &blobs, int &intHorizontalLinePosition, int &carCount);
    static void drawBlobInfoOnImage(std::vector<Blob> &blobs, cv::Mat &imgFrame2Copy);
    static void drawCarCountOnImage(int &carCount, cv::Mat &imgFrame2Copy);
    static bool checkIfBlobInsideSearchPolygon(cv::Rect &rect);
};

#endif