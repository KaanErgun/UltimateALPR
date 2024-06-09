#include "cardetection.h"

#include "uuid.h"

using namespace cv;

cv::Scalar CarDetection::SCALAR_BLACK = cv::Scalar(0.0, 0.0, 0.0);
cv::Scalar CarDetection::SCALAR_WHITE = cv::Scalar(255.0, 255.0, 255.0);
cv::Scalar CarDetection::SCALAR_YELLOW = cv::Scalar(0.0, 255.0, 255.0);
cv::Scalar CarDetection::SCALAR_GREEN = cv::Scalar(0.0, 200.0, 0.0);
cv::Scalar CarDetection::SCALAR_RED = cv::Scalar(0.0, 0.0, 255.0);

std::vector<cv::Point> CarDetection::CAMERA_1_SEARCH_AREA_POLYGON;
std::vector<cv::Point> CarDetection::CAMERA_2_SEARCH_AREA_POLYGON;
double CarDetection::CAMERA_1_HORIZONTAL_LINE_POSITION = 0.5;
double CarDetection::CAMERA_2_HORIZONTAL_LINE_POSITION = 0.5;
CarDetection::VerticalMotionDirection CarDetection::CAMERA_1_VERTICAL_MOTION_DIRECTON = CarDetection::Top2Bottom;
CarDetection::VerticalMotionDirection CarDetection::CAMERA_2_VERTICAL_MOTION_DIRECTON = CarDetection::Bottom2Top;
int CarDetection::CAMERA_1_MIN_CAR_AREA = 50000;
int CarDetection::CAMERA_2_MIN_CAR_AREA = 50000;

///////////////////////////////////////////////////////////////////////////////////////////////////
void CarDetection::matchCurrentFrameBlobsToExistingBlobs(std::vector<Blob> &existingBlobs, std::vector<Blob> &currentFrameBlobs) {

    for (auto &existingBlob : existingBlobs) {

        existingBlob.blnCurrentMatchFoundOrNewBlob = false;

        existingBlob.predictNextPosition();
    }

    for (auto &currentFrameBlob : currentFrameBlobs) {

        int intIndexOfLeastDistance = 0;
        double dblLeastDistance = 100000.0;

        for (unsigned int i = 0; i < existingBlobs.size(); i++) {

            if (existingBlobs[i].blnStillBeingTracked == true) {

                double dblDistance = distanceBetweenPoints(currentFrameBlob.centerPositions.back(), existingBlobs[i].predictedNextPosition);

                if (dblDistance < dblLeastDistance) {
                    dblLeastDistance = dblDistance;
                    intIndexOfLeastDistance = i;
                }
            }
        }

        if (dblLeastDistance < currentFrameBlob.dblCurrentDiagonalSize * 0.5) {
            addBlobToExistingBlobs(currentFrameBlob, existingBlobs, intIndexOfLeastDistance);
        }
        else {
            addNewBlob(currentFrameBlob, existingBlobs);
        }

    }

    for (auto &existingBlob : existingBlobs) {

        if (existingBlob.blnCurrentMatchFoundOrNewBlob == false) {
            existingBlob.intNumOfConsecutiveFramesWithoutAMatch++;
        }

        if (existingBlob.intNumOfConsecutiveFramesWithoutAMatch >= 5) {
            existingBlob.blnStillBeingTracked = false;
        }

    }

}

///////////////////////////////////////////////////////////////////////////////////////////////////
void CarDetection::addBlobToExistingBlobs(Blob &currentFrameBlob, std::vector<Blob> &existingBlobs, int &intIndex) {

    existingBlobs[intIndex].currentContour = currentFrameBlob.currentContour;
    existingBlobs[intIndex].currentBoundingRect = currentFrameBlob.currentBoundingRect;

    existingBlobs[intIndex].centerPositions.push_back(currentFrameBlob.centerPositions.back());

    existingBlobs[intIndex].dblCurrentDiagonalSize = currentFrameBlob.dblCurrentDiagonalSize;
    existingBlobs[intIndex].dblCurrentAspectRatio = currentFrameBlob.dblCurrentAspectRatio;

    existingBlobs[intIndex].blnStillBeingTracked = true;
    existingBlobs[intIndex].blnCurrentMatchFoundOrNewBlob = true;
    // existingBlobs[intIndex].isFirstDetection = false;
}

///////////////////////////////////////////////////////////////////////////////////////////////////
void CarDetection::addNewBlob(Blob &currentFrameBlob, std::vector<Blob> &existingBlobs) {

    currentFrameBlob.blnCurrentMatchFoundOrNewBlob = true;

    //modified by melih
    currentFrameBlob.uuid = uuid::generate_uuid_v4();
    // currentFrameBlob.isFirstDetection = true;

    existingBlobs.push_back(currentFrameBlob);
}

///////////////////////////////////////////////////////////////////////////////////////////////////
double CarDetection::distanceBetweenPoints(cv::Point point1, cv::Point point2) {

    int intX = abs(point1.x - point2.x);
    int intY = abs(point1.y - point2.y);

    return(sqrt(pow(intX, 2) + pow(intY, 2)));
}

///////////////////////////////////////////////////////////////////////////////////////////////////
void CarDetection::drawAndShowContours(cv::Size imageSize, std::vector<std::vector<cv::Point> > contours, std::string strImageName) {
    cv::Mat image(imageSize, CV_8UC3, SCALAR_BLACK);

    cv::drawContours(image, contours, -1, SCALAR_WHITE, -1);
#ifdef STEPS
    cv::imshow(strImageName, image);
#endif
}

///////////////////////////////////////////////////////////////////////////////////////////////////
void CarDetection::drawAndShowContours(cv::Size imageSize, std::vector<Blob> blobs, std::string strImageName) {

    cv::Mat image(imageSize, CV_8UC3, SCALAR_BLACK);

    std::vector<std::vector<cv::Point> > contours;

    for (auto &blob : blobs) {
        if (blob.blnStillBeingTracked == true) {
            contours.push_back(blob.currentContour);
        }
    }

    cv::drawContours(image, contours, -1, SCALAR_WHITE, -1);

#ifdef STEPS
    cv::imshow(strImageName, image);
#endif
}

///////////////////////////////////////////////////////////////////////////////////////////////////
bool CarDetection::checkIfBlobsCrossedTheLine(std::vector<Blob> &blobs, int &intHorizontalLinePosition, int &carCount, VerticalMotionDirection direction) {
    bool blnAtLeastOneBlobCrossedTheLine = false;

    for (auto &blob : blobs) {

        if (blob.blnStillBeingTracked == true && blob.centerPositions.size() >= 2) {
            int prevFrameIndex = (int)blob.centerPositions.size() - 2;
            int currFrameIndex = (int)blob.centerPositions.size() - 1;

            bool isOk = false;
            if (direction == Top2Bottom) {
                isOk = (blob.centerPositions[prevFrameIndex].y < intHorizontalLinePosition && blob.centerPositions[currFrameIndex].y >= intHorizontalLinePosition);
            } else if (direction == Bottom2Top) {
                isOk = (blob.centerPositions[prevFrameIndex].y > intHorizontalLinePosition && blob.centerPositions[currFrameIndex].y <= intHorizontalLinePosition);
            }
            
            if (isOk) {
                carCount++;
                blnAtLeastOneBlobCrossedTheLine = true;
                blob.isCrossedTheLine = true;
            }
        }

    }

    return blnAtLeastOneBlobCrossedTheLine;
}

///////////////////////////////////////////////////////////////////////////////////////////////////
bool CarDetection::checkIfBlobsCrossedTheLine2(std::vector<Blob> &blobs, int &intHorizontalLinePosition, int &carCount) {
    bool blnAtLeastOneBlobCrossedTheLine = false;

    for (auto &blob : blobs) {
        if (blob.blnStillBeingTracked == true) {

            // is car above the line
            if (blob.currentBoundingRect.y < intHorizontalLinePosition && (blob.currentBoundingRect.y + blob.currentBoundingRect.height) > intHorizontalLinePosition) {
                blnAtLeastOneBlobCrossedTheLine = true;
                
                if (!blob.isAboveTheLine) {
                    carCount++;
                }
                
                blob.isAboveTheLine = true;
            }
        }

    }

    return blnAtLeastOneBlobCrossedTheLine;
}

///////////////////////////////////////////////////////////////////////////////////////////////////
void CarDetection::drawBlobInfoOnImage(std::vector<Blob> &blobs, cv::Mat &imgFrame2Copy) {

    for (unsigned int i = 0; i < blobs.size(); i++) {

        if (blobs[i].blnStillBeingTracked == true) {
            cv::rectangle(imgFrame2Copy, blobs[i].currentBoundingRect, SCALAR_RED, 2);

            //modified by melih 
            continue;
            int intFontFace = cv::FONT_HERSHEY_SIMPLEX;
            double dblFontScale = blobs[i].dblCurrentDiagonalSize / 60.0;
            int intFontThickness = (int)std::round(dblFontScale * 1.0);

            cv::putText(imgFrame2Copy, std::to_string(i), blobs[i].centerPositions.back(), intFontFace, dblFontScale, SCALAR_GREEN, intFontThickness);
        }
    }
}

///////////////////////////////////////////////////////////////////////////////////////////////////
void CarDetection::drawCarCountOnImage(int &carCount, cv::Mat &imgFrame2Copy) {

    int intFontFace = cv::FONT_HERSHEY_SCRIPT_COMPLEX;
    double dblFontScale = (imgFrame2Copy.rows * imgFrame2Copy.cols) / 900000.0;
    int intFontThickness = (int)std::round(dblFontScale * 1.5);

    cv::Size textSize = cv::getTextSize(std::to_string(carCount), intFontFace, dblFontScale, intFontThickness, 0);

    cv::Point ptTextBottomLeftPosition;

    ptTextBottomLeftPosition.x = imgFrame2Copy.cols - 1 - (int)((double)textSize.width * 1.25);
    ptTextBottomLeftPosition.y = (int)((double)textSize.height * 1.25);

    cv::putText(imgFrame2Copy, std::to_string(carCount), ptTextBottomLeftPosition, intFontFace, dblFontScale, SCALAR_RED, intFontThickness);

}

bool CarDetection::checkIfBlobInsideSearchPolygon(cv::Rect &rect)
{
    std::vector<cv::Point> corners;

    corners.push_back(cv::Point(rect.x, rect.y));
    corners.push_back(cv::Point(rect.x + rect.width, rect.y));
    corners.push_back(cv::Point(rect.x + rect.width, rect.y + rect.height));
    corners.push_back(cv::Point(rect.x, rect.y + rect.height));
    
    for (auto &cornerPoint : corners)
    {
        if (cv::pointPolygonTest(CAMERA_1_SEARCH_AREA_POLYGON, cornerPoint, false) == -1)  {
            return false;
        }
    }

    return true;
}