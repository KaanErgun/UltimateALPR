#include "recognizertool.h"
using namespace cv;
extern cv::Mat plate_img;

bool RecognizerTool::settingsApplied = false;
bool RecognizerTool::applySettings()
{
    bool blnKNNTrainingSuccessful = loadKNNDataAndTrainKNN(); // attempt KNN training

    if (blnKNNTrainingSuccessful == false)
    { // if KNN training was not successful
        std::cout << std::endl
                  << std::endl
                  << "error: error: KNN traning was not successful" << std::endl
                  << std::endl;
        return false; // and exit program
    }

    settingsApplied = true;

    return true;
}

void RecognizerTool::drawRedRectangleAroundPlate(cv::Mat &imgOriginalScene, PossiblePlate &licPlate)
{
    cv::Point2f p2fRectPoints[4];

    licPlate.rrLocationOfPlateInScene.points(p2fRectPoints); // get 4 vertices of rotated rect

    for (int i = 0; i < 4; i++)
    { // draw 4 red lines
        cv::line(imgOriginalScene, p2fRectPoints[i], p2fRectPoints[(i + 1) % 4], PlateRecognizer::SCALAR_RED, 2);
    }
}

void RecognizerTool::writeLicensePlateCharsOnImage(cv::Mat &imgOriginalScene, PossiblePlate &licPlate)
{
    cv::Point ptCenterOfTextArea;    // this will be the center of the area the text will be written to
    cv::Point ptLowerLeftTextOrigin; // this will be the bottom left of the area that the text will be written to

    int intFontFace = cv::FONT_HERSHEY_SIMPLEX;                  // choose a plain jane font
    double dblFontScale = (double)licPlate.imgPlate.rows / 30.0; // base font scale on height of plate area
    int intFontThickness = (int)std::round(dblFontScale * 1.5);  // base font thickness on font scale
    int intBaseline = 0;

    cv::Size textSize = cv::getTextSize(licPlate.strChars, intFontFace, dblFontScale, intFontThickness, &intBaseline); // call getTextSize

    ptCenterOfTextArea.x = (int)licPlate.rrLocationOfPlateInScene.center.x; // the horizontal location of the text area is the same as the plate

    if (licPlate.rrLocationOfPlateInScene.center.y < (imgOriginalScene.rows * 0.75))
    { // if the license plate is in the upper 3/4 of the image
      // write the chars in below the plate
        ptCenterOfTextArea.y = (int)std::round(licPlate.rrLocationOfPlateInScene.center.y) + (int)std::round((double)licPlate.imgPlate.rows * 1.6);
    }
    else
    { // else if the license plate is in the lower 1/4 of the image
      // write the chars in above the plate
        ptCenterOfTextArea.y = (int)std::round(licPlate.rrLocationOfPlateInScene.center.y) - (int)std::round((double)licPlate.imgPlate.rows * 1.6);
    }

    ptLowerLeftTextOrigin.x = (int)(ptCenterOfTextArea.x - (textSize.width / 2));  // calculate the lower left origin of the text area
    ptLowerLeftTextOrigin.y = (int)(ptCenterOfTextArea.y + (textSize.height / 2)); // based on the text area center, width, and height

    // write the text on the image
    cv::putText(imgOriginalScene, licPlate.strChars, ptLowerLeftTextOrigin, intFontFace, dblFontScale, PlateRecognizer::SCALAR_YELLOW, intFontThickness);
}

RecognizerTool::DetectStatus RecognizerTool::detect_plate_on_image(cv::Mat &imgOriginalScene, std::vector<DetectedPlate> &plates)
{
    cv::Mat sceneCopy = imgOriginalScene.clone();

    if (!settingsApplied)
        return RecognizerTool::DetectStatus::SettingsNotYetApplied;

    if (imgOriginalScene.empty())
    {
        return RecognizerTool::DetectStatus::FrameIsEmpty;
    }

    std::vector<PossiblePlate> vectorOfPossiblePlates = detectPlatesInScene(imgOriginalScene);

    vectorOfPossiblePlates = detectCharsInPlates(vectorOfPossiblePlates);

    if (vectorOfPossiblePlates.empty())
    {
        return RecognizerTool::DetectStatus::PlateWasNotFound;
    }
    else
    {
        std::sort(vectorOfPossiblePlates.begin(), vectorOfPossiblePlates.end(), PossiblePlate::sortDescendingByNumberOfChars);

        for (auto _posPlate : vectorOfPossiblePlates)
        {
            if (_posPlate.strChars.length() < 3) continue;

            DetectedPlate dPlate;

            dPlate.plateStr = _posPlate.strChars;
            dPlate.plateImg = sceneCopy(_posPlate.rrLocationOfPlateInScene.boundingRect());

            plates.push_back(dPlate);

            RecognizerTool::drawRedRectangleAroundPlate(imgOriginalScene, _posPlate);
            RecognizerTool::writeLicensePlateCharsOnImage(imgOriginalScene, _posPlate);
        }

        if (plates.size() == 0)
        {
            return RecognizerTool::DetectStatus::PlateWasNotFound;
        }

        return RecognizerTool::DetectStatus::PlateWasFound;
    }
}
