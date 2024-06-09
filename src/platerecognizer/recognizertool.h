// main.h

#ifndef RECOGNIZER_TOOL
#define RECOGNIZER_TOOL

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <string>

#include "defines.h"
#include "PossiblePlate.h"
#include "DetectPlates.h"
#include "DetectChars.h"

class RecognizerTool
{

public:
	enum DetectStatus
	{
		PlateWasFound,
		SettingsNotYetApplied,
		FrameIsEmpty,
		PlateWasNotFound
	};

	struct DetectedPlate
	{
		std::string plateStr;
		cv::Mat plateImg;
	};
	

	static bool applySettings();

	static void drawRedRectangleAroundPlate(cv::Mat &imgOriginalScene, PossiblePlate &licPlate);
	static void writeLicensePlateCharsOnImage(cv::Mat &imgOriginalScene, PossiblePlate &licPlate);

	static RecognizerTool::DetectStatus detect_plate_on_image(cv::Mat &imgOriginalScene, std::vector<DetectedPlate> &plates);

private:
	static bool settingsApplied;
};

#endif
