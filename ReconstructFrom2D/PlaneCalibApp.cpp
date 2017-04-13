#include "PlaneCalibApp.h"

#undef GFLAGS_DLL_DEFINE_FLAG
#define GFLAGS_DLL_DEFINE_FLAG
#include <gflags\gflags.h>

namespace planecalib
{
	//	///////////////////////////////////////////////////////
	//
	DEFINE_int32(PyramidMaxTopLevelWidth, 960, "Maximum width of the highest pyramid level for a frame.");
	DEFINE_int32(SBIMaxWidth, 120, "Maximum width for the Small Blurry Image, input will be downsampled until width is less than this.");
	DEFINE_int32(FeatureDetectorThreshold, 10, "Threshold for the keypoint detector");
	DEFINE_int32(MatcherPixelSearchDistance, 8, "The search distance for matching features (distance from point projection or from epiplar line). Units in pixels of the highest pyramid level.");
	//
	//
	DEFINE_int32(CameraId, 0, "Id of the camera to open (OpenCV).");
	//DEFINE_string(VideoFile, "", "Name of the video file to use (e.g. rotation3.mp4). If both VideoFile and SequenceFormat are empty, the camera is used.");
	//DEFINE_string(ImageSequenceFormat, "", "sprintf format for the sequence (e.g. \"/cityOfSights/CS_BirdsView_L0/Frame_%.5d.jpg\". This is appended to the data path. If both VideoFile and SequenceFormat are empty, the camera is used.");
	DEFINE_int32(ImageSequenceStartIdx, 0, "Start index for the image sequence.");
	DEFINE_int32(DropFrames, 0, "The system will ignore this many frames per iteration, effectively lowering the frame rate or skipping frames in a video.");
	DEFINE_int32(InputMaxImageWidth, 1920, "Maximum width of input image. Input will be downsampled to be under this width.");
	DEFINE_bool(SingleThreaded, false, "Use a single thread for easier debugging.");
	//DEFINE_string(RecordPath, "record/", "Path where the frames will be stored in case of recording.");
	//DEFINE_string(RecordVideoFile, "video.avi", "Output video file for recording.");

	///////////////////////////////////////////////////////

	//DEFINE_int32(WindowWidth, 1280, "Initial width of the window.");
	//DEFINE_int32(WindowHeight, 960, "Initial height of the window.");
	DEFINE_int32(WindowWidth, 800, "Initial width of the window.");
	DEFINE_int32(WindowHeight, 600, "Initial height of the window.");
	//
}

PlaneCalibApp::PlaneCalibApp()
{
	init();
}

PlaneCalibApp::PlaneCalibApp(const QStringList & images_path_list)
{
	for (QStringList::const_iterator it = images_path_list.begin(); it != images_path_list.end(); ++it)
		images_path_list_.push_back(it->toStdString());

	init();
}


PlaneCalibApp::~PlaneCalibApp()
{
}

void PlaneCalibApp::init()
{
	system_.reset(new planecalib::PlaneCalibSystem());
	if (!images_path_list_.empty())
	{
		IplImage * color_img = cvLoadImage(images_path_list_.front().c_str());
		IplImage * gray_img = cvLoadImage(images_path_list_.front().c_str(), CV_LOAD_IMAGE_GRAYSCALE);
		cv::Mat1b imageGray = cv::cvarrToMat(gray_img);
		cv::Mat3b imageColor = cv::cvarrToMat(color_img);

		system_->init(current_time(), imageColor, imageGray);

	}
	else
	{
		IplImage * color_img = cvCreateImage(cvSize(4032, 3024), IPL_DEPTH_8U, 3);
		IplImage *gray_img = cvCreateImage(cvSize(4032, 3024), IPL_DEPTH_8U, 1);
		cv::Mat1b imageGray = cv::cvarrToMat(gray_img);
		cv::Mat3b imageColor = cv::cvarrToMat(color_img);
		system_->init(current_time(), imageColor, imageGray);
	}
	system_->setSingleThreaded(false);

	frame_count_ = 0;
}

void PlaneCalibApp::process_images()
{
	std::list<std::string>::iterator images_it = images_path_list_.begin();
	if (images_it != images_path_list_.end())
	{
		images_it++;
		frame_count_++;
	}
	for (; images_it != images_path_list_.end(); ++images_it)
	{
		std::string img_path = *images_it;
		IplImage * color_img = cvLoadImage(img_path.c_str(), CV_LOAD_IMAGE_COLOR);
		IplImage * gray_img = cvLoadImage(img_path.c_str(), CV_LOAD_IMAGE_GRAYSCALE);
		cv::Mat1b imageGray = cv::cvarrToMat(gray_img);
		cv::Mat3b imageColor = cv::cvarrToMat(color_img);
		system_->processImage(current_time(), imageColor, imageGray);
		frame_count_++;
	}
}

void PlaneCalibApp::calibrate(Eigen::Vector2f &primary_point, Eigen::Vector2f &focal_length)
{
	planecalib::CameraModel camera;
	camera.init(Eigen::Vector2f(2014.3, 1666.6),
		Eigen::Vector2f(2358.0, 2367.9), Eigen::Vector2i(4032, 3024));
	camera.getDistortionModel().init(0.1);

	float noiseStd = 3 / 3;

	CalibrationError error;

	//system_->setExpectedPixelNoiseStd(std::max(3 * noiseStd, 0.3f));
	system_->setUse3DGroundTruth(false);
	system_->setFix3DPoints(false);
	system_->doHomographyBA();
	system_->doHomographyCalib(true);
	system_->doFullBA();

	error.compute(camera, system_->getCamera());
	std::cout << "errorFocal: " << error.errorFocal << std::endl;
	std::cout << "errorP0: " << error.errorP0 << std::endl;
	std::cout << "errorDist0: " << error.errorDist0 << std::endl;

	primary_point = system_->getCamera().getPrincipalPoint();
	focal_length = system_->getCamera().getFocalLength();
	if (focal_length[0] != focal_length[0])
		focal_length.setZero();
}

void PlaneCalibApp::set_images_path_list(const QStringList & images_path_list)
{
	for (QStringList::const_iterator it = images_path_list.begin(); it != images_path_list.end(); ++it)
		images_path_list_.push_back(it->toStdString());

	init();
}

double PlaneCalibApp::current_time()
{
	std::chrono::milliseconds ms = std::chrono::duration_cast<std::chrono::milliseconds>(
		std::chrono::system_clock::now().time_since_epoch());
	return (double)ms.count();
}