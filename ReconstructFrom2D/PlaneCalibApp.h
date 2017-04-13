#pragma once

#include <QStringList>
#include <planecalib\Profiler.h>
#include <planecalib\PlaneCalibSystem.h>
#include <planecalib\HomographyCalibration.h>
#include <planecalib\PoseTracker.h>
#include <planecalib\CameraModel.h>
#include <Eigen\Core>

#include <opencv\highgui.h>

#include <vector>
#include <list>
#include <string>
#include <ctime>
#include <chrono>
#include <memory>

class CalibrationError
{
public:
	void compute(const planecalib::CameraModel &ref, const planecalib::CameraModel &exp)
	{
		errorFocal = (exp.getFocalLength() - ref.getFocalLength()).norm();
		errorP0 = (exp.getPrincipalPoint() - ref.getPrincipalPoint()).norm();
		errorDist0 = exp.getDistortionModel().getLambda() - ref.getDistortionModel().getLambda();
	}

	float errorFocal;
	float errorP0;
	float errorDist0;
};

class PlaneCalibApp
{
public:
	PlaneCalibApp();
	PlaneCalibApp(const QStringList &images_path_list);
	~PlaneCalibApp();

	void process_images();
	void calibrate(Eigen::Vector2f &primary_point, Eigen::Vector2f &focal_length);
	void set_images_path_list(const QStringList &images_path_list);

private:
	std::shared_ptr<planecalib::PlaneCalibSystem> system_;
	std::list<std::string> images_path_list_;
	int frame_count_;

	void init();
	double current_time();
};

