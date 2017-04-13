#include "Reconstructor.h"



Reconstructor::Reconstructor(QObject *parent)
	: QThread(parent)
{
	ep = NULL;
	primary_point_[0] = 1512;
	primary_point_[1] = 2016;
	focal_length_[0] = 2000;
	focal_length_[1] = 2000;
	//init_engine();
}

Reconstructor::Reconstructor(Eigen::Vector2f primary_point, Eigen::Vector2f focal_length)
{
	ep = NULL;
	primary_point_ = primary_point;
	focal_length_ = focal_length;
	if (primary_point_.isZero())
	{
		primary_point_[0] = 1512;
		primary_point_[1] = 2016;
	}
	if (focal_length_.isZero())
	{
		focal_length_[0] = 2000;
		focal_length_[1] = 2000;
	}

	//init_engine();
}

Reconstructor::~Reconstructor()
{
	if (ep)
	{
		delete(ep);
		ep = NULL;
	}
}

void Reconstructor::reconstruct()
{
	if (!ep)
	{
		if (!(ep = engOpen("\0"))) //启动matlab 引擎
		{
			std::cerr << "Initilizing Reconstructor failed." << std::endl;
		}
		else
		{
			engSetVisible(ep, false); // 设置窗口不可见
		}
	}

	engEvalString(ep, "cd \'D:\\ProgrammingTools\\Matlab\\workspace\\Reconstruction\\';");
	engEvalString(ep, "addpath(\'D:\\ProgrammingTools\\Matlab\\workspace\\Reconstruction\'");
	engEvalString(ep, "reconstruct_process();");
	///* Set the camera instrinsic parameters */
	//if (focal_length_[0] < 100 || focal_length_[1] < 100)
	//{
	//	engEvalString(ep, "load(\'images\\cameraParams_reserve.mat\');");
	//}
	//else
	//{
	//	engEvalString(ep, "load(\'images\\camstruct.mat\');");
	//	std::string fx = std::to_string(focal_length_[0]);
	//	std::string fy = std::to_string(focal_length_[1]);
	//	std::string px = std::to_string(primary_point_[0]);
	//	std::string py = std::to_string(primary_point_[1]);

	//	std::string set_cam_struct_com1 = "camstruct.IntrinsicMatrix(1,1) = " + fx + ";";
	//	std::string set_cam_struct_com2 = "camstruct.IntrinsicMatrix(2,2) = " + fy + ";";
	//	std::string set_cam_struct_com3 = "camstruct.IntrinsicMatrix(3,1) = " + px + ";";
	//	std::string set_cam_struct_com4 = "camstruct.IntrinsicMatrix(3,2) = " + py + ";";
	//	engEvalString(ep, set_cam_struct_com1.c_str());
	//	engEvalString(ep, set_cam_struct_com2.c_str());
	//	engEvalString(ep, set_cam_struct_com3.c_str());
	//	engEvalString(ep, set_cam_struct_com4.c_str());
	//	engEvalString(ep, "cameraParams = cameraParameters(camstruct);");
	//	engEvalString(ep, "save(\'images\\cameraParams_test.mat\', \'cameraParams\');");
	//	engEvalString(ep, "load(\'images\\cameraParams_test.mat\');");
	//}
	//
	///* Load images and extract features points */
	//int num_images = images_path_list_.size();
	//std::string create_images_cell_cmd = "images = cell(1, " + std::to_string(num_images) + ");";
	////std::string create_undistort_cell_cmd = "images_undistort = cell(1, " + std::to_string(num_images) + ");";
	//std::string create_images_points_cell_cmd = "images_points = cell(1, " + std::to_string(num_images) + ");";
	//engEvalString(ep, create_images_cell_cmd.c_str());
	////engEvalString(ep, create_undistort_cell_cmd.c_str());
	//engEvalString(ep, create_images_points_cell_cmd.c_str());
	//int img_idx = 0;
	//for (QStringList::iterator path_it = images_path_list_.begin(); path_it != images_path_list_.end(); ++path_it, ++img_idx)
	//{
	//	/* Read image */
	//	QString read_img_cmd = "I = imread(\'" + *path_it + "\');";
	//	engEvalString(ep, read_img_cmd.toLocal8Bit().data());
	//	/* Undistort the image */
	//	engEvalString(ep, "I = undistortImage(I, cameraParams);");
	//	/* Store the image */
	//	std::string store_image_cmd = "images{" + std::to_string(img_idx) + "} = I;";
	//	engEvalString(ep, store_image_cmd.c_str());
	//	/* Compute corner points features of the image */
	//	std::string compute_corners_cmd = "images_points{" + std::to_string(img_idx) 
	//		+ "} = detectMinEigenFeatures(rgb2gray(I), \'MinQuality\', 0.001);";
	//	engEvalString(ep, compute_corners_cmd.c_str());
	//	/* Save the corner points features */
	//	QString file_name = QFileInfo(*path_it).baseName();
	//	QString features_file_path = "D:\\Projects\\ReconstructFrom2D\\data\\features\\" + file_name + ".txt";
	//	QString save_features_cmd = "save_cornerPoints(" + features_file_path 
	//		+ ", images_points{" + QString::number(img_idx) + "});";
	//	engEvalString(ep, save_features_cmd.toLocal8Bit().data());
	//} 
}

void Reconstructor::set_cam_intrinsic_para(Eigen::Vector2f primary_point, Eigen::Vector2f focal_length)
{
	primary_point_ = primary_point;
	focal_length_ = focal_length;
}

void Reconstructor::set_images_path_list(const QStringList & images_path_list)
{
	for (QStringList::const_iterator it = images_path_list.begin(); it != images_path_list.end(); ++it)
		images_path_list_.push_back(*it);
}

void Reconstructor::reconstruct_from_planes()
{

}

void Reconstructor::run()
{
	reconstruct();
}

void Reconstructor::init_engine()
{
	std::cout << "Initializing engine..." << std::endl;
	ep = NULL;
	if (!(ep = engOpen("\0"))) //启动matlab 引擎
	{
		std::cerr << "Initilizing Reconstructor failed." << std::endl;
	}
	if (ep)
	{
		std::cout << "Initialization done." << std::endl;
		engSetVisible(ep, false); // 设置窗口不可见
		//engEvalString(ep, "a = 3 + 5;");
		//engEvalString(ep, "dlmwrite(\'D:\\ProgrammingTools\\Matlab\\workspace\\test.txt\', a);");
	}
}
