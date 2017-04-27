#include "ReconstructFrom2D.h"

ReconstructFrom2D::ReconstructFrom2D(QWidget *parent)
	: QMainWindow(parent)
{
	initUI();

	move(100, 100);

	connect(actionExit.get(), SIGNAL(triggered()), this, SLOT(close()));
	connect(actionOpen.get(), SIGNAL(triggered()), markWidget_.get(), SLOT(load_images()));
	connect(actionSaveState.get(), SIGNAL(triggered()), markWidget_.get(), SLOT(save_scene_state()));
	connect(actionLoadState.get(), SIGNAL(triggered()), markWidget_.get(), SLOT(load_scene_state()));
	connect(this, SIGNAL(set_images_path_list(QStringList)), markWidget_.get(), SLOT(set_images_path(QStringList)));
	connect(actionUpdateScene.get(), SIGNAL(triggered()), markWidget_.get(), SLOT(update_scene()));

	primary_point_.setZero();
	focal_length_.setZero();
}

void ReconstructFrom2D::initUI()
{
	tabWidget_.reset(new QTabWidget());
	markWidget_.reset(new MarkWidget(this));
	displayWidget_.reset(new DisplayWidget(this));
	tabWidget_->addTab(markWidget_.get(), "Start");
	tabWidget_->addTab(displayWidget_.get(), "Result");

	actionOpen.reset(new QAction(tr("&Open"), this));
	actionSave.reset(new QAction(tr("&Save"), this));
	actionExit.reset(new QAction(tr("&Exit"), this));
	actionSaveState.reset(new QAction(tr("&Save Scene State"), this));
	actionLoadState.reset(new QAction(tr("&Load Scene State"), this));
	actionUpdateScene.reset(new QAction(tr("&Update Scene"), this));
	menuFile.reset(menuBar()->addMenu(tr("&File")));
	menuScene.reset(menuBar()->addMenu(tr("&Scene")));
	menuFile->addAction(actionOpen.get());
	menuFile->addAction(actionSave.get());
	menuFile->addSeparator();
	menuFile->addAction(actionExit.get());
	menuScene->addAction(actionSaveState.get());
	menuScene->addAction(actionLoadState.get());
	menuScene->addAction(actionUpdateScene.get());

	// 设置显示当前布局
	setCentralWidget(tabWidget_.get());
	//this->setLayout(control_layout);
}

//void ReconstructFrom2D::calibrate_camera()
//{
//	if (!calib_app_)
//		calib_app_.reset(new PlaneCalibApp(images_path_list_));
//	else
//		calib_app_->set_images_path_list(images_path_list_);
//
//	calib_app_->process_images();
//	calib_app_->calibrate(primary_point_, focal_length_);
//
//	QMessageBox msgBox;
//	msgBox.setFixedWidth(700);
//	if (!focal_length_.isZero())
//		msgBox.setText("The camera calibration has done. Camera parameters are estimated successfully.");
//	else
//		msgBox.setText("Camera calibration failed for unknown reason!");
//	msgBox.exec();
//}

//void ReconstructFrom2D::reconstruct()
//{
//	//if (focal_length_.isZero())
//		//return;
//
//	if (!reconstructor_)
//	{
//		reconstructor_.reset(new Reconstructor(primary_point_, focal_length_));
//		connect(reconstructor_.get(), SIGNAL(progress_report(int)), this, SLOT(show_progress(int)));
//		connect(reconstructor_.get(), SIGNAL(reconstruct_finished()), this, SLOT(recon_finished()));
//	}
//	else
//	{
//		if (reconstructor_->isRunning())
//			reconstructor_->quit();
//		reconstructor_->set_cam_intrinsic_para(primary_point_, focal_length_);
//	}
//	
//	reconstructor_->set_images_path_list(images_path_list_);
//	reconstructor_->start();
//}


void ReconstructFrom2D::post_process()
{
	boost::filesystem::path frames_dir("..\\data\\3d_frames");

	boost::filesystem::recursive_directory_iterator it(frames_dir);
	boost::filesystem::recursive_directory_iterator end;

	std::list<std::string> frames_paths;

	while (it != end)
	{
		if (boost::filesystem::is_regular_file(*it)
			&& (it->path().extension() == ".ply" || it->path().extension() == ".off"))
			frames_paths.push_back(it->path().string());
		it++;
	}

	std::vector<std::shared_ptr<ShapeModel>> shapes(frames_paths.size());
	int idx = 0;
	for (std::list<std::string>::iterator frame_it = frames_paths.begin(); 
		frame_it != frames_paths.end(); ++frame_it, ++idx)
	{
		shapes[idx].reset(new PCModel(frame_it->c_str()));
	}

	if (!artec_processor_)
		artec_processor_.reset(new ArtecProcessor());

	artec_processor_->set_input_cloud(shapes);
	artec_processor_->pointcloud_register();
}

void ReconstructFrom2D::recon_finished()
{
	
}

void ReconstructFrom2D::slot_resize(int dw, int dh)
{
	int width = this->width();
	int height = this->height();
	width += dw;
	height += dh;
	this->resize(width, height);
	this->setGeometry(0, 0, width, height);
}

void ReconstructFrom2D::receive_status(QString msg)
{
	statusBar()->showMessage(msg);
}

//void ReconstructFrom2D::set_image_thumbnails()
//{
//	imagesListWidget_->clear();
//	imagesListWidget_->setViewMode(QListWidget::IconMode);
//	imagesListWidget_->setIconSize(QSize(200, 150));
//	imagesListWidget_->setResizeMode(QListWidget::Adjust);
//	int img_idx = 1;
//	for (QStringList::iterator img_it = images_path_list_.begin(); img_it != images_path_list_.end(); ++img_it, img_idx++)
//	{
//		QString img_title = "Image " + QString::number(img_idx);
//		imagesListWidget_->addItem(new QListWidgetItem(QIcon(img_it->toLocal8Bit().data()), img_title));
//	}
//}
