#pragma once

#include <QtWidgets/QMainWindow>
#include <qlistwidget.h>
#include <qopenglwidget.h>
#include <qpushbutton.h>
#include <qfiledialog.h>
#include <qpropertyanimation.h>
#include <qgraphicsview.h>
#include <QGraphicsPixmapItem>
#include <QMessageBox>
#include <QFile>
#include <QDir>
#include <QtGui>
#include <QPainter>
#include <QTabWidget>

#include <memory>
#include <list>

#include <boost\filesystem.hpp>

#include "PlaneCalibApp.h"
#include "Reconstructor.h"
#include "PCModel.h"
#include "ArtecProcessor.h"
#include "MarkGraphicsScene.h"
#include "MarkWidget.h"
#include "DisplayWidget.h"

class ReconstructFrom2D : public QMainWindow
{
	Q_OBJECT

public:
	ReconstructFrom2D(QWidget *parent = Q_NULLPTR);

	public slots:
	//void load_images();
	//void show_big_image(QListWidgetItem *item);
	//void calibrate_camera();
	//void reconstruct();
	void post_process();
	void recon_finished();
	void slot_resize(int dw, int dh);
	void receive_status(QString msg);

signals:
	void set_images_path_list(QStringList list);

private:
	//Ui::ReconstructFrom2DClass ui;
	//QStringList images_path_list_;
	std::shared_ptr<QGraphicsScene> progress_img_scene_;
	std::shared_ptr<QGraphicsPixmapItem> progress_img_item_;
	std::shared_ptr<PlaneCalibApp> calib_app_;
	std::shared_ptr<ArtecProcessor> artec_processor_;

	Eigen::Vector2d primary_point_;
	Eigen::Vector2d focal_length_;
	std::shared_ptr<Reconstructor> reconstructor_;

	std::shared_ptr<QTabWidget> tabWidget_;
	std::shared_ptr<MarkWidget> markWidget_;
	std::shared_ptr<DisplayWidget> displayWidget_;

	std::shared_ptr<QMenu> menuFile;
	std::shared_ptr<QAction> actionOpen;
	std::shared_ptr<QAction> actionSave;
	std::shared_ptr<QAction> actionExit;
	std::shared_ptr<QMenu> menuScene;
	std::shared_ptr<QAction> actionLoadState;
	std::shared_ptr<QAction> actionSaveState;
	std::shared_ptr<QAction> actionUpdateScene;
	
	//void set_image_thumbnails();
	void initUI();
};
