#pragma once

#include <QThread>
#include <qfileinfo.h>
//#include <QMutex>
//#include <QWaitCondition>

#include <engine.h>
#include <iostream>

#include <Eigen\Core>

class Reconstructor : public QThread
{
	Q_OBJECT

public:
	Reconstructor(QObject *parent = 0);
	Reconstructor(Eigen::Vector2f primary_point, Eigen::Vector2f focal_length);
	~Reconstructor();

	void reconstruct();
	void set_cam_intrinsic_para(Eigen::Vector2f primary_point, Eigen::Vector2f focal_length);
	void set_images_path_list(const QStringList &images_path_list);
	void reconstruct_from_planes();

signals:
	void progress_report(int progress_id);
	void reconstruct_finished();

protected:
	void run();

private:
	Engine *ep;
	Eigen::Vector2f primary_point_;
	Eigen::Vector2f focal_length_;
	QStringList images_path_list_;

	void init_engine();
};
