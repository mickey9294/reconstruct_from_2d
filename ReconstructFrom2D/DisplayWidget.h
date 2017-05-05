#pragma once

#include <QWidget>
#include <QPixmap>

#include <memory>
#include <vector>
#include <Eigen\Core>

#include "MyGLWidget.h"
#include "MeshModel.h"

class DisplayWidget : public QWidget
{
	Q_OBJECT

public:
	DisplayWidget(QWidget *parent = 0);
	~DisplayWidget();

	public slots:
	void set_model(MeshModel mesh, std::vector<Eigen::Vector2d> verts_2d, QPixmap image);

private:
	std::shared_ptr<MyGLWidget> glWidget_;

	std::shared_ptr<QVBoxLayout> central_layout;
};
