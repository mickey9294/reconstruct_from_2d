#include "DisplayWidget.h"

DisplayWidget::DisplayWidget(QWidget *parent)
	: QWidget(parent)
{
	//glWidget_.reset(new MyGLWidget(this));

	central_layout.reset(new QVBoxLayout());
	//central_layout->addWidget(glWidget_.get());

	setLayout(central_layout.get());
}

DisplayWidget::~DisplayWidget()
{
}

void DisplayWidget::set_model(MeshModel mesh, std::vector<Eigen::Vector2d> verts_2d, QPixmap image)
{
	if(glWidget_)
		central_layout->removeWidget(glWidget_.get());

	glWidget_.reset(new MyGLWidget(mesh, verts_2d, image, this));
	central_layout->addWidget(glWidget_.get());
}
