#include "DisplayWidget.h"

DisplayWidget::DisplayWidget(QWidget *parent)
	: QWidget(parent)
{
	glWidget_.reset(new MyGLWidget(this));

	central_layout.reset(new QVBoxLayout());
	central_layout->addWidget(glWidget_.get());

	setLayout(central_layout.get());
}

DisplayWidget::~DisplayWidget()
{
}
