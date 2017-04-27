#pragma once

#include <QWidget>

#include <memory>

#include "MyGLWidget.h"

class DisplayWidget : public QWidget
{
	Q_OBJECT

public:
	DisplayWidget(QWidget *parent = 0);
	~DisplayWidget();

private:
	std::shared_ptr<MyGLWidget> glWidget_;

	std::shared_ptr<QVBoxLayout> central_layout;
};
