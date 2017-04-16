#pragma once

#include <QWidget>
#include <qpushbutton.h>
#include <QLabel>
#include <qlayout.h>
#include <QListWidget>
#include <QFileDialog>

#include <memory>

#include "MarkGraphicsScene.h"
#include "FacesIdentifier.h"

class MarkWidget : public QWidget
{
	Q_OBJECT

public:
	MarkWidget(QWidget *parent = NULL);
	~MarkWidget();
	
	public slots:
	void set_images_path(QStringList list);
	void reset_display();
	void load_images();
	void set_mark_image(QListWidgetItem *item);
	void detect_planes();

signals:
	void set_image(QString image_path);

private:
	std::shared_ptr<MarkGraphicsScene> displayWidget_;
	std::shared_ptr<QPushButton> undoButton_;
	std::shared_ptr<QPushButton> loadButton_;
	std::shared_ptr<QPushButton> resetButton_;
	std::shared_ptr<QPushButton> detectPlanesButton_;
	std::shared_ptr<QListWidget> imagesListWidget_;

	QStringList images_path_list_;

	void set_image_thumbnails();
};
