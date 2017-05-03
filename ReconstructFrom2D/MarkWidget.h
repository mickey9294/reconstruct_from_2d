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
#include "ConstraintsGenerator.h"
#include "VertexRecognition.h"

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
	void generate_constraints();
	void save_scene_state();
	void load_scene_state();
	void update_scene();
	void recon_from_file();
	void recognize_precise_vertices();

signals:
	void set_image(QString image_path);
	void change_label_state();
	void stop_labeling();
	
protected:
	void keyPressEvent(QKeyEvent *event);

private:
	std::shared_ptr<MarkGraphicsScene> displayWidget_;
	std::shared_ptr<QPushButton> undoButton_;
	std::shared_ptr<QPushButton> loadButton_;
	std::shared_ptr<QPushButton> resetButton_;
	std::shared_ptr<QPushButton> recognitionButton_;
	std::shared_ptr<QPushButton> detectPlanesButton_;
	std::shared_ptr<QPushButton> addConstraintsButton_;
	std::shared_ptr<QPushButton> reconButton_;
	std::shared_ptr<QListWidget> imagesListWidget_;
	std::shared_ptr<ConstraintsGenerator> constraints_generator_;

	std::shared_ptr<QLabel> image_list_label;
	std::shared_ptr<QVBoxLayout> control_layout;
	std::shared_ptr<QHBoxLayout> load_layout;
	std::shared_ptr<QVBoxLayout> display_layout;
	std::shared_ptr<QHBoxLayout> undo_layout;
	std::shared_ptr<QHBoxLayout> central_layout;

	QStringList images_path_list_;

	void set_image_thumbnails();
};
