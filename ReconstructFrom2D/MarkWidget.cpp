#include "MarkWidget.h"

MarkWidget::MarkWidget(QWidget *parent)
	: QWidget(parent)
{
	undoButton_.reset(new QPushButton("Undo"));
	loadButton_.reset(new QPushButton("Load Images"));
	resetButton_.reset(new QPushButton("Reset"));
	detectPlanesButton_.reset(new QPushButton("Detect Planes"));
	displayWidget_.reset(new MarkGraphicsScene());
	imagesListWidget_.reset(new QListWidget());
	imagesListWidget_->setFixedWidth(250);
	//imagesListWidget_->setFixedHeight(550);
	imagesListWidget_->setMinimumHeight(550);
	displayWidget_->resize(800, 600);


	QLabel* image_list_label = new QLabel("Images List");
	//image_list_label->setMaximumHeight(30);
	QVBoxLayout * control_layout = new QVBoxLayout();
	control_layout->addWidget(image_list_label);
	control_layout->addWidget(imagesListWidget_.get());
	control_layout->setAlignment(Qt::AlignTop);
	QHBoxLayout * load_layout = new QHBoxLayout();
	load_layout->addStretch(0);
	load_layout->addWidget(loadButton_.get());
	load_layout->addStretch(0);
	control_layout->addLayout(load_layout);

	QVBoxLayout * display_layout = new QVBoxLayout();
	QHBoxLayout * undo_layout = new QHBoxLayout();
	undo_layout->addWidget(detectPlanesButton_.get());
	undo_layout->addStretch(0.5);
	undo_layout->addWidget(resetButton_.get());
	undo_layout->addWidget(undoButton_.get());
	display_layout->addWidget(displayWidget_.get());
	display_layout->addLayout(undo_layout);

	QHBoxLayout *central_layout = new QHBoxLayout();
	central_layout->setSpacing(20);
	central_layout->addLayout(control_layout);
	central_layout->addLayout(display_layout);

	setLayout(central_layout);

	connect(loadButton_.get(), SIGNAL(clicked()), this, SLOT(load_images()));
	connect(resetButton_.get(), SIGNAL(clicked()), displayWidget_.get(), SLOT(reset()));
	connect(undoButton_.get(), SIGNAL(clicked()), displayWidget_.get(), SLOT(undo()));
	connect(imagesListWidget_.get(), SIGNAL(itemClicked(QListWidgetItem *)), this, SLOT(set_mark_image(QListWidgetItem *)));
	connect(this, SIGNAL(set_image(QString)), displayWidget_.get(), SLOT(set_image(QString)));
	connect(this, SIGNAL(change_label_state()), displayWidget_.get(), SLOT(change_state()));
	connect(detectPlanesButton_.get(), SIGNAL(clicked()), this, SLOT(detect_planes()));
}

MarkWidget::~MarkWidget()
{

}

void MarkWidget::reset_display()
{
}

void MarkWidget::load_images()
{
	images_path_list_ = QFileDialog::getOpenFileNames(this, "Input Images",
		"D:\\Pictures\\2Dto3D", tr("Images (*.png *.jpg)"));
	//QString images_dir = QFileDialog::getExistingDirectory(this, tr("Open Directory"), "D:\\Pictures\\2Dto3D", QFileDialog::ShowDirsOnly | QFileDialog::DontResolveSymlinks);
	//QDir imageDir(images_dir);
	//imageDir.setNameFilters(QStringList() << "*.jpg");
	//QStringList temp = imageDir.entryList(QDir::NoDotAndDotDot | QDir::System | QDir::Hidden | QDir::Files, QDir::DirsFirst);
	//QStringList images_path_list;
	//for (QStringList::iterator it = temp.begin(); it != temp.end(); it++)
	//{
	//	QString name = *it;
	//	QString file = imageDir.absoluteFilePath(name);
	//	//file += ".jpg";
	//	images_path_list.push_back(file);
	//} 

	//std::ofstream out("D:\\ProgrammingTools\\Matlab\\workspace\\Reconstruction\\image_dir.txt");
	//images_dir.replace("/", "\\");
	//out << images_dir.toStdString() << std::endl;
	//out.close();

	if (!images_path_list_.empty())
		set_image_thumbnails();
}

void MarkWidget::set_images_path(QStringList list)
{
	images_path_list_.clear();
	for (QStringList::iterator it = list.begin(); it != list.end(); it++)
		images_path_list_.push_back(*it);
}

void MarkWidget::set_image_thumbnails()
{
	imagesListWidget_->clear();
	imagesListWidget_->setViewMode(QListWidget::IconMode);
	imagesListWidget_->setIconSize(QSize(230, 172.5));
	imagesListWidget_->setResizeMode(QListWidget::Adjust);
	int img_idx = 1;
	for (QStringList::iterator img_it = images_path_list_.begin(); img_it != images_path_list_.end(); ++img_it, img_idx++)
	{
		QString img_title = "Image " + QString::number(img_idx);
		imagesListWidget_->addItem(new QListWidgetItem(QIcon(img_it->toLocal8Bit().data()), img_title));
	}
}

void MarkWidget::set_mark_image(QListWidgetItem * item)
{
	QString title = item->text();
	int id = title.section(' ', 1, 1).toInt() - 1;
	QString path = images_path_list_[id];
	emit(set_image(path));
}

void MarkWidget::detect_planes()
{
	std::shared_ptr<FacesIdentifier> faces_detector(new FacesIdentifier(displayWidget_->get_vertices(), displayWidget_->get_edges()));
	std::vector<std::vector<int>>face_circuits;
	faces_detector->identify_all_faces(face_circuits);
	displayWidget_->set_faces(face_circuits);

	ConstraintsGenerator cg(800, 1024, 768, displayWidget_->get_vertices(), displayWidget_->get_edges(), face_circuits, displayWidget_->get_parallel_groups());
	cg.add_perspective_symmetry_constraint();
}

void MarkWidget::keyPressEvent(QKeyEvent * event)
{
	if (event->key() == Qt::Key_Return)
	{
		emit change_label_state();
	}
}
