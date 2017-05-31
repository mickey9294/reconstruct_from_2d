#include "MarkWidget.h"

MarkWidget::MarkWidget(QWidget *parent)
	: QWidget(parent)
{
	undoButton_.reset(new QPushButton("Undo"));
	loadButton_.reset(new QPushButton("Load Images"));
	resetButton_.reset(new QPushButton("Reset"));
	reconButton_.reset(new QPushButton("Reconstruct From File"));
	recognitionButton_.reset(new QPushButton("Recognize Precise Vertices"));
	detectPlanesButton_.reset(new QPushButton("Detect Planes"));
	connectButton_.reset(new QPushButton("Auto Connect"));
	addConstraintsButton_.reset(new QPushButton("Add Constraints"));
	state_label_.reset(new QLabel("Marking State"));
	displayWidget_.reset(new MarkGraphicsScene());
	imagesListWidget_.reset(new QListWidget());
	imagesListWidget_->setFixedWidth(250);
	//imagesListWidget_->setFixedHeight(550);
	imagesListWidget_->setMinimumHeight(550);
	displayWidget_->resize(800, 600);


	image_list_label.reset(new QLabel("Images List"));
	//image_list_label->setMaximumHeight(30);
	control_layout.reset(new QVBoxLayout());
	control_layout->addWidget(image_list_label.get());
	control_layout->addWidget(imagesListWidget_.get());
	control_layout->setAlignment(Qt::AlignTop);
	load_layout.reset(new QHBoxLayout());
	load_layout->addStretch(0);
	load_layout->addWidget(loadButton_.get());
	load_layout->addStretch(0);
	control_layout->addLayout(load_layout.get());

	display_layout.reset(new QVBoxLayout());
	undo_layout.reset(new QHBoxLayout());
	undo_layout->addWidget(recognitionButton_.get());
	undo_layout->addWidget(connectButton_.get());
	undo_layout->addWidget(detectPlanesButton_.get());
	undo_layout->addWidget(addConstraintsButton_.get());
	undo_layout->addWidget(reconButton_.get());
	undo_layout->addStretch(0.5);
	undo_layout->addWidget(resetButton_.get());
	undo_layout->addWidget(undoButton_.get());
	display_layout->addWidget(state_label_.get());
	display_layout->addWidget(displayWidget_.get());
	display_layout->addLayout(undo_layout.get());

	central_layout.reset(new QHBoxLayout());
	central_layout->setSpacing(20);
	central_layout->addLayout(control_layout.get());
	central_layout->addLayout(display_layout.get());

	setLayout(central_layout.get());

	connect(loadButton_.get(), SIGNAL(clicked()), this, SLOT(load_images()));
	connect(resetButton_.get(), SIGNAL(clicked()), displayWidget_.get(), SLOT(reset()));
	connect(undoButton_.get(), SIGNAL(clicked()), displayWidget_.get(), SLOT(undo()));
	connect(addConstraintsButton_.get(), SIGNAL(clicked()), this, SLOT(generate_constraints()));
	connect(recognitionButton_.get(), SIGNAL(clicked()), this, SLOT(recognize_precise_vertices()));
	connect(imagesListWidget_.get(), SIGNAL(itemClicked(QListWidgetItem *)), this, SLOT(set_mark_image(QListWidgetItem *)));
	connect(reconButton_.get(), SIGNAL(clicked()), this, SLOT(recon_from_file()));
	connect(this, SIGNAL(set_image(QString)), displayWidget_.get(), SLOT(set_image(QString)));
	connect(this, SIGNAL(change_label_state()), displayWidget_.get(), SLOT(change_state()));
	connect(this, SIGNAL(stop_labeling()), displayWidget_.get(), SLOT(finish_label_parallelism()));
	connect(detectPlanesButton_.get(), SIGNAL(clicked()), this, SLOT(detect_planes()));
	connect(displayWidget_.get(), SIGNAL(set_state_text(QString)), state_label_.get(), SLOT(setText(QString)));
	connect(connectButton_.get(), SIGNAL(clicked()), displayWidget_.get(), SLOT(auto_connect()));
}

MarkWidget::~MarkWidget()
{

}

void MarkWidget::reset_display()
{
	if (constraints_generator_)
		constraints_generator_->reset();
	displayWidget_->reset();
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
	{
		set_image_thumbnails();

		displayWidget_->set_image(images_path_list_.front());
	}
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

void MarkWidget::update_scene()
{
	QString update_file_path = QFileDialog::getOpenFileName(this, "Input Images",
		"..\\matlab", tr("Text File (*.txt *.csv)"));

	std::vector<Eigen::Vector2d> refined_vertices(displayWidget_->num_vertices());
	std::ifstream in(update_file_path.toLocal8Bit().data());
	if (in.is_open())
	{
		std::string line;
		std::vector<std::string> line_split;
		for (int i = 0; i < displayWidget_->num_vertices(); i++)
		{
			std::getline(in, line);
			boost::split(line_split, line, boost::is_any_of(","), boost::token_compress_on);
			refined_vertices[i][0] = std::stof(line_split[0]);
			refined_vertices[i][1] = std::stof(line_split[1]);
		}

		in.close();
	}

	displayWidget_->update_scene(refined_vertices);
}

void MarkWidget::recon_from_file()
{
	QString q_path = QFileDialog::getOpenFileName(this, "Shape Vector",
		"..\\matlab", tr("CSV File (*.csv)"));

	if (!constraints_generator_)
	{
		constraints_generator_.reset(new ConstraintsGenerator(displayWidget_->get_image_path(), displayWidget_->width(), displayWidget_->height(),
			displayWidget_->get_vertices(), displayWidget_->get_edges(), displayWidget_->const_face_circuits(),
			displayWidget_->get_parallel_groups(), displayWidget_->get_precises_vertices(), this));
		connect(constraints_generator_.get(), SIGNAL(report_status(QString)), this, SLOT(receive_status(QString)));
	}
	else
		constraints_generator_->reset(displayWidget_->get_image_path(), displayWidget_->width(), displayWidget_->height(),
			displayWidget_->get_vertices(), displayWidget_->get_edges(), displayWidget_->const_face_circuits(), displayWidget_->get_parallel_groups());
	constraints_generator_->reconstruct_from_file(q_path);
}

void MarkWidget::recognize_precise_vertices()
{
	VertexRecognition recognition;
	recognition.set_image(displayWidget_->get_image());
	recognition.set_sketch_vertices(displayWidget_->get_vertices());

	std::vector<int> precise_vertices_id;
	std::vector<QPointF> precise_vertices;
	std::vector<std::vector<QLineF>> line_segments;
	recognition.get_precise_vertices(precise_vertices_id, precise_vertices, line_segments);

	displayWidget_->set_line_segments(line_segments);
	displayWidget_->set_precise_vertices(precise_vertices_id, precise_vertices);
}

void MarkWidget::receive_status(QString text)
{
	emit report_status(text);
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
	displayWidget_->clear_line_segments();

	//if (displayWidget_->num_faces() == 0)
	//{
	std::shared_ptr<FacesIdentifier> faces_detector(new FacesIdentifier(displayWidget_->get_vertices(), displayWidget_->get_edges()));
	std::vector<std::vector<int>>face_circuits;
	faces_detector->identify_all_faces(face_circuits);
	displayWidget_->set_faces(face_circuits);
	/*}
	else
		displayWidget_->repaint_faces();*/

	emit stop_labeling();
}

void MarkWidget::generate_constraints()
{
	displayWidget_->clear_line_segments();
	displayWidget_->save_current_state();

	if (!constraints_generator_)
	{
		constraints_generator_.reset(new ConstraintsGenerator(displayWidget_->get_image_path(), displayWidget_->width(), displayWidget_->height(),
			displayWidget_->get_vertices(), displayWidget_->get_edges(), displayWidget_->const_face_circuits(), 
			displayWidget_->get_parallel_groups(), displayWidget_->get_precises_vertices(), this));
		connect(constraints_generator_.get(), SIGNAL(report_status(QString)), this, SLOT(receive_status(QString)));
	}
	else
		constraints_generator_->reset(displayWidget_->get_image_path(), displayWidget_->width(), displayWidget_->height(),
			displayWidget_->get_vertices(), displayWidget_->get_edges(), displayWidget_->const_face_circuits(), displayWidget_->get_parallel_groups());
	
	std::vector<Eigen::Vector2d> refined_vertices;
	Eigen::VectorXd refined_q;
	MeshModel mesh;
	constraints_generator_->add_constraints(refined_vertices, refined_q, mesh);
	//mesh.output("..\\mesh.ply");

	if(!refined_vertices.empty())
		displayWidget_->update_scene(refined_vertices);

	emit reconstruct_done(mesh, refined_vertices, displayWidget_->get_image());
}

void MarkWidget::save_scene_state()
{
	displayWidget_->save_current_state();
}

void MarkWidget::load_scene_state()
{
	displayWidget_->load_current_state();
}

void MarkWidget::keyPressEvent(QKeyEvent * event)
{
	if (event->key() == Qt::Key_Return)
	{
		emit change_label_state();
	}
}
