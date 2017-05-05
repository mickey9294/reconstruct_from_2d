#include "MarkGraphicsScene.h"

MarkGraphicsScene::MarkGraphicsScene(QWidget *parent)
	: QGraphicsView(parent)
{
	srand(time(NULL));

	this->setRenderHints(QPainter::Antialiasing);
	this->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
	this->setVerticalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
	
	graphics_scene_.reset(new QGraphicsScene(0,0,IMAGE_WIDTH,IMAGE_HEIGHT));

	line_pen_.setWidth(3);
	line_pen_.setColor(QColor(0, 0, 255));

	vertex_pen_.setWidth(1);
	vertex_pen_.setColor(QColor(255, 0, 0));
	vertex_brush_.setColor(QColor(255, 0, 0));
	vertex_brush_.setStyle(Qt::SolidPattern);

	focus_pen_.setWidth(4);
	focus_pen_.setColor(QColor(0, 255, 255));

	parallel_pen_ = random_pen();

	vertex_.reset(new QPointF());

	//QPolygonF poly;
	//poly << QPointF(10, 10) << QPointF(10, 50) << QPointF(30, 70 )<< QPointF(60, 50) << QPointF(50, 10);
	//QGraphicsPolygonItem * item = new QGraphicsPolygonItem(poly);
	//QBrush brush;
	//brush.setColor(Qt::red);
	//brush.setStyle(Qt::SolidPattern);
	//QPen pen(Qt::green);
	//QPainter painter;
	//painter.setBrush(brush);
	//painter.drawPolygon(poly);

	//graphics_scene_->addPolygon(poly, pen, brush);
	state_ = LabelSilhouette;
	focused_line_id_ = -1;

	this->setScene(graphics_scene_.get());
}

MarkGraphicsScene::~MarkGraphicsScene()
{
}

std::list<QPointF>& MarkGraphicsScene::get_vertices()
{
	return vertex_list_;
}

std::list<Line>& MarkGraphicsScene::get_edges()
{
	return line_list_;
}

std::list<std::vector<int>>& MarkGraphicsScene::get_parallel_groups()
{
	return parallel_lines_group_;
}

QPixmap & MarkGraphicsScene::get_image()
{
	return image_;
}

const std::string & MarkGraphicsScene::get_image_path() const
{
	return image_path_;
}

const std::vector<std::vector<int>>& MarkGraphicsScene::const_face_circuits() const
{
	return face_circuits_;
}

int MarkGraphicsScene::num_faces() const
{
	return face_circuits_.size();
}

int MarkGraphicsScene::num_vertices() const
{
	return vertex_list_.size();
}

void MarkGraphicsScene::update_scene(const std::vector<Eigen::Vector2d>& refined_vertices)
{
	std::vector<Eigen::Vector2d>::const_iterator r_it = refined_vertices.begin();
	std::list<QPointF>::iterator vert_it = vertex_list_.begin();
	std::list<QGraphicsEllipseItem *>::iterator vi_it = vertex_item_stack_.begin();
	std::list<QGraphicsTextItem *>::iterator ni_it = number_item_stack_.begin();

	/* Clear faces' display */
	for (std::list<QGraphicsPolygonItem *>::iterator f_it = face_item_stack_.begin();
		f_it != face_item_stack_.end(); ++f_it)
		graphics_scene_->removeItem(*f_it);
	face_item_stack_.clear();

	float half_w = (float)image_.width() / 2.0;
	float half_h = (float)image_.height() / 2.0;

	/* Update vertices */
	for (; r_it != refined_vertices.end() && vert_it != vertex_list_.end()
		&& vi_it != vertex_item_stack_.end() && ni_it != number_item_stack_.end();
		++r_it, ++vert_it, ++vi_it, ++ni_it)
	{
		(*vi_it)->setRect(half_w + r_it->x() - 3, half_h - r_it->y() - 3, 6, 6);
		(*vi_it)->setZValue(1.0);
		(*ni_it)->setPos(half_w + r_it->x(), half_h - r_it->y());

		vert_it->setX(half_w + r_it->x());
		vert_it->setY(half_h - r_it->y());
	}

	/* Update edges */
	std::list<Line>::iterator l_it = line_list_.begin();
	std::list<QGraphicsLineItem *>::iterator li_it = line_item_stack_.begin();
	for (; l_it != line_list_.end() && li_it != line_item_stack_.end(); ++l_it, ++li_it)
	{
		QPointF v1(half_w + refined_vertices[l_it->p1()].x(), half_h - refined_vertices[l_it->p1()].y());
		QPointF v2(half_w + refined_vertices[l_it->p2()].x(), half_h - refined_vertices[l_it->p2()].y());
		QLineF new_line(v1, v2);
		(*li_it)->setLine(new_line);
		(*li_it)->setPen(line_pen_);
	}
	update();
}

void MarkGraphicsScene::set_precise_vertices(const std::vector<int>& precise_id, const std::vector<QPointF>& precise_vertices)
{
	/* update vertices */
	QPen precise_pen;
	precise_pen.setColor(QColor(255, 153, 0));
	precise_pen.setWidth(1);
	QBrush precise_brush(QColor(255, 153, 0), Qt::SolidPattern);

	precise_verts_id_.resize(precise_id.size());
	for (int i = 0; i < precise_id.size(); i++)
	{
		int vert_id = precise_id[i];
		precise_verts_id_[i] = vert_id;
		const QPointF &precise_vert = precise_vertices[i];

		std::list<QPointF>::iterator vert_it = vertex_list_.begin();
		std::list<QGraphicsEllipseItem *>::iterator vert_item_it = vertex_item_stack_.begin();
		std::advance(vert_it, vert_id);
		std::advance(vert_item_it, vert_id);

		vert_it->setX(precise_vert.x());
		vert_it->setY(precise_vert.y());
		(*vert_item_it)->setRect(precise_vert.x() - 4, precise_vert.y() - 4, 8, 8);
		(*vert_item_it)->setPen(precise_pen);
		(*vert_item_it)->setBrush(precise_brush);
	}

	/* update edges */
	std::list<Line>::iterator l_it = line_list_.begin();
	std::list<QGraphicsLineItem *>::iterator li_it = line_item_stack_.begin();
	for (; l_it != line_list_.end() && li_it != line_item_stack_.end(); ++l_it, ++li_it)
	{
		std::list<QPointF>::iterator vert_it = std::next(vertex_list_.begin(), l_it->p1());
		QPointF &v1 = *vert_it;
		vert_it = std::next(vertex_list_.begin(), l_it->p2());
		QPointF &v2 = *vert_it;
		QLineF new_line(v1, v2);
		(*li_it)->setLine(new_line);
	}

	update();
}

void MarkGraphicsScene::set_line_segments(const std::vector<std::vector<QLineF>>& line_segments)
{
	QPen ls_pen(QColor(255, 255, 0));
	ls_pen.setWidth(2.0);

	for (std::vector<std::vector<QLineF>>::const_iterator patch_it = line_segments.begin();
		patch_it != line_segments.end(); patch_it++)
	{
		for (std::vector<QLineF>::const_iterator ls_it = patch_it->begin();
			ls_it != patch_it->end(); ++ls_it)
		{
			QGraphicsLineItem * ls_item = graphics_scene_->addLine(*ls_it, ls_pen);
			ls_item_stack_.push_back(ls_item);
		}
	}
	update();
}

std::vector<int>& MarkGraphicsScene::get_precises_vertices()
{
	return precise_verts_id_;
}

void MarkGraphicsScene::save_current_state()
{
	std::ofstream out(".\\state.txt");
	if (out.is_open())
	{
		/* Save the number */
		out << vertex_list_.size() << " " << line_list_.size() << " "
			<< face_circuits_.size() << " " << parallel_lines_group_.size() << " "
			<< precise_verts_id_.size() << std::endl;

		/* Save vertices */
		for (std::list<QPointF>::iterator vert_it = vertex_list_.begin(); vert_it != vertex_list_.end(); ++vert_it)
			out << vert_it->x() << " " << vert_it->y() << std::endl;

		/* Save the edges */
		for (std::list<Line>::iterator edge_it = line_list_.begin(); edge_it != line_list_.end(); ++edge_it)
			out << edge_it->p1() << " " << edge_it->p2() << std::endl;

		/* Save the face circuits */
		for (std::vector<std::vector<int>>::iterator face_it = face_circuits_.begin(); face_it != face_circuits_.end(); ++face_it)
		{
			if (!face_it->empty())
			{
				out << face_it->front();
				for (int i = 1; i < face_it->size(); i++)
					out << " " << face_it->operator[](i);
				out << std::endl;
			}
		}

		/* Save the parallel groups */
		for (std::list<std::vector<int>>::iterator group_it = parallel_lines_group_.begin();
			group_it != parallel_lines_group_.end(); ++group_it)
		{
			if (!group_it->empty())
			{
				out << group_it->front();
				for (int i = 1; i < group_it->size(); i++)
					out << " " << group_it->operator[](i);
				out << std::endl;
			}
		}

		/* Save the precise vertices id */
		std::vector<int>::iterator p_it = precise_verts_id_.begin();
		if (p_it != precise_verts_id_.end())
		{
			out << *p_it;
			std::advance(p_it, 1);
			for (;p_it != precise_verts_id_.end(); ++p_it)
			{
				out << " " << *p_it;
			}
			out << std::endl;
		}

		out.close();
	}
}

void MarkGraphicsScene::load_current_state()
{
	std::ifstream in(".\\state.txt");
	if (in.is_open())
	{
		std::string line;
		std::getline(in, line);

		std::vector<std::string> line_split;
		boost::split(line_split, line, boost::is_any_of(" "), boost::token_compress_on);
		int num_vertices = std::stoi(line_split[0]);
		int num_edges = std::stoi(line_split[1]);
		int num_faces = std::stoi(line_split[2]);
		int num_parallel_groups = std::stoi(line_split[3]);
		int num_precise_vertices = std::stoi(line_split[4]);
		
		for (int i = 0; i < num_vertices; i++)
		{
			std::getline(in, line);
			boost::split(line_split, line, boost::is_any_of(" "), boost::token_compress_on);
			QPointF vert(std::stof(line_split[0]), std::stof(line_split[1]));
			vertex_list_.push_back(vert);
		}

		for (int i = 0; i < num_edges; i++)
		{
			std::getline(in, line);
			boost::split(line_split, line, boost::is_any_of(" "), boost::token_compress_on);
			Line edge(std::stoi(line_split[0]), std::stoi(line_split[1]));
			line_list_.push_back(edge);
		}

		std::vector<std::vector<int>> face_circuits(num_faces);
		for (int i = 0; i < num_faces; i++)
		{
			std::getline(in, line);
			boost::split(line_split, line, boost::is_any_of(" "), boost::token_compress_on);
			face_circuits[i].resize(line_split.size());
			for (int j = 0; j < line_split.size(); j++)
				face_circuits[i][j] = std::stoi(line_split[j]);
		}

		for (int i = 0; i < num_parallel_groups; i++)
		{
			std::getline(in, line);
			boost::split(line_split, line, boost::is_any_of(" "), boost::token_compress_on);
			std::vector<int> group(line_split.size());
			for (int j = 0; j < line_split.size(); j++)
				group[j] = std::stoi(line_split[j]);
			parallel_lines_group_.push_back(group);
		}

		std::unordered_set<int> precise_set;
		std::getline(in, line);
		boost::split(line_split, line, boost::is_any_of(" "), boost::token_compress_on);
		assert(line_split.size() == num_precise_vertices);
		precise_verts_id_.resize(num_precise_vertices);
		for (int i = 0; i < line_split.size(); i++)
		{
			int precise_id = std::stoi(line_split[i]);
			precise_verts_id_[i] = precise_id;
			precise_set.insert(precise_id);
		}

		/* Restore state of graphics scene */
		QFont font;
		font.setPointSize(12);
		font.setBold(true);
		QPen precise_pen;
		precise_pen.setColor(QColor(255, 153, 0));
		precise_pen.setWidth(1);
		QBrush precise_brush(QColor(255, 153, 0), Qt::SolidPattern);
		int idx = 0;
		for (std::list<QPointF>::iterator vert_it = vertex_list_.begin(); vert_it != vertex_list_.end(); ++vert_it, ++idx)
		{
			if(precise_set.find(idx) == precise_set.end())
				vertex_item_ = graphics_scene_->addEllipse(vert_it->x() - 3, vert_it->y() - 3, 6, 6, vertex_pen_, vertex_brush_);
			else
				vertex_item_ = graphics_scene_->addEllipse(vert_it->x() - 3, vert_it->y() - 3, 6, 6, precise_pen, precise_brush);
			vertex_item_->setZValue(1.0);
			vertex_item_stack_.push_back(vertex_item_);
			number_item_ = graphics_scene_->addText(QString::number(idx), font);
			number_item_->setZValue(1.0);
			number_item_->setPos(*vert_it);
			number_item_stack_.push_back(number_item_);
			mode_stack_.push_back(0);
		}

		for (std::list<Line>::iterator edge_it = line_list_.begin(); edge_it != line_list_.end(); ++edge_it)
		{
			std::list<QPointF>::iterator it = vertex_list_.begin(); 
			std::advance(it, edge_it->p1());
			QPointF v1 = *it;
			it = vertex_list_.begin();
			std::advance(it, edge_it->p2());
			QPointF v2 = *it;

			QLineF line(v1, v2);
			line_item_ = graphics_scene_->addLine(line, line_pen_);
			line_item_->setZValue(0.7);
			line_item_stack_.push_back(line_item_);
			mode_stack_.push_back(1);
		}
		perm_ = true;

		set_faces(face_circuits);

		for (std::list<std::vector<int>>::iterator group_it = parallel_lines_group_.begin();
			group_it != parallel_lines_group_.end(); ++group_it)
		{
			parallel_pen_ = random_pen();
			for (std::vector<int>::iterator l_it = group_it->begin(); l_it != group_it->end(); ++l_it)
			{
				std::list<QGraphicsLineItem *>::iterator it = line_item_stack_.begin();
				std::advance(it, *l_it);
				(*it)->setPen(parallel_pen_);
			}
		}

		update();

		in.close();
	}
}

void MarkGraphicsScene::set_image(QString image_path)
{
	emit set_state_text("Marking vertices and edges");

	QImage test_img(image_path);
	image_ = QPixmap::fromImage(test_img);

	if (image_.width() < 10)
	{
		std::cerr << "Error: Cannot load image " << image_path.toStdString() << "!" << std::endl;
	}
	else
	{
		image_path_ = image_path.toStdString();

		if (image_.width() > IMAGE_WIDTH)
			image_ = image_.scaledToWidth(IMAGE_WIDTH);
		int img_w = image_.width();
		int img_h = image_.height();

		//int scroll_bar_width = qApp->style()->pixelMetric(QStyle::PM_ScrollBarExtent);
		this->setFixedSize(img_w, img_h);
		//setContentsMargins(0, 0, 0, 0);

		//QRectF new_scene_rect(0, 0, img_w, img_h);
		//graphics_scene_->setSceneRect(new_scene_rect);

		pixmap_item_.reset(new QGraphicsPixmapItem());
		pixmap_item_->setPixmap(image_);
		
		graphics_scene_->addItem(pixmap_item_.get());

		reset();
	}
}

void MarkGraphicsScene::set_faces(const std::vector<std::vector<int>>& face_circuits)
{
	face_circuits_.resize(face_circuits.size());

	int face_idx = 0;
	for (std::vector<std::vector<int>>::const_iterator face_it = face_circuits.begin();
		face_it != face_circuits.end(); ++face_it, ++face_idx)
	{
		face_circuits_[face_idx].clear();
		face_circuits_[face_idx].reserve(face_it->size());

		QVector<QPointF> face_vertices;
		face_vertices.reserve(face_it->size());
		std::vector<QPointF> verts_vec(vertex_list_.begin(), vertex_list_.end());
		for (std::vector<int>::const_iterator vert_it = face_it->begin(); vert_it != face_it->end(); vert_it++)
		{
			face_circuits_[face_idx].push_back(*vert_it);
			face_vertices.push_back(verts_vec[*vert_it]);
		}
		//face_vertices.push_back(verts_vec.front());

		QPolygonF face(face_vertices);
		int r = rand() % 255;
		int g = rand() % 255;
		int b = rand() % 255;
		QColor face_color(r, g, b, 158);
		QBrush face_brush;
		face_brush.setColor(face_color);
		face_brush.setStyle(Qt::SolidPattern);
		QPen face_pen = Qt::NoPen;
		
		QGraphicsPolygonItem * face_item = graphics_scene_->addPolygon(face, face_pen, face_brush);
		face_item->setZValue(0.5);
		face_item_stack_.push_back(face_item);
	}

	update();
}

void MarkGraphicsScene::change_state()
{
	if (state_ == LabelSilhouette)
	{
		std::cout << "Change state. Now label parallel lines." << std::endl;
		state_ = LabelParallel;

		/* Compute U_ and A_ */
		N_.resize(2, line_list_.size());
		A_.resize(2, line_list_.size());
		B_.resize(2, line_list_.size());
		int idx = 0;
		std::vector<QPointF> vertices_vec(vertex_list_.begin(), vertex_list_.end());
		for (std::list<Line>::iterator it = line_list_.begin(); it != line_list_.end(); ++it, ++idx)
		{
			QPointF &v1 = vertices_vec[it->p1()];
			QPointF &v2 = vertices_vec[it->p2()];

			Eigen::Vector2d u;
			u[0] = v2.x() - v1.x();
			u[1] = v2.y() - v1.y();
			u.normalize();

			N_.block<2, 1>(0, idx) = u;
			A_.block<2, 1>(0, idx) = Eigen::Vector2d(v1.x(), v1.y());
			B_.block<2, 1>(0, idx) = Eigen::Vector2d(v2.x(), v2.y());
		}
		parallel_pen_ = random_pen();

		line_chosen_before_.resize(line_list_.size(), false);

		emit set_state_text("Marking parallel lines");
	}
	else if (state_ == LabelParallel)
	{
		std::cout << "Change state. Now label vertices or edges." << std::endl;
		state_ = LabelSilhouette;
		N_.resize(0, 0);
		A_.resize(0, 0);
		B_.resize(0, 0);

		emit set_state_text("Marking vertices and edges");
	}
}

void MarkGraphicsScene::finish_label_parallelism()
{
	if (!current_parallel_group_.empty())
	{
		std::vector<int> new_parallel_group(current_parallel_group_.size());
		int idx = 0;
		for (std::unordered_set<int>::iterator it = current_parallel_group_.begin();
			it != current_parallel_group_.end(); ++it)
		{
			new_parallel_group[idx++] = *it;
		}
		parallel_lines_group_.push_back(new_parallel_group);

		current_parallel_group_.clear();
		parallel_pen_ = random_pen();
	}
}

void MarkGraphicsScene::reset()
{
	while (!line_item_stack_.empty())
	{
		QGraphicsLineItem *item = line_item_stack_.back();
		line_item_stack_.pop_back();
		graphics_scene_->removeItem(item);
	}
	while (!vertex_item_stack_.empty())
	{
		QGraphicsEllipseItem *item = vertex_item_stack_.back();
		vertex_item_stack_.pop_back();
		graphics_scene_->removeItem(item);
	}
	while (!face_item_stack_.empty())
	{
		QGraphicsPolygonItem *item = face_item_stack_.back();
		face_item_stack_.pop_back();
		graphics_scene_->removeItem(item);
	}
	while (!number_item_stack_.empty())
	{
		QGraphicsTextItem *item = number_item_stack_.back();
		number_item_stack_.pop_back();
		graphics_scene_->removeItem(item);
	}
	line_list_.clear();
	vertex_list_.clear();

	//line_item_stack_.clear();
	//vertex_item_stack_.clear();	
	mode_stack_.clear();
	perm_ = false;

	parallel_lines_group_.clear();
	std::fill(line_chosen_before_.begin(), line_chosen_before_.end(), false);
	current_parallel_group_.clear();

	clear_line_segments();
	
	if (state_ == LabelParallel)
		change_state();
}

void MarkGraphicsScene::mousePressEvent(QMouseEvent *event)
{
	if (state_ == LabelSilhouette)
	{
		if (event->button() == Qt::RightButton)
		{
			mode_stack_.push_back(1);
			perm_ = false;
			line_.reset(new QLineF(event->pos(), event->pos()));
			line_item_ = graphics_scene_->addLine(*line_, line_pen_);
			line_item_->setZValue(0.7);
		}
		else if (event->button() == Qt::LeftButton)
		{
			mode_stack_.push_back(0);
			vertex_->setX(event->pos().x());
			vertex_->setY(event->pos().y());
			vertex_item_ = graphics_scene_->addEllipse(vertex_->x() - 3, vertex_->y() - 3, 6, 6, vertex_pen_, vertex_brush_);
			vertex_item_->setZValue(1.0);

			QFont font;
			font.setPointSize(12);
			font.setBold(true);
			number_item_ = graphics_scene_->addText(QString::number(vertex_list_.size()), font);
			number_item_->setZValue(1.0);
			number_item_->setPos(event->pos());
		}
	}
	else if (state_ == LabelParallel)
	{
		if (event->button() == Qt::LeftButton)
		{
			if (focused_line_id_ >= 0)
			{
				if (!line_chosen_before_[focused_line_id_])
				{
					line_chosen_before_[focused_line_id_] = true;
					line_item_->setPen(parallel_pen_);
					current_parallel_group_.insert(focused_line_id_);
				}
				else
				{
					line_chosen_before_[focused_line_id_] = false;
					line_item_->setPen(line_pen_);
					current_parallel_group_.erase(focused_line_id_);
				}
				update();
			}
		}
		else if (event->button() == Qt::MiddleButton)
		{
			if (!current_parallel_group_.empty())
			{
				std::vector<int> new_parallel_group(current_parallel_group_.size());
				int idx = 0;
				for (std::unordered_set<int>::iterator it = current_parallel_group_.begin();
					it != current_parallel_group_.end(); ++it)
				{
					new_parallel_group[idx++] = *it;
				}
				parallel_lines_group_.push_back(new_parallel_group);

				current_parallel_group_.clear();
				parallel_pen_ = random_pen();
			}
		}
	}
};

void MarkGraphicsScene::mouseMoveEvent(QMouseEvent * event)
{
	if (state_ == LabelSilhouette)
	{
		if (line_ && !perm_)
		{
			line_->setP2(event->pos());
			line_item_->setLine(*line_);
			update();
		}
	}
	else if (state_ == LabelParallel)
	{
		int nearest_edge_id = -1;
		float min_dist = std::numeric_limits<float>::max();
		bool ret = find_nearest_edge(event->pos(), nearest_edge_id, min_dist);
		if (ret)
		{
			if (focused_line_id_ != nearest_edge_id)
			{
				if(focused_line_id_ >= 0 && !line_chosen_before_[focused_line_id_])
					line_item_->setPen(line_pen_);
				focused_line_id_ = nearest_edge_id;
			}
			std::list<QGraphicsLineItem *>::iterator edge_it = line_item_stack_.begin();
			std::advance(edge_it, nearest_edge_id);
			line_item_ = *edge_it;

			if (!line_chosen_before_[focused_line_id_])
			{
				line_item_->setPen(focus_pen_);
				update();
			}
		}
		else
		{
			if (focused_line_id_ >= 0 && !line_chosen_before_[focused_line_id_])
			{
				line_chosen_before_[focused_line_id_] = false;
				focused_line_id_ = -1;
				line_item_->setPen(line_pen_);
				update();
			}
		}
	}
}

void MarkGraphicsScene::mouseReleaseEvent(QMouseEvent *event) {
	if (state_ == LabelSilhouette)
	{
		if (event->button() == Qt::RightButton)
		{
			QPointF real_p1, real_p2;
			int real_p1_idx = find_nearest_vertex(line_->p1(), real_p1);
			int real_p2_idx = find_nearest_vertex(line_->p2(), real_p2);
			if (real_p1_idx != real_p2_idx)
			{
				line_->setP1(real_p1);
				line_->setP2(real_p2);
				line_item_->setLine(*line_);
				update();

				perm_ = true;
				//QLineF new_line(*line_);
				line_list_.push_back(Line(real_p1_idx, real_p2_idx));
				/*if (line_item_stack_.size() >= STACK_SIZE)
					line_item_stack_.pop_front();*/
				line_item_stack_.push_back(line_item_);
			}
			else
			{
				graphics_scene_->removeItem(line_item_);
				mode_stack_.pop_back();
			}
		}
		else if (event->button() == Qt::LeftButton)
		{
			QPointF new_vertex(*vertex_);
			vertex_list_.push_back(new_vertex);
			/*if (vertex_item_stack_.size() >= STACK_SIZE)
				vertex_item_stack_.pop_front();*/
			vertex_item_stack_.push_back(vertex_item_);
			number_item_stack_.push_back(number_item_);
		}
	}
}

void MarkGraphicsScene::keyPressEvent(QKeyEvent * event)
{
	std::cout << event->key() << std::endl;
	if (event->key() == Qt::Key_Return)
	{
		change_state();
	}
}

int MarkGraphicsScene::find_nearest_vertex(const QPointF & point, QPointF &nearest_vertex)
{
	float min_dist = std::numeric_limits<float>::max();
	//QPointF nearest_vertex;
	int nearest_index = 0;
	int vert_idx = 0;
	for (std::list<QPointF>::iterator vert_it = vertex_list_.begin(); vert_it != vertex_list_.end(); ++vert_it, ++vert_idx)
	{
		float dist = std::pow(point.x() - vert_it->x(), 2) + std::pow(point.y() - vert_it->y(), 2);
		if (dist < min_dist)
		{
			min_dist = dist;
			nearest_vertex = *vert_it;
			nearest_index = vert_idx;
		}
	}
	return nearest_index;
}

bool MarkGraphicsScene::find_nearest_edge(const QPoint & point, int & nearest_edge_id, float & distance)
{
	if (line_list_.empty())
		return false;

	Eigen::Vector2d p((float)point.x(), (float)point.y());
	//Eigen::MatrixXd AP = A_.colwise() - p;

	//Eigen::MatrixXd d_vec = N_.array().rowwise() * ((AP.transpose() * N_).diagonal().transpose().array());
	//Eigen::VectorXd result = (AP - d_vec).colwise().norm();

	//distance = result.minCoeff(&nearest_edge_id);

	float dist_threshold = 0.05 * this->height();
	
	distance = std::numeric_limits<float>::max();
	for (int i = 0; i < line_list_.size(); i++)
	{
		float min_dist = std::numeric_limits<float>::max();

		Eigen::Vector2d v1 = A_.col(i);
		Eigen::Vector2d v2 = B_.col(i);

		float l2 = (v2 - v1).transpose() * (v2 - v1);
		if (std::abs(l2) < 1e-4)
		{
			min_dist = (p - v1).norm();
		}
		else
		{
			float t = std::max(0.0, std::min(1.0, (p - v1).dot(v2 - v1) / l2));
			Eigen::Vector2d projection = v1 + t * (v2 - v1);
			min_dist = (p - projection).norm();
		}

		if (min_dist < distance)
		{
			distance = min_dist;
			nearest_edge_id = i;
		}
	}

	if (distance > dist_threshold)
		return false;

	return true;
}

QColor MarkGraphicsScene::random_color()
{
	int r = rand() % 255;
	int g = rand() % 255;
	int b = rand() % 255;
	QColor color(r, g, b);

	return color;
}

QPen MarkGraphicsScene::random_pen()
{
	QPen pen;
	pen.setWidth(5);
	pen.setColor(random_color());
	
	return pen;
}

void MarkGraphicsScene::clear_line_segments()
{
	for (std::list<QGraphicsLineItem *>::iterator ls_it = ls_item_stack_.begin();
		ls_it != ls_item_stack_.end(); ++ls_it)
	{
		graphics_scene_->removeItem(*ls_it);
	}
}

void MarkGraphicsScene::repaint_faces()
{
	int face_idx = 0;
	for (std::vector<std::vector<int>>::const_iterator face_it = face_circuits_.begin();
		face_it != face_circuits_.end(); ++face_it, ++face_idx)
	{
		QVector<QPointF> face_vertices;
		face_vertices.reserve(face_it->size());
		std::vector<QPointF> verts_vec(vertex_list_.begin(), vertex_list_.end());
		for (std::vector<int>::const_iterator vert_it = face_it->begin(); vert_it != face_it->end(); vert_it++)
		{
			face_vertices.push_back(verts_vec[*vert_it]);
		}

		QPolygonF face(face_vertices);
		int r = rand() % 255;
		int g = rand() % 255;
		int b = rand() % 255;
		QColor face_color(r, g, b, 158);
		QBrush face_brush;
		face_brush.setColor(face_color);
		face_brush.setStyle(Qt::SolidPattern);
		QPen face_pen = Qt::NoPen;

		QGraphicsPolygonItem * face_item = graphics_scene_->addPolygon(face, face_pen, face_brush);
		face_item_stack_.push_back(face_item);
	}

	update();
}

void MarkGraphicsScene::undo()
{
	if (state_ == LabelSilhouette)
	{
		if (mode_stack_.empty())
			return;
		int last_mode = mode_stack_.back();
		mode_stack_.pop_back();
		if (last_mode == 1)
		{
			if (!line_item_stack_.empty())
			{
				QGraphicsLineItem *last_line_item = line_item_stack_.back();
				line_item_stack_.pop_back();
				graphics_scene_->removeItem(last_line_item);
				line_list_.pop_back();
			}
		}
		else if (last_mode == 0)
		{
			if (!vertex_item_stack_.empty())
			{
				QGraphicsEllipseItem * last_vertex_item = vertex_item_stack_.back();
				vertex_item_stack_.pop_back();
				graphics_scene_->removeItem(last_vertex_item);
				vertex_list_.pop_back();
			}
			if (!number_item_stack_.empty())
			{
				QGraphicsTextItem * last_number_item = number_item_stack_.back();
				number_item_stack_.pop_back();
				graphics_scene_->removeItem(last_number_item);
			}
		}

		update();
	}
}