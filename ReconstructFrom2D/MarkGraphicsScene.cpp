#include "MarkGraphicsScene.h"

MarkGraphicsScene::MarkGraphicsScene(QWidget *parent)
	: QGraphicsView(parent)
{
	srand(time(NULL));

	this->setRenderHints(QPainter::Antialiasing);
	this->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
	this->setVerticalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
	
	graphics_scene_.reset(new QGraphicsScene(0,0,IMAGE_WIDTH,IMAGE_HEIGHT));

	line_pen_.setWidth(2);
	line_pen_.setColor(QColor(0, 0, 255));

	vertex_pen_.setWidth(3);
	vertex_pen_.setColor(QColor(255, 0, 0));

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

void MarkGraphicsScene::set_image(QString image_path)
{
	QImage test_img(image_path);
	image_ = QPixmap::fromImage(test_img);

	if (image_.width() < 10)
	{
		std::cerr << "Error: Cannot load image " << image_path.toStdString() << "!" << std::endl;
	}
	else
	{
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
	for (std::vector<std::vector<int>>::const_iterator face_it = face_circuits.begin();
		face_it != face_circuits.end(); ++face_it)
	{
		QVector<QPointF> face_vertices;
		face_vertices.reserve(face_it->size());
		std::vector<QPointF> verts_vec(vertex_list_.begin(), vertex_list_.end());
		for (std::vector<int>::const_iterator vert_it = face_it->begin(); vert_it != face_it->end(); vert_it++)
		{
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

			Eigen::Vector2f u;
			u[0] = v2.x() - v1.x();
			u[1] = v2.y() - v1.y();
			u.normalize();

			N_.block<2, 1>(0, idx) = u;
			A_.block<2, 1>(0, idx) = Eigen::Vector2f(v1.x(), v1.y());
			B_.block<2, 1>(0, idx) = Eigen::Vector2f(v2.x(), v2.y());
		}
		parallel_pen_ = random_pen();

		line_chosen_before_.resize(line_list_.size(), false);
	}
	else if (state_ == LabelParallel)
	{
		std::cout << "Change state. Now label vertices or edges." << std::endl;
		state_ = LabelSilhouette;
		N_.resize(0, 0);
		A_.resize(0, 0);
		B_.resize(0, 0);
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
		}
		else if (event->button() == Qt::LeftButton)
		{
			mode_stack_.push_back(0);
			vertex_->setX(event->pos().x());
			vertex_->setY(event->pos().y());
			vertex_item_ = graphics_scene_->addEllipse(vertex_->x() - 2, vertex_->y() - 2, 4, 4, vertex_pen_);

			QFont font;
			font.setPointSize(12);
			font.setBold(true);
			number_item_ = graphics_scene_->addText(QString::number(vertex_list_.size()), font);
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

	Eigen::Vector2f p((float)point.x(), (float)point.y());
	Eigen::MatrixXf AP = A_.colwise() - p;

	Eigen::MatrixXf d_vec = N_.array().rowwise() * ((AP.transpose() * N_).diagonal().transpose().array());
	Eigen::VectorXf result = (AP - d_vec).colwise().norm();

	distance = result.minCoeff(&nearest_edge_id);

	float dist_threshold = 0.05 * this->height();
	if (distance > dist_threshold)
		return false;
	//Eigen::Vector2f P1 = A_.col(nearest_edge_id);
	//Eigen::Vector2f P2 = B_.col(nearest_edge_id);
	//float end_dist = std::min((p - P1).norm(), (p - P2).norm());
	//if (end_dist > dist_threshold)
	//	return false;
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
	pen.setWidth(3);
	pen.setColor(random_color());
	
	return pen;
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