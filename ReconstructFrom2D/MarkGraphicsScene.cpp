#include "MarkGraphicsScene.h"

MarkGraphicsScene::MarkGraphicsScene(QWidget *parent)
	: QGraphicsView(parent)
{
	this->setRenderHints(QPainter::Antialiasing);
	this->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
	this->setVerticalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
	
	graphics_scene_.reset(new QGraphicsScene(0,0,IMAGE_WIDTH,IMAGE_HEIGHT));

	line_pen_.setWidth(2);
	line_pen_.setColor(QColor(0, 0, 255));

	vertex_pen_.setWidth(3);
	vertex_pen_.setColor(QColor(255, 0, 0));

	vertex_.reset(new QPointF());

	this->setScene(graphics_scene_.get());
}

MarkGraphicsScene::~MarkGraphicsScene()
{
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
	line_list_.clear();
	vertex_list_.clear();
	//line_item_stack_.clear();
	//vertex_item_stack_.clear();	
	mode_stack_.clear();
	perm_ = false;
}

void MarkGraphicsScene::mousePressEvent(QMouseEvent *event)
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
		vertex_item_ = graphics_scene_->addEllipse(vertex_->x() - 1.5, vertex_->y() - 1.5, 3, 3, vertex_pen_);
	}
};

void MarkGraphicsScene::mouseMoveEvent(QMouseEvent * event)
{
	if (line_ && !perm_)
	{
		line_->setP2(event->pos());
		line_item_->setLine(*line_);
		update();
	}
}

void MarkGraphicsScene::mouseReleaseEvent(QMouseEvent *event) {
	if (event->button() == Qt::RightButton)
	{
		QPointF real_p1, real_p2;
		int real_p1_idx = find_nearest_vertex(line_->p1(), real_p1);
		int real_p2_idx = find_nearest_vertex(line_->p2(), real_p2);
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
	else if (event->button() == Qt::LeftButton)
	{
		QPointF new_vertex(*vertex_);
		vertex_list_.push_back(new_vertex);
		/*if (vertex_item_stack_.size() >= STACK_SIZE)
			vertex_item_stack_.pop_front();*/
		vertex_item_stack_.push_back(vertex_item_);
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

void MarkGraphicsScene::undo()
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
	}

	update();
}