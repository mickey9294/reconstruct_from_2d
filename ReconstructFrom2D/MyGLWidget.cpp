#include "MyGLWidget.h"

MyGLWidget::MyGLWidget(QWidget *parent)
	: QOpenGLWidget(parent)
{
	m = 1;
	m_model.reset(new PCModel());
	m_xRot = 0;
	m_yRot = 0;
	m_zRot = 0;
	translation_.setZero();

	m_transparent = QCoreApplication::arguments().contains(QStringLiteral("--transparent"));
	if (m_transparent)
		setAttribute(Qt::WA_TranslucentBackground);

	setFocusPolicy(Qt::StrongFocus);
}

MyGLWidget::~MyGLWidget()
{
}

void MyGLWidget::setModel(std::shared_ptr<ShapeModel> model)
{
	m_model = model;

	update();
}

void MyGLWidget::initializeGL()
{
	initializeOpenGLFunctions();
	glClearColor(255, 255, 255, m_transparent ? 0 : 1);

	glClearDepth(1.0f);
	glEnable(GL_DEPTH_TEST);
	glEnable(GL_BLEND);
	glEnable(GL_COLOR_MATERIAL);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA); //指定混合函数
	glDepthFunc(GL_LEQUAL);
	glHint(GL_PERSPECTIVE_CORRECTION_HINT, GL_NICEST);
	glShadeModel(GL_SMOOTH);
	init_light();
}

void MyGLWidget::init_light()
{
	GLfloat white_light[] = { 0.23, 0.23, 0.23, 1.0 };

	GLfloat light_position0[] = { 1.0, 1.0, 1.0, 0.0 };
	GLfloat light_position1[] = { 1.0, 1.0, -1.0, 0.0 };
	GLfloat light_position2[] = { 1.0, -1.0, 1.0, 0.0 };
	GLfloat light_position3[] = { 1.0, -1.0, -1.0, 0.0 };
	GLfloat light_position4[] = { -1.0, 1.0, 1.0, 0.0 };
	GLfloat light_position5[] = { -1.0, -1.0, 1.0, 0.0 };
	GLfloat light_position6[] = { -1.0, 1.0, -1.0, 0.0 };
	GLfloat light_position7[] = { -1.0, -1.0, -1.0, 0.0 };

	glLightfv(GL_LIGHT0, GL_POSITION, light_position0); glLightfv(GL_LIGHT0, GL_DIFFUSE, white_light); glLightfv(GL_LIGHT0, GL_SPECULAR, white_light);
	glLightfv(GL_LIGHT1, GL_POSITION, light_position1); glLightfv(GL_LIGHT1, GL_DIFFUSE, white_light); glLightfv(GL_LIGHT1, GL_SPECULAR, white_light);
	glLightfv(GL_LIGHT2, GL_POSITION, light_position2); glLightfv(GL_LIGHT2, GL_DIFFUSE, white_light); glLightfv(GL_LIGHT2, GL_SPECULAR, white_light);
	glLightfv(GL_LIGHT3, GL_POSITION, light_position3); glLightfv(GL_LIGHT3, GL_DIFFUSE, white_light); glLightfv(GL_LIGHT3, GL_SPECULAR, white_light);
	glLightfv(GL_LIGHT4, GL_POSITION, light_position4); glLightfv(GL_LIGHT4, GL_DIFFUSE, white_light); glLightfv(GL_LIGHT4, GL_SPECULAR, white_light);
	glLightfv(GL_LIGHT5, GL_POSITION, light_position5); glLightfv(GL_LIGHT5, GL_DIFFUSE, white_light); glLightfv(GL_LIGHT5, GL_SPECULAR, white_light);
	glLightfv(GL_LIGHT6, GL_POSITION, light_position6); glLightfv(GL_LIGHT6, GL_DIFFUSE, white_light); glLightfv(GL_LIGHT6, GL_SPECULAR, white_light);
	glLightfv(GL_LIGHT7, GL_POSITION, light_position7); glLightfv(GL_LIGHT7, GL_DIFFUSE, white_light); glLightfv(GL_LIGHT7, GL_SPECULAR, white_light);

	glEnable(GL_LIGHTING);
	glEnable(GL_LIGHT0);
	glEnable(GL_LIGHT1);
	glEnable(GL_LIGHT2);
	glEnable(GL_LIGHT3);
	glEnable(GL_LIGHT4);
	//glEnable(GL_LIGHT5);
	//glEnable(GL_LIGHT6);
	glEnable(GL_LIGHT7);
}

void MyGLWidget::paintGL()
{
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glClearColor(1.0, 1.0, 1.0, 1.0f);
	//glEnable(GL_DEPTH_TEST);
	//if (m_model != NULL)
	draw();
	glFlush();
	glFinish();
}

void MyGLWidget::draw()
{
	glPushMatrix();

	GLfloat no_mat[4] = { 0.0, 0.0, 0, 1 };
	GLfloat mat_diffuse[4] = { 0, 0, 0, 1 };		//r±íÊ¾´óÖµ£¬b±íÊ¾Ð¡Öµ
	GLfloat mat_specular[4] = { 0.5, 0.5, 0.5, 1 };
	float no_shininess[4] = { 1, 1, 1, 0 };
	glMaterialfv(GL_FRONT, GL_AMBIENT, no_mat);
	glMaterialfv(GL_FRONT, GL_SPECULAR, mat_specular);
	glMaterialfv(GL_FRONT, GL_SHININESS, no_shininess);
	glMaterialfv(GL_FRONT, GL_EMISSION, no_mat);

	//glMatrixMode(GL_MODELVIEW);
	//glLoadIdentity();

	/*QVector4D eye = QVector4D(0.0, 0.0, m, 1.0);
	gluLookAt(eye.x(), eye.y(), eye.z(), 0.0, 0.0, 0.0, 0.0, 1.0, 0.0);*/

	Eigen::Vector3d centroid;
	if (m_model != NULL)
		centroid = m_model->get_centroid();
	else
		centroid.setZero();
	glTranslatef(centroid.x(), centroid.y(), centroid.z());
	glRotatef(m_xRot, 1.0, 0.0, 0.0);
	glRotatef(m_yRot, 0.0, 1.0, 0.0);
	glRotatef(m_zRot, 0.0, 0.0, 1.0);
	glTranslatef(-centroid.x(), -centroid.y(), -centroid.z());

	//glTranslatef(translation_.x(), translation_.y(), translation_.z());

	if (m_model != NULL)
		m_model->draw(m);

	glPopMatrix();
}

void MyGLWidget::resizeGL(int width, int height)
{
	GLfloat nRange = 5.0f;
	if (height == 0) {    // Prevent A Divide By Zero By  
		height = 1;    // Making Height Equal One  
	}
	glViewport(0, 0, width, height);    // Reset The Current Viewport  
	glMatrixMode(GL_PROJECTION);       // Select The Projection Matrix  
	glLoadIdentity();                  // Reset The Projection Matrix  

	if (width <= height)
		glOrtho(-nRange, nRange, -nRange*height / width, nRange*height / width, -nRange, nRange);
	else
		glOrtho(-nRange*width / height, nRange*width / height, -nRange, nRange, -nRange, nRange);
	glMatrixMode(GL_MODELVIEW);      // Select The Modelview Matrix  
	glLoadIdentity();
}

void MyGLWidget::mousePressEvent(QMouseEvent *event)
{
	m_lastPos = event->pos();
	clickEvent = true;
}

void MyGLWidget::mouseMoveEvent(QMouseEvent *event)
{
	int dx = event->x() - m_lastPos.x();
	int dy = event->y() - m_lastPos.y();

	if (event->buttons() & Qt::LeftButton) {
		setXRotation(m_xRot + 3 * dy);
		setYRotation(m_yRot + 3 * dx);
	}
	else if (event->buttons() & Qt::RightButton) {
		setXRotation(m_xRot + 3 * dy);
		setZRotation(m_zRot + 3 * dx);
	}
	else if (event->buttons() & Qt::MiddleButton)
	{
		translation_[0] += 0.001 * dx;
		translation_[1] += 0.001 * dy;
		update();
	}
	m_lastPos = event->pos();
}

void MyGLWidget::wheelEvent(QWheelEvent *e)
{
	if (e->delta()>0)
		m -= 0.1f;
	if (e->delta()<0)
		m += 0.1f;
	if (m < 0.1)
		m = 0.1;
	update();
}

static void qNormalizeAngle(int &angle)
{
	while (angle < 0)
		angle += 360 * 16;
	while (angle > 360 * 16)
		angle -= 360 * 16;
}

void MyGLWidget::setXRotation(int angle)
{
	qNormalizeAngle(angle);
	if (angle != m_xRot) {
		m_xRot = angle;
		update();
	}
}

void MyGLWidget::setYRotation(int angle)
{
	qNormalizeAngle(angle);
	if (angle != m_yRot) {
		m_yRot = angle;
		update();
	}
}

void MyGLWidget::setZRotation(int angle)
{
	qNormalizeAngle(angle);
	if (angle != m_zRot) {
		m_zRot = angle;
		update();
	}
}

void MyGLWidget::rotateModel(float angle, float x, float y, float z)
{
	if (m_model != NULL)
	{
		m_model->rotate(angle, x, y, z);
		update();
	}
}

void MyGLWidget::slotSnapshot(const std::string _filename)
{
	QImage image;
	size_t w(width()), h(height());
	GLenum buffer(GL_BACK);

	try
	{
		image = QImage(w, h, QImage::Format_RGB32);

		std::vector<GLubyte> fbuffer(3 * w*h);

		qApp->processEvents();
		makeCurrent();
		update();
		glFinish();

		glReadBuffer(buffer);
		glPixelStorei(GL_PACK_ALIGNMENT, 1);
		glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
		paintGL();
		glReadPixels(0, 0, w, h, GL_RGB, GL_UNSIGNED_BYTE, &fbuffer[0]);

		unsigned int x, y, offset;

		for (y = 0; y<h; ++y) {
			for (x = 0; x<w; ++x) {
				offset = 3 * (y*w + x);
				image.setPixel(x, h - y - 1, qRgb(fbuffer[offset],
					fbuffer[offset + 1],
					fbuffer[offset + 2]));
			}
		}


		//		QString name = "snapshot-";
		//#if defined(_MSC_VER)
		//		{
		//			std::stringstream s;
		//			QDateTime         dt = QDateTime::currentDateTime();
		//			s << dt.date().year()
		//				<< std::setw(2) << std::setfill('0') << dt.date().month()
		//				<< std::setw(2) << std::setfill('0') << dt.date().day()
		//				<< std::setw(2) << std::setfill('0') << dt.time().hour()
		//				<< std::setw(2) << std::setfill('0') << dt.time().minute()
		//				<< std::setw(2) << std::setfill('0') << dt.time().second();
		//			name += QString(s.str().c_str());
		//		}
		//#else
		//		name += QDateTime::currentDateTime().toString("yyMMddhhmmss");
		//#endif
		QString name = _filename.c_str();
		name += ".png";

		image.save(name, "PNG");
	}
	catch (std::bad_alloc&)
	{
		qWarning("Mem Alloc Error");
	}
}