#include "MyGLWidget.h"

MyGLWidget::MyGLWidget(QWidget *parent)
	: QOpenGLWidget(parent),
	clearColor_(Qt::white),
	program_(0)
{
	scale_ = 1;
	m_xRot = 0;
	m_yRot = 0;
	m_zRot = 0;
	view_z_ = 4.0;
}

MyGLWidget::MyGLWidget(const MeshModel &mesh, const std::vector<Eigen::Vector2d> &verts_2d, const QPixmap &image, QWidget *parent)
	: QOpenGLWidget(parent),
	clearColor_(Qt::white),
	program_(0),
	scale_(1),
	m_xRot(0), m_yRot(0), m_zRot(0),
	view_z_(4.0)
{
	const std::vector<std::vector<int>>&face_circuits = mesh.const_faces();
	const std::vector<Eigen::Vector3d> &verts_3d = mesh.const_vertices();
	const std::vector<Eigen::Vector3i> &triangles = mesh.const_triangles();

	faces_.resize(triangles.size());
	for (int i = 0; i < triangles.size(); i++)
	{
		faces_[i][0] = triangles[i][0];
		faces_[i][1] = triangles[i][1];
		faces_[i][2] = triangles[i][2];
	}

	verts_3d_.resize(verts_3d.size());
	for (int i = 0; i < verts_3d.size(); i++)
	{
		verts_3d_[i][0] = verts_3d[i][0];
		verts_3d_[i][1] = verts_3d[i][1];
		verts_3d_[i][2] = verts_3d[i][2];
	}

	double width = image.width();
	double height = image.height();
	verts_2d_.resize(verts_2d.size());
	for (int i = 0; i < verts_2d.size(); i++)
	{
		verts_2d_[i][0] = width / 2.0 + verts_2d[i][0];
		verts_2d_[i][1] = height /2.0 - verts_2d[i][1];
	}

	tex_image_ = image.toImage();
}

MyGLWidget::~MyGLWidget()
{
	makeCurrent();
	vbo_.destroy();
	if (textures_)
		delete textures_;
	if(program_)
		delete program_;
	doneCurrent();
}

void MyGLWidget::initializeGL()
{
	initializeOpenGLFunctions();

	makeObject();

	glEnable(GL_DEPTH_TEST);
	glDisable(GL_CULL_FACE);
	
#define PROGRAM_VERTEX_ATTRIBUTE 0
#define PROGRAM_TEXCOORD_ATTRIBUTE 1

	QOpenGLShader *vshader = new QOpenGLShader(QOpenGLShader::Vertex, this);
	const char *vsrc =
		"attribute highp vec4 vertex;\n"
		"attribute mediump vec4 texCoord;\n"
		"varying mediump vec4 texc;\n"
		"uniform mediump mat4 matrix;\n"
		"void main(void)\n"
		"{\n"
		"    gl_Position = matrix * vertex;\n"
		"    texc = texCoord;\n"
		"}\n";
	vshader->compileSourceCode(vsrc);

	QOpenGLShader *fshader = new QOpenGLShader(QOpenGLShader::Fragment, this);
	const char *fsrc =
		"uniform sampler2D texture;\n"
		"varying mediump vec4 texc;\n"
		"void main(void)\n"
		"{\n"
		"    gl_FragColor = texture2D(texture, texc.st);\n"
		"}\n";
	fshader->compileSourceCode(fsrc);

	program_ = new QOpenGLShaderProgram;
	program_->addShader(vshader);
	program_->addShader(fshader);
	program_->bindAttributeLocation("vertex", PROGRAM_VERTEX_ATTRIBUTE);
	program_->bindAttributeLocation("texCoord", PROGRAM_TEXCOORD_ATTRIBUTE);
	program_->link();

	program_->bind();
	program_->setUniformValue("texture", 0);
	
}

void MyGLWidget::paintGL()
{
	glClearColor(clearColor_.redF(), clearColor_.greenF(), clearColor_.blueF(), clearColor_.alphaF());
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	QMatrix4x4 m;
	//m.ortho(-0.5f, +0.5f, +0.5f, -0.5f, 4.0f, 15.0f);
	//m.translate(0.0f, 0.0f, -10.0f);
	m.rotate(m_xRot / 16.0f, 1.0f, 0.0f, 0.0f);
	m.rotate(m_yRot / 16.0f, 0.0f, 1.0f, 0.0f);
	m.rotate(m_zRot / 16.0f, 0.0f, 0.0f, 1.0f);
	QMatrix4x4 camera;
	camera.lookAt(QVector3D(0, 0, view_z_), QVector3D(0, 0, 0), QVector3D(0, 1, 0));


	program_->setUniformValue("matrix", proj_ * camera * m);
	program_->enableAttributeArray(PROGRAM_VERTEX_ATTRIBUTE);
	program_->enableAttributeArray(PROGRAM_TEXCOORD_ATTRIBUTE);
	program_->setAttributeBuffer(PROGRAM_VERTEX_ATTRIBUTE, GL_FLOAT, 0, 3, 5 * sizeof(GLfloat));
	program_->setAttributeBuffer(PROGRAM_TEXCOORD_ATTRIBUTE, GL_FLOAT, 3 * sizeof(GLfloat), 2, 5 * sizeof(GLfloat));

	textures_->bind();
	//GLint start = 0;
	for (int i = 0; i < faces_.size(); ++i) {
		glDrawArrays(GL_TRIANGLES, 3 * i, 3);
		//start += faces_[i].size();
	}

}

void MyGLWidget::resizeGL(int width, int height)
{
	//GLfloat nRange = 3.0 * std::tan(25.0 * M_PI / 180.0);

	//if (height == 0) {    // Prevent A Divide By Zero By  
	//	height = 1;    // Making Height Equal One  
	//}
	//glViewport(0, 0, width, height);    // Reset The Current Viewport  
	//glMatrixMode(GL_PROJECTION);       // Select The Projection Matrix  
	//glLoadIdentity();                  // Reset The Projection Matrix  

	//if (width <= height)
	//	glOrtho(-nRange, nRange, -nRange*height / width, nRange*height / width, 3.0, 10.0);
	//else
	//	glOrtho(-nRange*width / height, nRange*width / height, -nRange, nRange, 3.0, 10.0);
	//glMatrixMode(GL_MODELVIEW);      // Select The Modelview Matrix  
	//glLoadIdentity();

	proj_.setToIdentity();
	proj_.perspective(30.0f, GLfloat(width) / height, 2.0f, 100.0f);
}

void MyGLWidget::mousePressEvent(QMouseEvent *event)
{
	lastPos_ = event->pos();
	//clickEvent = true;
}

void MyGLWidget::mouseMoveEvent(QMouseEvent *event)
{
	int dx = event->x() - lastPos_.x();
	int dy = event->y() - lastPos_.y();

	if (event->buttons() & Qt::LeftButton) {
		setXRotation(m_xRot + 3 * dy);
		setYRotation(m_yRot + 3 * dx);
	}
	else if (event->buttons() & Qt::RightButton) {
		setXRotation(m_xRot + 3 * dy);
		setZRotation(m_zRot + 3 * dx);
	}
	/*else if (event->buttons() & Qt::MiddleButton)
	{
		translation_[0] += 0.001 * dx;
		translation_[1] += 0.001 * dy;
		update();
	}*/
	lastPos_ = event->pos();
}

void MyGLWidget::wheelEvent(QWheelEvent *e)
{
	if (e->delta()>0)
		view_z_ -= 0.1f;
	if (e->delta()<0)
		view_z_ += 0.1f;
	if (view_z_ < 0.1)
		view_z_ = 0.1;
	update();
}

void MyGLWidget::makeObject()
{
	double width = tex_image_.width();
	double height = tex_image_.height();
	textures_ = new QOpenGLTexture(tex_image_);
	QVector<GLfloat> vertData;
	for (int i = 0; i < faces_.size(); i++)
	{
		Eigen::Vector3i triangle = faces_[i];

		for (int j = 0; j < 3; j++)
		{
			int vert_id = triangle[j];
			const Eigen::Vector3d &vert = verts_3d_[vert_id];
			const Eigen::Vector2d &vert_2d = verts_2d_[vert_id];

			/* vertex position */
			vertData.append(scale_ * vert[0]);
			vertData.append(scale_ * vert[1]);
			vertData.append(scale_ * vert[2]);

			/* texture coordinate */
			vertData.append(vert_2d[0] / width);
			vertData.append(vert_2d[1] / height);
		}
	}

	vbo_.create();
	vbo_.bind();
	vbo_.allocate(vertData.constData(), vertData.count() * sizeof(GLfloat));
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