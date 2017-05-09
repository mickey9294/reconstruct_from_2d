#ifndef MYGLWIDGET_H
#define MYGLWIDGET_H

#include <QOpenGLWidget>
#include <QtOpenGL>
#include <QOpenGLFunctions>
#include <QOpenGLBuffer>
#include <QPixmap>
#include <QVector>
#include <QVector3D>

#include <GL/glu.h>
#include <Eigen\Core>
#include <memory>
#include <Windows.h>

#include "ShapeModel.h"
#include "PCModel.h"
#include "MeshModel.h"

#ifndef PI
#define PI 3.1415926536
#endif

QT_FORWARD_DECLARE_CLASS(QOpenGLShaderProgram);
QT_FORWARD_DECLARE_CLASS(QOpenGLTexture)

class MyGLWidget : public QOpenGLWidget, protected QOpenGLFunctions
{
	Q_OBJECT

public:
	MyGLWidget(QWidget *parent = 0);
	MyGLWidget(const MeshModel &mesh, const std::vector<Eigen::Vector2d> &verts_2d, 
		const QPixmap &image, QWidget *parent = 0);
	~MyGLWidget();

	int xRotation() const { return m_xRot; }
	int yRotation() const { return m_yRot; }
	int zRotation() const { return m_zRot; }

	public slots:
	void slotSnapshot(const std::string _filename);
	void setXRotation(int angle);
	void setYRotation(int angle);
	void setZRotation(int angle);

signals:
	void xRotationChanged(int angle);
	void yRotationChanged(int angle);
	void zRotationChanged(int angle);
	void addDebugText(QString text);

protected:
	void initializeGL();
	void paintGL();
	void resizeGL(int w, int h);
	void mousePressEvent(QMouseEvent *event);
	void mouseMoveEvent(QMouseEvent *event);
	//void mouseReleaseEvent(QMouseEvent *event);
	void wheelEvent(QWheelEvent *e);

private:
	GLfloat m_xRot;
	GLfloat m_yRot;
	GLfloat m_zRot;
	float scale_;
	QPoint lastPos_;
	QColor clearColor_;
	QOpenGLTexture *textures_;
	QOpenGLShaderProgram *program_;
	QOpenGLBuffer vbo_;

	std::vector<Eigen::Vector3i> faces_;
	std::vector<Eigen::Vector3d> verts_3d_;
	std::vector<Eigen::Vector2d> verts_2d_;
	QImage tex_image_;

	QMatrix4x4 proj_;
	GLfloat view_z_;

	void makeObject();
};

#endif