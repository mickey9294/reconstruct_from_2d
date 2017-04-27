#pragma once

#include <QThread>
#include <qfileinfo.h>
//#include <QMutex>
//#include <QWaitCondition>

#include <vector>
#include <iostream>
#include <fstream>

#include <Eigen\Core>

#include "PlanarFace.h"

class Reconstructor : public QObject
{
	Q_OBJECT

public:
	Reconstructor(QObject *parent = 0);
	Reconstructor(float focal_length, QObject *parent = 0);
	~Reconstructor();

	void reconstruct(const std::vector<Eigen::Vector2f> & verts_2d, const Eigen::VectorXf &q,
		const std::vector<std::vector<int>> &vert_to_face_map, const std::vector<PlanarFace> &faces);

private:
	float focal_length_;

	void output_shape(const std::vector<Eigen::Vector3f> &verts, const std::list<Eigen::Vector3i> &triangles);
};
