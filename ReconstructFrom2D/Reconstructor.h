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

	void reconstruct(const std::vector<Eigen::Vector2d> & verts_2d, const Eigen::VectorXd &q,
		const std::vector<std::vector<int>> &vert_to_face_map, const std::vector<PlanarFace> &faces,
		std::vector<Eigen::Vector3d> &verts_3d, std::vector<Eigen::Vector3i> &triangles);

private:
	float focal_length_;

	void output_shape(const std::vector<Eigen::Vector3d> &verts, const std::vector<Eigen::Vector3i> &triangles);
};
