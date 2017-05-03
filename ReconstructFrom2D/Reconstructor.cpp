#include "Reconstructor.h"



Reconstructor::Reconstructor(QObject *parent)
	: QObject(parent)
{
}

Reconstructor::Reconstructor(float focal_length, QObject *parent)
	: QObject(parent)
{
	focal_length_ = focal_length;
}

Reconstructor::~Reconstructor()
{

}

void Reconstructor::reconstruct(const std::vector<Eigen::Vector2d> & verts_2d, const Eigen::VectorXd &q,
	const std::vector<std::vector<int>> &vert_to_face_map, const std::vector<PlanarFace> &faces)
{
	int Nf = q.size() / 3;
	int num_verts = verts_2d.size();

	std::vector<Eigen::Vector3d> verts_3d(num_verts);
	std::vector<Eigen::Vector3d> face_normals(Nf);

	for (int i = 0; i < Nf; i++)
	{
		face_normals[i] = q.segment(3 * i, 3);
	}

	for (int i = 0; i < num_verts; i++)
	{
		const std::vector<int> &related_faces_id = vert_to_face_map[i];

		Eigen::Vector3d x_aug;
		x_aug.segment(0, 2) = verts_2d[i];
		x_aug[2] = -focal_length_;

		float Z = 0;
		for (int j = 0; j < related_faces_id.size(); j++)
		{
			int related_face_id = related_faces_id[j];
			
			Z += focal_length_ / (face_normals[related_face_id].transpose() * x_aug);
		}
		Z /= (float)related_faces_id.size();
		float X = -Z * x_aug[0] / focal_length_;
		float Y = -Z * x_aug[1] / focal_length_;

		verts_3d[i][0] = X;
		verts_3d[i][1] = Y;
		verts_3d[i][2] = Z;
	}

	std::list<Eigen::Vector3i> triangles;
	for (int i = 0; i < faces.size(); i++)
	{
		const std::vector<int> &circuit = faces[i].const_circuit();

		int first_vert = circuit.front();
		for (int j = 1; j < circuit.size() - 1; j++)
		{
			Eigen::Vector3i tri(first_vert, circuit[j], circuit[j + 1]);
			triangles.push_back(tri);
		}
	}

	output_shape(verts_3d, triangles);
}

void Reconstructor::output_shape(const std::vector<Eigen::Vector3d>& verts, const std::list<Eigen::Vector3i>& triangles)
{
	std::ofstream out("..\\shape.off");
	if (out.is_open())
	{
		out << "OFF" << std::endl;
		out << verts.size() << " " << triangles.size() << " 0" << std::endl;

		for (std::vector<Eigen::Vector3d>::const_iterator v_it = verts.begin();
			v_it != verts.end(); ++v_it)
			out << v_it->operator[](0) << " " << v_it->operator[](1) << " " << v_it->operator[](2) << std::endl;

		for (std::list<Eigen::Vector3i>::const_iterator f_it = triangles.begin();
			f_it != triangles.end(); ++f_it)
			out << "3 " << f_it->operator[](0) << " " << f_it->operator[](1)
				<< " " << f_it->operator[](2) << std::endl;

		out.close();
	}
}

