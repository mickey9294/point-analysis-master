#include "obb.h"

OBB::OBB(QObject *parent)
	: QObject(parent), m_count(0)
{

}

OBB::OBB(Eigen::Vector3f xAxis, Eigen::Vector3f yAxis, Eigen::Vector3f zAxis, Eigen::Vector3f centroid,
	double xLength, double yLength, double zLength, int label, QObject *parent)
	: QObject(parent), m_count(0)
{
	x_axis = xAxis;
	y_axis = yAxis;
	z_axis = zAxis;

	x_length = xLength;
	y_length = yLength;
	z_length = zLength;
	m_centroid = centroid;
	m_label = label;

	if (m_label >= 0 && m_label <= 10)
	{
		m_color.setX(COLORS[m_label][0]);
		m_color.setY(COLORS[m_label][1]);
		m_color.setZ(COLORS[m_label][2]);
	}
	else
	{
		m_color.setX(COLORS[0][0]);
		m_color.setY(COLORS[0][1]);
		m_color.setZ(COLORS[0][2]);
	}
}

OBB::OBB(const OBB &obb)
{
	x_axis = obb.getXAxis();
	y_axis = obb.getYAxis();
	z_axis = obb.getZAxis();
	QVector3D scale = obb.getScale();
	x_length = scale.x();
	y_length = scale.y();
	z_length = scale.z();
	m_centroid = obb.getCentroid();
	m_color = obb.getColor();
	m_count = obb.count();
	std::memcpy(m_data.data(), obb.constData(), m_count * sizeof(float));
	m_label = obb.getLabel();
	m_vertices = QVector<QVector3D>(obb.getVertices());
	m_sample_points = obb.getSamplePoints();
}

OBB::~OBB()
{

}

Eigen::Vector3f OBB::getXAxis() const
{
	return x_axis;
}

Eigen::Vector3f OBB::getYAxis() const
{
	return y_axis;
}

Eigen::Vector3f OBB::getZAxis() const
{
	return z_axis;
}

QVector3D OBB::getScale() const
{
	return QVector3D(x_length, y_length, z_length);
}

QVector<Eigen::Vector3f> OBB::getAxes() const
{
	QVector<Eigen::Vector3f> axes(3);
	axes[0] = Eigen::Vector3f(x_axis);
	axes[1] = Eigen::Vector3f(y_axis);
	axes[2] = Eigen::Vector3f(z_axis);

	return axes;
}

void OBB::triangulate()
{
	m_vertices.resize(8);

	QMatrix4x4 translate_matrices[8];
	translate_matrices[0].setToIdentity();
	translate_matrices[0].translate(eigen_vector3f_to_qvector3d(x_length / 2.0 * x_axis));
	translate_matrices[0].translate(eigen_vector3f_to_qvector3d(z_length / 2.0 * z_axis));
	translate_matrices[0].translate(eigen_vector3f_to_qvector3d(y_length / 2.0  * y_axis));
	translate_matrices[1].setToIdentity();
	translate_matrices[1].translate(eigen_vector3f_to_qvector3d(-x_length / 2.0  * x_axis));
	translate_matrices[1].translate(eigen_vector3f_to_qvector3d(z_length / 2.0  * z_axis));
	translate_matrices[1].translate(eigen_vector3f_to_qvector3d(y_length / 2.0  * y_axis));
	translate_matrices[2].setToIdentity();
	translate_matrices[2].translate(eigen_vector3f_to_qvector3d(-x_length / 2.0  * x_axis));
	translate_matrices[2].translate(eigen_vector3f_to_qvector3d(-y_length / 2.0  * y_axis));
	translate_matrices[2].translate(eigen_vector3f_to_qvector3d(z_length / 2.0  * z_axis));
	translate_matrices[3].setToIdentity();
	translate_matrices[3].translate(eigen_vector3f_to_qvector3d(x_length / 2.0  * x_axis));
	translate_matrices[3].translate(eigen_vector3f_to_qvector3d(-y_length / 2.0  * y_axis));
	translate_matrices[3].translate(eigen_vector3f_to_qvector3d(z_length / 2.0  * z_axis));
	translate_matrices[4].setToIdentity();
	translate_matrices[4].translate(eigen_vector3f_to_qvector3d(x_length / 2.0  * x_axis));
	translate_matrices[4].translate(eigen_vector3f_to_qvector3d(y_length / 2.0  * y_axis));
	translate_matrices[4].translate(eigen_vector3f_to_qvector3d(-z_length / 2.0  * z_axis));
	translate_matrices[5].setToIdentity();
	translate_matrices[5].translate(eigen_vector3f_to_qvector3d(-x_length / 2.0  * x_axis));
	translate_matrices[5].translate(eigen_vector3f_to_qvector3d(y_length / 2.0  * y_axis));
	translate_matrices[5].translate(eigen_vector3f_to_qvector3d(-z_length / 2.0  * z_axis));
	translate_matrices[6].setToIdentity();
	translate_matrices[6].translate(eigen_vector3f_to_qvector3d(-x_length / 2.0  * x_axis));
	translate_matrices[6].translate(eigen_vector3f_to_qvector3d(-y_length / 2.0  * y_axis));
	translate_matrices[6].translate(eigen_vector3f_to_qvector3d(-z_length / 2.0  * z_axis));
	translate_matrices[7].setToIdentity();
	translate_matrices[7].translate(eigen_vector3f_to_qvector3d(x_length / 2.0  * x_axis));
	translate_matrices[7].translate(eigen_vector3f_to_qvector3d(-y_length / 2.0  * y_axis));
	translate_matrices[7].translate(eigen_vector3f_to_qvector3d(-z_length / 2.0  * z_axis));

	for (int i = 0; i < 8; i++)
		m_vertices[i] = translate_matrices[i] * eigen_vector3f_to_qvector3d(m_centroid);

	Eigen::Vector3f face_normals[6] = {
		x_axis, y_axis, z_axis, -x_axis, -y_axis, -z_axis
	};

	m_data.resize(192);

	add(m_vertices[0], m_vertices[1], m_vertices[3], face_normals[2]);
	add(m_vertices[1], m_vertices[2], m_vertices[3], face_normals[2]);
	add(m_vertices[0], m_vertices[3], m_vertices[7], face_normals[0]);
	add(m_vertices[0], m_vertices[7], m_vertices[4], face_normals[0]);
	add(m_vertices[0], m_vertices[4], m_vertices[1], face_normals[1]);
	add(m_vertices[1], m_vertices[4], m_vertices[5], face_normals[1]);
	add(m_vertices[4], m_vertices[7], m_vertices[6], face_normals[5]);
	add(m_vertices[4], m_vertices[6], m_vertices[5], face_normals[5]);
	add(m_vertices[1], m_vertices[5], m_vertices[2], face_normals[3]);
	add(m_vertices[2], m_vertices[5], m_vertices[6], face_normals[3]);
	add(m_vertices[2], m_vertices[7], m_vertices[3], face_normals[4]);
	add(m_vertices[2], m_vertices[6], m_vertices[7], face_normals[4]);
}

void OBB::add(QVector3D v0, QVector3D v1, QVector3D v2, Eigen::Vector3f normal)
{
	float * triangle = m_data.data() + m_count;
	/* The fist vertex of the triangle */
	triangle[0] = v0.x();
	triangle[1] = v0.y();
	triangle[2] = v0.z();
	/* The second vertex of the triangle */
	triangle[3] = v1.x();
	triangle[4] = v1.y();
	triangle[5] = v1.z();
	/* The third vertex of the triangle */
	triangle[6] = v2.x();
	triangle[7] = v2.y();
	triangle[8] = v2.z();
	/* The normal vector of the triangle facet */
	triangle[9] = normal.x();
	triangle[10] = normal.y();
	triangle[11] = normal.z();

	m_count += 12;
}

QVector<QVector3D> OBB::getVertices() const
{
	return m_vertices;
}

void OBB::setColor(QVector3D color)
{
	m_color = color;
}

int OBB::samplePoints(int num_of_samples)
{
	using namespace Utils_sampling;

	if (!m_sample_points.empty())
		m_sample_points.clear();

	if (m_count > 0)
	{
		if (num_of_samples == 0)
			num_of_samples = 1000;

		std::vector<Utils_sampling::Vec3> verts(vertexCount());
		std::vector<Vec3> nors(vertexCount());
		std::vector<int> tris(vertexCount());

		int idx = 0;
		for (int i = 0; i < m_count - 24; i += 12)
		{
			float * triangle = m_data.data() + i;

			Vec3 v0(triangle[0], triangle[1], triangle[2]);
			Vec3 v1(triangle[3], triangle[4], triangle[5]);
			Vec3 v2(triangle[6], triangle[7], triangle[8]);

			Vec3 n(triangle[9], triangle[10], triangle[11]);

			verts[idx] = v0;
			tris[idx] = idx;
			nors[idx++] = n;
			verts[idx] = v1;
			tris[idx] = idx;
			nors[idx++] = n;
			verts[idx] = v2;
			tris[idx] = idx;
			nors[idx++] = n;
		}

		std::vector<Vec3> samples_pos;
		std::vector<Vec3> samples_nor;

		poisson_disk(0, num_of_samples, verts, nors, tris, samples_pos, samples_nor);

		m_sample_points.resize(samples_pos.size());

		idx = 0;
		for (std::vector<Vec3>::iterator it = samples_pos.begin(); it != samples_pos.end(); ++it)
		{
			Vec3 point = *it;
			Eigen::Vector3f sample_point(point.x, point.y, point.z);
			m_sample_points[idx++] = sample_point;
		}

		return samples_pos.size();
	}
	else
		return 0;
}

QVector<Eigen::Vector3f> OBB::getSamplePoints() const
{
	return m_sample_points;
}

void OBB::setSamplePoints(Eigen::MatrixXf samples_mat)
{
	if (!m_sample_points.empty())
		m_sample_points.clear();

	m_sample_points.resize(samples_mat.size());

	for (int i = 0; i < samples_mat.cols(); i++)
	{
		Eigen::Vector3f col_sample = samples_mat.col(i);
		m_sample_points[i] = col_sample;
	}
}

QVector3D OBB::eigen_vector3f_to_qvector3d(Eigen::Vector3f vec)
{
	QVector3D qvec(vec.x(), vec.y(), vec.z());
	return qvec;
}

using namespace Eigen;

void OBB::setXAxis(Vector3f xAxis)
{
	x_axis = xAxis;
}

void OBB::setYAxis(Vector3f yAxis)
{
	y_axis = yAxis;
}

void OBB::setZAxis(Vector3f zAxis)
{
	z_axis = zAxis;
}

void OBB::setCentroid(Vector3f centroid)
{
	m_centroid = centroid;
}

void OBB::setXAxis(Vector3d xAxis)
{
	x_axis = Vector3f((float)xAxis.x(), (float)xAxis.y(), (float)xAxis.z());
}

void OBB::setYAxis(Vector3d yAxis)
{
	y_axis = Vector3f((float)yAxis.x(), (float)yAxis.y(), (float)yAxis.z());
}

void OBB::setZAxis(Vector3d zAxis)
{
	z_axis = Vector3f((float)zAxis.x(), (float)zAxis.y(), (float)zAxis.z());
}

void OBB::setCentroid(Vector3d centroid)
{
	m_centroid = Vector3f((float)centroid.x(), (float)centroid.y(), (float)centroid.z());
}