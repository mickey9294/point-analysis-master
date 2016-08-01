#include "papart.h"

using namespace Eigen;
PAPart::PAPart(OBB * obb) : m_cluster_no(0)
{
	m_label = obb->getLabel();
	Eigen::Vector3f centroid = obb->getCentroid();
	m_translate = Vector3f(centroid.x(), centroid.y(), centroid.z());
	QVector3D scale = obb->getScale();
	m_scale = Vector3f(scale.x(), scale.y(), scale.z());
	Vector3f x_axis = obb->getXAxis();
	Vector3f y_axis = obb->getYAxis();
	Vector3f z_axis = obb->getZAxis();
	
	Matrix3f base_axes = Matrix3f::Identity();
	m_rotate.col(0) = x_axis;
	m_rotate.col(1) = y_axis;
	m_rotate.col(2) = z_axis;

	const Vector3f gravity(0.0, -1.0, 0.0);
	Matrix<float, 3, 4> Rt;
	Rt.block<3, 3>(0, 0) = m_rotate;
	Rt.block<3, 1>(0, 3) = m_translate;
	Matrix4f Sb;
	Sb << m_scale[0] , 0.0f , 0.0f , 0.0f, 
		0.0f , m_scale[1] , 0.0f , 0.0f,
		0.0f , 0.0f , m_scale[2] , 0.0f,
		0.0f , 0.0f , 0.0f , 1.0f;
	
	m_height = (gravity.transpose() * Rt * Sb).transpose();

	m_axes.col(0) = x_axis;
	m_axes.col(1) = y_axis;
	m_axes.col(2) = z_axis;
}

PAPart::PAPart(const PAPart &part)
{
	m_rotate = Matrix3f(part.getRotMat());
	m_translate = Vector3f(part.getTransVec());
	m_height = Vector4f(part.getHeight());
	m_scale = Vector3f(part.getScale());
	m_label = part.getLabel();

	m_axes = part.getAxes();
	m_vertices_indices = part.getVerticesIndices();
	m_cluster_no = part.getClusterNo();
}

PAPart::PAPart() : m_cluster_no(0), m_label(0)
{
	m_rotate.setIdentity();
	m_translate.setZero();
	m_scale.setZero();
	m_height.setZero();
	m_axes.setIdentity();
}


PAPart::~PAPart()
{
}

Matrix3f PAPart::getRotMat() const
{
	return m_rotate;
}

QMatrix4x4 PAPart::getQRotMat()
{
	QVector4D col0(m_rotate.col(0).x(), m_rotate.col(0).y(), m_rotate.col(0).z(), 0);
	QVector4D col1(m_rotate.col(1).x(), m_rotate.col(1).y(), m_rotate.col(1).z(), 0);
	QVector4D col2(m_rotate.col(2).x(), m_rotate.col(2).y(), m_rotate.col(2).z(), 0);
	QVector4D col3(0, 0, 0, 1.0);
	QMatrix4x4 qRotMat;
	qRotMat.setColumn(0, col0);
	qRotMat.setColumn(1, col1);
	qRotMat.setColumn(2, col2);
	qRotMat.setColumn(3, col3);
	qDebug() << qRotMat;

	return qRotMat;
}

Vector3f PAPart::getTransVec() const
{
	return m_translate;
}

Vector3f PAPart::getScale() const
{
	return m_scale;
}

QVector3D PAPart::getQTransVec()
{
	return QVector3D(m_translate.x(), m_translate.y(), m_translate.z());
}

QVector3D PAPart::getQScale()
{
	return QVector3D(m_scale.x(), m_scale.y(), m_scale.z());
}

Vector4f PAPart::getHeight() const
{
	return m_height;
}

QVector4D PAPart::getQHeight()
{
	return QVector4D(m_height[0], m_height[1], m_height[2], m_height[3]);
}

Matrix3f PAPart::getAxes() const
{
	return Matrix3f(m_axes);
}

bool PAPart::isInside(Vector3f point)
{
	Vector3f centroid(m_translate.x(), m_translate.y(), m_translate.z());
	Vector3f vec = point - centroid;
	float projection_x = std::fabs(vec.dot(m_axes.col(0)));
	float projection_y = std::fabs(vec.dot(m_axes.col(1)));
	float projection_z = std::fabs(vec.dot(m_axes.col(2)));

	if (projection_x <= m_scale.x() && projection_y <= m_scale.y() && projection_z <= m_scale.z())
		return true;
	else
		return false;
}

void PAPart::setVerticesIndices(QList<int> indices)
{
	m_vertices_indices.resize(indices.size());
	int i = 0;
	for (QList<int>::iterator it = indices.begin(); it != indices.end(); ++it)
		m_vertices_indices[i++] = *it;
}

std::vector<int> PAPart::getVerticesIndices() const
{
	return std::vector<int>(m_vertices_indices);
}

void PAPart::setClusterNo(int cluster_no)
{
	m_cluster_no = cluster_no;
}

using namespace std;
void PAPart::saveToFile(string name)
{
	string path = "../data/candidates/" + name + ".txt";
	ofstream out(path.c_str());

	out << m_label << " " << m_cluster_no << endl;
	for (int i = 0; i < m_rotate.rows(); i++)
		out << m_rotate(i, 0) << " " << m_rotate(i, 1) << " " << m_rotate(i, 2) << endl;

	out << m_translate(0) << " " << m_translate(1) << " " << m_translate(2) << endl;

	out << m_scale(0) << " " << m_scale(1) << " " << m_scale(2) << endl;

	out << m_height(0) << " " << m_height(1) << " " << m_height(2) << " " << m_height(3) << endl;

	for (int i = 0; i < m_axes.rows(); i++)
		out << m_axes(i, 0) << " " << m_axes(i, 1) << " " << m_axes(i, 2) << endl;

	for (vector<int>::iterator it = m_vertices_indices.begin(); it != m_vertices_indices.end(); ++it)
		out << *it << endl;

	out.close();
}

PAPart::PAPart(string path)
{
	ifstream in(path.c_str());

	if (in.is_open())
	{
		char buffer[1024];

		/* Read label and cluster no */
		in.getline(buffer, 512);
		QString label_str(buffer);
		m_label = label_str.section(' ', 0, 0).toInt();
		m_cluster_no = label_str.section(' ', 1, 1).toInt();

		/* Read the rotation matrix */
		for (int i = 0; i < 3; i++)
		{
			in.getline(buffer, 512);
			QString rot_str(buffer);
			m_rotate(i, 0) = rot_str.section(' ', 0, 0).toFloat();
			m_rotate(i, 1) = rot_str.section(' ', 1, 1).toFloat();
			m_rotate(i, 2) = rot_str.section(' ', 2, 2).toFloat();
		}

		/* Read the translate vector */
		in.getline(buffer, 512);
		QString trans_str(buffer);
		m_translate(0) = trans_str.section(' ', 0, 0).toFloat();
		m_translate(1) = trans_str.section(' ', 1, 1).toFloat();
		m_translate(2) = trans_str.section(' ', 2, 2).toFloat();

		/* Read the scale vector */
		in.getline(buffer, 512);
		QString scale_str(buffer);
		m_scale(0) = scale_str.section(' ', 0, 0).toFloat();
		m_scale(1) = scale_str.section(' ', 1, 1).toFloat();
		m_scale(2) = scale_str.section(' ', 2, 2).toFloat();

		/* Read the height vector */
		in.getline(buffer, 512);
		QStringList h_str = QString(buffer).split(' ');
		for (int i = 0; i < h_str.size(); i++)
			m_height(i) = h_str.at(i).toFloat();

		/* Read the 3 axes */
		for (int i = 0; i < 3; i++)
		{
			in.getline(buffer, 512);
			QString axis_str(buffer);
			m_axes(i, 0) = axis_str.section(' ', 0, 0).toFloat();
			m_axes(i, 1) = axis_str.section(' ', 1, 1).toFloat();
			m_axes(i, 2) = axis_str.section(' ', 2, 2).toFloat();
		}

		/* Read the vertices indices */
		vector<int> indices_list;
		while (!in.eof())
		{
			in.getline(buffer, 64);
			if (strlen(buffer) > 0)
			{
				int idx = atoi(buffer);
				indices_list.push_back(idx);
			}
		}
		m_vertices_indices.resize(indices_list.size());
		memcpy(m_vertices_indices.data(), indices_list.data(), indices_list.size() * sizeof(int));

		in.close();
	}
}

OBB * PAPart::generateOBB()
{
	Vector3f _x_axis = m_axes.col(0);
	Vector3f _y_axis = m_axes.col(1);
	Vector3f _z_axis = m_axes.col(2);

	float x_length = m_scale.x();
	float y_length = m_scale.y();
	float z_length = m_scale.z();
	OBB *obb = new OBB(_x_axis, _y_axis, _z_axis, m_translate, (double)x_length, (double)y_length, (double)z_length, m_label);
	obb->triangulate();
	return obb;
}