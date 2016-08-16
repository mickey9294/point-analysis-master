#include "obb.h"

using namespace Eigen;

OBB::OBB(QObject *parent)
	: QObject(parent)
{
	x_axis = Vector3f(1.0, 0, 0);
	y_axis = Vector3f(0, 1.0, 0);
	z_axis = Vector3f(0, 0, 1.0);
	x_length = 1.0;
	y_length = 1.0;
	z_length = 1.0;
	m_centroid.setZero();
}

OBB::OBB(Eigen::Vector3f xAxis, Eigen::Vector3f yAxis, Eigen::Vector3f zAxis, Eigen::Vector3f centroid,
	double xLength, double yLength, double zLength, int label, QObject *parent)
	: QObject(parent)
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

	triangulate();
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
	m_label = obb.getLabel();
	m_vertices = obb.getVertices();
	m_faces = obb.getFaces();
	m_faces_normals = obb.getFacesNormals();
	m_sample_points = obb.getSamplePoints();
}

OBB::OBB(const OBB * obb)
{
	x_axis = obb->getXAxis();
	y_axis = obb->getYAxis();
	z_axis = obb->getZAxis();
	QVector3D scale = obb->getScale();
	x_length = scale.x();
	y_length = scale.y();
	z_length = scale.z();
	m_centroid = obb->getCentroid();
	m_color = obb->getColor();
	m_label = obb->getLabel();
	m_vertices = obb->getVertices();
	m_faces = obb->getFaces();
	m_faces_normals = obb->getFacesNormals();
	m_sample_points = obb->getSamplePoints();
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

Eigen::Matrix3f OBB::getAxes() const
{
	Eigen::Matrix3f axes;
	axes.col(0) = x_axis;
	axes.col(1) = y_axis;
	axes.col(2) = z_axis;

	return axes;
}

Matrix<float, 4, 3> OBB::getAugmentedAxes()
{
	Eigen::Matrix<float, 4, 3> axes;
	axes.block<3, 1>(0, 0) = x_axis;
	axes.block<3, 1>(0, 1) = y_axis;
	axes.block<3, 1>(0, 2) = z_axis;
	axes.block<1, 3>(3, 0) = Vector3f::Zero();
	return axes;
}

void OBB::triangulate()
{
	m_vertices.resize(8);
	
	Eigen::Translation<float, 3> translations[8];
	Eigen::Translation<float, 3> trans00(x_length / 2.0 * x_axis);
	Eigen::Translation<float, 3> trans01(z_length / 2.0 * z_axis);
	Eigen::Translation<float, 3> trans02(y_length / 2.0 * y_axis);
	translations[0] = trans00 * trans01 * trans02;
	Eigen::Translation<float, 3> trans10(-x_length / 2.0 * x_axis);
	Eigen::Translation<float, 3> trans11(z_length / 2.0 * z_axis);
	Eigen::Translation<float, 3> trans12(y_length / 2.0 * y_axis);
	translations[1] = trans10 * trans11 * trans12;
	Eigen::Translation<float, 3> trans20(-x_length / 2.0 * x_axis); 
	Eigen::Translation<float, 3> trans21(-y_length / 2.0 * y_axis);
	Eigen::Translation<float, 3> trans22(z_length / 2.0 * z_axis);
	translations[2] = trans20 * trans21 * trans22;
	Eigen::Translation<float, 3> trans30(x_length / 2.0 * x_axis);
	Eigen::Translation<float, 3> trans31(-y_length / 2.0 * y_axis);
	Eigen::Translation<float, 3> trans32(z_length / 2.0 * z_axis);
	translations[3] = trans30 * trans31 * trans32;
	Eigen::Translation<float, 3> trans40(x_length / 2.0 * x_axis);
	Eigen::Translation<float, 3> trans41(y_length / 2.0 * y_axis);
	Eigen::Translation<float, 3> trans42(-z_length / 2.0 * z_axis);
	translations[4] = trans40 * trans41 * trans42;
	Eigen::Translation<float, 3> trans50(-x_length / 2.0 * x_axis);
	Eigen::Translation<float, 3> trans51(y_length / 2.0 * y_axis);
	Eigen::Translation<float, 3> trans52(-z_length / 2.0 * z_axis);
	translations[5] = trans50 * trans51 * trans52;
	Eigen::Translation<float, 3> trans60(-x_length / 2.0 * x_axis);
	Eigen::Translation<float, 3> trans61(-y_length / 2.0 * y_axis);
	Eigen::Translation<float, 3> trans62(-z_length / 2.0 * z_axis);
	translations[6] = trans60 * trans61 * trans62;
	Eigen::Translation<float, 3> trans70(x_length / 2.0 * x_axis);
	Eigen::Translation<float, 3> trans71(-y_length / 2.0 * y_axis);
	Eigen::Translation<float, 3> trans72(-z_length / 2.0 * z_axis);
	translations[7] = trans70 * trans71 * trans72;
	

	for (int i = 0; i < 8; i++)
		m_vertices[i] = translations[i] * m_centroid;

	Eigen::Vector3f face_normals[6] = {
		x_axis, y_axis, z_axis, -x_axis, -y_axis, -z_axis
	};

	m_faces.resize(12);
	m_faces_normals.resize(12);
	m_faces[0] = Vector3i(0, 1, 3);
	m_faces[1] = Vector3i(1, 2, 3);
	m_faces[2] = Vector3i(0, 3, 7);
	m_faces[3] = Vector3i(0, 7, 4);
	m_faces[4] = Vector3i(0, 4, 1);
	m_faces[5] = Vector3i(1, 4, 5);
	m_faces[6] = Vector3i(4, 7, 6);
	m_faces[7] = Vector3i(4, 6, 5);
	m_faces[8] = Vector3i(1, 5, 2);
	m_faces[9] = Vector3i(2, 5, 6);
	m_faces[10] = Vector3i(2, 7, 3);
	m_faces[11] = Vector3i(2, 6, 7);
	m_faces_normals[0] = face_normals[2];
	m_faces_normals[1] = face_normals[2];
	m_faces_normals[2] = face_normals[0];
	m_faces_normals[3] = face_normals[0];
	m_faces_normals[4] = face_normals[1];
	m_faces_normals[5] = face_normals[1];
	m_faces_normals[6] = face_normals[5];
	m_faces_normals[7] = face_normals[5];
	m_faces_normals[8] = face_normals[3];
	m_faces_normals[9] = face_normals[3];
	m_faces_normals[10] = face_normals[4];
	m_faces_normals[11] = face_normals[4];
}

QVector<Eigen::Vector3f> OBB::getVertices() const
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

	if (vertexCount() < 4)
		triangulate();

	if (vertexCount() > 0)
	{
		if (num_of_samples == 0)
			num_of_samples = 1000;

		std::vector<Utils_sampling::Vec3> verts(vertexCount());
		std::vector<Vec3> nors(vertexCount());
		std::vector<int> tris(3 * m_faces.size());

		int vert_idx = 0;
		for (QVector<Vector3f>::iterator vertex_it = m_vertices.begin(); vertex_it != m_vertices.end(); ++vertex_it)
		{
			Vec3 vert(vertex_it->x(), vertex_it->y(), vertex_it->z());
			verts[vert_idx] = vert;
			//tris[vert_idx] = vert_idx;
			nors[vert_idx++] = Vec3(0, 0, 0);
		}

		int face_idx = 0;
		for (QVector<Vector3i>::iterator face_it = m_faces.begin(); face_it != m_faces.end(); ++face_it)
		{
			tris[face_idx++] = face_it->x();
			tris[face_idx++] = face_it->y();
			tris[face_idx++] = face_it->z();
		}

		std::vector<Vec3> samples_pos;
		std::vector<Vec3> samples_nor;

		poisson_disk(0, num_of_samples, verts, nors, tris, samples_pos, samples_nor);

		m_sample_points.resize(samples_pos.size());

		int idx = 0;
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

void OBB::setSamplePoints(Eigen::MatrixXd samples_mat)
{
	if (!m_sample_points.empty())
		m_sample_points.clear();

	m_sample_points.resize(samples_mat.size());

	for (int i = 0; i < samples_mat.cols(); i++)
	{
		Eigen::Vector3d col_sample = samples_mat.col(i);
		Eigen::Vector3f sample((float)col_sample.x(), (float)col_sample.y(), (float)col_sample.z());
		m_sample_points[i] = sample;
	}
}

void OBB::setSamplePoints(pcl::PointCloud<pcl::PointXYZ> cloud)
{
	using namespace pcl;

	m_sample_points.clear();
	m_sample_points.resize(cloud.size());

	int sample_idx = 0;
	for (PointCloud<PointXYZ>::iterator point_it = cloud.begin(); point_it != cloud.end(); ++point_it)
	{
		Vector3f sample(point_it->x, point_it->y, point_it->z);
		m_sample_points[sample_idx++] = sample;
	}
}

QVector3D OBB::eigen_vector3f_to_qvector3d(Eigen::Vector3f vec)
{
	QVector3D qvec(vec.x(), vec.y(), vec.z());
	return qvec;
}

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

void OBB::draw(int scale)
{
	/* Draw the oriented box */
	if (m_vertices.size() < 3)
		triangulate();

	glColor4f(COLORS[m_label][0], COLORS[m_label][1], COLORS[m_label][2], 1.0);
	/*glBegin(GL_TRIANGLES);
	assert(m_faces.size() == m_faces_normals.size());
	QVector<Vector3i>::iterator face_it;
	QVector<Vector3f>::iterator normal_it;
	for (face_it = m_faces.begin(), normal_it = m_faces_normals.begin(); 
		face_it != m_faces.end() && normal_it != m_faces_normals.end(); ++face_it, ++normal_it)
	{
		glNormal3f(normal_it->x(), normal_it->y(), normal_it->z());
		Vector3f v0 = m_vertices[face_it->x()];
		Vector3f v1 = m_vertices[face_it->y()];
		Vector3f v2 = m_vertices[face_it->z()];

		glVertex3f(scale * v0.x(), scale * v0.y(), scale * v0.z());
		glVertex3f(scale * v1.x(), scale * v1.y(), scale * v1.z());
		glVertex3f(scale * v2.x(), scale * v2.y(), scale * v2.z());
	}
	glEnd();*/
	glLineWidth(2.0);
	glBegin(GL_LINES);
	drawLine(0, 1, scale);
	drawLine(1, 5, scale);
	drawLine(5, 4, scale);
	drawLine(4, 0, scale);
	drawLine(3, 2, scale);
	drawLine(2, 6, scale);
	drawLine(6, 7, scale);
	drawLine(7, 3, scale);
	drawLine(1, 2, scale);
	drawLine(0, 3, scale);
	drawLine(4, 7, scale);
	drawLine(5, 6, scale);
	glEnd();


	/* Draw the local system axes */
	/* draw x axis */
	glColor4f(COLORS[0][0], COLORS[0][1], COLORS[0][2], 1.0);
	glLineWidth(1.0);
	glBegin(GL_LINES);
	glVertex3f(scale * m_centroid.x(), scale * m_centroid.y(), scale * m_centroid.z());
	Vector3f x_end = m_centroid + x_axis;
	glVertex3f(scale * x_end.x(), scale * x_end.y(), scale * x_end.z());
	glEnd();
	/* draw y axis */
	glColor4f(COLORS[1][0], COLORS[1][1], COLORS[1][2], 1.0);
	glBegin(GL_LINES);
	glVertex3f(scale * m_centroid.x(), scale * m_centroid.y(), scale * m_centroid.z());
	Vector3f y_end = m_centroid + y_axis;
	glVertex3f(scale * y_end.x(), scale * y_end.y(), scale * y_end.z());
	glEnd();
	/* draw z axis */
	glColor4f(COLORS[2][0], COLORS[2][1], COLORS[2][2], 1.0);
	glBegin(GL_LINES);
	glVertex3f(scale * m_centroid.x(), scale * m_centroid.y(), scale * m_centroid.z());
	Vector3f z_end = m_centroid + z_axis;
	glVertex3f(scale * z_end.x(), scale * z_end.y(), scale * z_end.z());
	glEnd();
}

void OBB::drawSamples(int scale)
{
	glColor4f(COLORS[m_label][0], COLORS[m_label][1], COLORS[m_label][2], 1.0);
	glBegin(GL_POINTS);

	for (QVector<Vector3f>::iterator sample_it = m_sample_points.begin(); sample_it != m_sample_points.end(); ++sample_it)
		glVertex3f(scale * sample_it->x(), scale * sample_it->y(), scale * sample_it->z());

	glEnd();
}

int OBB::vertexCount() const
{
	return m_vertices.size();
}

int OBB::faceCount() const
{
	return m_faces.size();
}

QVector<Vector3i> OBB::getFaces() const
{
	return m_faces;
}

QVector<Vector3f> OBB::getFacesNormals() const
{
	return m_faces_normals;
}

QVector<Vector3f> OBB::getSamples() const
{
	return m_sample_points;
}

void OBB::translate(float x, float y, float z)
{
	Translation<float, 3> trans(x, y, z);
	m_centroid = trans * m_centroid;

	for (QVector<Vector3f>::iterator vertex_it = m_vertices.begin(); vertex_it != m_vertices.end(); ++vertex_it)
		*vertex_it = trans * (*vertex_it);

	for (QVector<Vector3f>::iterator sample_it = m_sample_points.begin(); sample_it != m_sample_points.end(); ++sample_it)
		*sample_it = trans * (*sample_it);
}

void OBB::rotate(float angle, float x, float y, float z)
{
	float angle_rad = angle / 180.0 * PI;
	AngleAxis<float> rotation(angle_rad, Vector3f(x, y, z));

	m_centroid = rotation * m_centroid;
	x_axis = rotation * x_axis;
	y_axis = rotation * y_axis;
	z_axis = rotation * z_axis;

	m_vertices.clear();
	m_faces.clear();
	m_faces_normals.clear();
	m_sample_points.clear();
	x_length = 1.0;
	y_length = 1.0;
	z_length = 1.0;
}

void OBB::rotate(float angle, float x, float y, float z, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
	using namespace pcl;
	rotate(angle, x, y, z);

	/* Recompute the x_length, y_length and z_length */
	float min[3] = { 100.0, 100.0, 100.0 };
	float max[3] = { -100.0, -100.0, -100.0 };
	Matrix3f local_axes = getAxes();
	Matrix3f transform_mat = getAxes().inverse();
	for (PointCloud<PointXYZ>::iterator point_it = cloud->begin(); point_it != cloud->end(); ++point_it)
	{
		Vector3f p(point_it->x, point_it->y, point_it->z);
		Vector3f point = transform_mat *p;

		if (point.x() < min[0])
			min[0] = point.x();
		if (point.x() > max[0])
			max[0] = point.x();
		if (point.y() < min[1])
			min[1] = point.y();
		if (point.y() > max[1])
			max[1] = point.y();
		if (point.z() < min[2])
			min[2] = point.z();
		if (point.z() > max[2])
			max[2] = point.z();
	}

	x_length = max[0] - min[0];
	y_length = max[1] - min[1];
	z_length = max[2] - min[2];

	m_centroid[0] = (max[0] + min[0]) / 2.0;
	m_centroid[1] = (max[1] + min[1]) / 2.0;
	m_centroid[2] = (max[2] + min[2]) / 2.0;

	m_centroid = local_axes * m_centroid;
	//samplePoints();
}

using namespace pcl;
void OBB::rotate(Matrix3d rotate_mat, Vector3d translate_vec, PointCloud<PointXYZ>::Ptr cloud)
{
	Matrix3f _rotate_mat = rotate_mat.cast<float>();
	Vector3f _translate_vec = translate_vec.cast<float>();

	m_centroid = _rotate_mat * m_centroid + _translate_vec;
	x_axis = _rotate_mat * x_axis;
	y_axis = _rotate_mat * y_axis;
	z_axis = _rotate_mat * z_axis;

	m_vertices.clear();
	m_faces.clear();
	m_faces_normals.clear();
	m_sample_points.clear();
	x_length = 1.0;
	y_length = 1.0;
	z_length = 1.0;

	/* Recompute the x_length, y_length and z_length */
	float min[3] = { 100.0, 100.0, 100.0 };
	float max[3] = { -100.0, -100.0, -100.0 };
	Matrix3f transform_mat = getAxes().inverse();
	for (PointCloud<PointXYZ>::iterator point_it = cloud->begin(); point_it != cloud->end(); ++point_it)
	{
		Vector3f p(point_it->x, point_it->y, point_it->z);
		//p -= m_centroid;
		Vector3f point = transform_mat *p;

		if (point.x() < min[0])
			min[0] = point.x();
		if (point.x() > max[0])
			max[0] = point.x();
		if (point.y() < min[1])
			min[1] = point.y();
		if (point.y() > max[1])
			max[1] = point.y();
		if (point.z() < min[2])
			min[2] = point.z();
		if (point.z() > max[2])
			max[2] = point.z();
	}

	x_length = max[0] - min[0];
	y_length = max[1] - min[1];
	z_length = max[2] - min[2];

	m_centroid[0] = (max[0] + min[0]) / 2.0;
	m_centroid[1] = (max[1] + min[1]) / 2.0;
	m_centroid[2] = (max[2] + min[2]) / 2.0;

	triangulate();
}

void OBB::transform(Matrix4f transform_mat, PointCloud<PointXYZ>::Ptr cloud)
{
	Vector4f centroid_aug;
	centroid_aug.block<3, 1>(0, 0) = m_centroid;
	centroid_aug(3, 0) = 1.0;
	Matrix<float, 4, 3> axes = getAugmentedAxes();

	centroid_aug = transform_mat * centroid_aug;
	for (int i = 0; i < 3; i++)
		axes.col(i) = transform_mat * axes.col(i);

	x_axis = axes.block<3, 1>(0, 0);
	y_axis = axes.block<3, 1>(0, 1);
	z_axis = axes.block<3, 1>(0, 2);
	m_centroid = centroid_aug.block<3, 1>(0, 0);

	m_vertices.clear();
	m_faces.clear();
	m_faces_normals.clear();
	m_sample_points.clear();
	x_length = 1.0;
	y_length = 1.0;
	z_length = 1.0;

	/* Recompute the x_length, y_length and z_length */
	float min[3] = { 100.0, 100.0, 100.0 };
	float max[3] = { -100.0, -100.0, -100.0 };
	Matrix3f local_axes_mat = getAxes();
	Matrix3f coord_trans = local_axes_mat.inverse();
	for (PointCloud<PointXYZ>::iterator point_it = cloud->begin(); point_it != cloud->end(); ++point_it)
	{
		Vector3f p(point_it->x, point_it->y, point_it->z);
		//p -= m_centroid;
		Vector3f point = coord_trans *p;

		if (point.x() < min[0])
			min[0] = point.x();
		if (point.x() > max[0])
			max[0] = point.x();
		if (point.y() < min[1])
			min[1] = point.y();
		if (point.y() > max[1])
			max[1] = point.y();
		if (point.z() < min[2])
			min[2] = point.z();
		if (point.z() > max[2])
			max[2] = point.z();
	}

	x_length = max[0] - min[0];
	y_length = max[1] - min[1];
	z_length = max[2] - min[2];

	Vector3f local_centroid((max[0] + min[0]) / 2.0, (max[1] + min[1]) / 2.0, (max[2] + min[2]) / 2.0);
	m_centroid = local_axes_mat * local_centroid;

	triangulate();
	/* transform the vertices of OBB */
	/*for (QVector<Vector3f>::iterator vertex_it = m_vertices.begin(); vertex_it != m_vertices.end(); ++vertex_it)
	{
		Vector4f vertex_aug(vertex_it->x(), vertex_it->y(), vertex_it->z(), 1.0);
		vertex_aug = transform_mat * vertex_aug;
		*vertex_it = vertex_aug.block<3, 1>(0, 0);
	}*/

	/* transform the normals of faces */
	/*for (QVector<Vector3f>::iterator face_norm_it = m_faces_normals.begin(); face_norm_it != m_faces_normals.end(); ++face_norm_it)
	{
		Vector4f norm_aug(face_norm_it->x(), face_norm_it->y(), face_norm_it->z(), 0);
		norm_aug = transform_mat * norm_aug;
		*face_norm_it = norm_aug.block<3, 1>(0, 0);
	}*/
}

int OBB::sampleCount() const
{
	return m_sample_points.size();
}

void OBB::drawLine(int vert_no_0, int vert_no_1, float scale)
{
	Vector3f vert0 = m_vertices[vert_no_0];
	Vector3f vert1 = m_vertices[vert_no_1];
	glVertex3f(scale * vert0.x(), scale * vert0.y(), scale * vert0.z());
	glVertex3f(scale * vert1.x(), scale * vert1.y(), scale * vert1.z());
}

QVector<Vector3f>::iterator OBB::samples_begin()
{
	return m_sample_points.begin();
}

QVector<Vector3f>::iterator OBB::samples_end()
{
	return m_sample_points.end();
}

void OBB::normalize(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
	/* Compute the rotation matrix from local y axis to global y axis */
	Matrix3f rotation_y = Utils::rotation_matrix(y_axis, Vector3f(0, 1.0, 0));

	/* Rotate three axes by rotation_y matrix */
	y_axis = rotation_y * y_axis;
	x_axis = rotation_y * x_axis;
	z_axis = rotation_y * z_axis;

	/* Rotate the local system around y_axis to align z_axis to global z-axis */
	Matrix3f rotation_z = Utils::rotation_matrix(z_axis, Vector3f(0, 0, 1.0));
	z_axis = rotation_z * z_axis;
	x_axis = rotation_z * x_axis;

	/* Rotate the local system back to make y_axis on the initial direction */
	Matrix3f rotation_y_inv = rotation_y.inverse();
	x_axis = rotation_y_inv * x_axis;
	y_axis = rotation_y_inv * y_axis;
	z_axis = rotation_y_inv * z_axis;

	m_vertices.clear();
	m_faces.clear();
	m_faces_normals.clear();
	m_sample_points.clear();
	x_length = 1.0;
	y_length = 1.0;
	z_length = 1.0;

	/* Recompute the x_length, y_length and z_length */
	float min[3] = { 100.0, 100.0, 100.0 };
	float max[3] = { -100.0, -100.0, -100.0 };
	Matrix3f local_axes_mat = getAxes();
	Matrix3f coord_trans = local_axes_mat.inverse();
	for (PointCloud<PointXYZ>::iterator point_it = cloud->begin(); point_it != cloud->end(); ++point_it)
	{
		Vector3f p(point_it->x, point_it->y, point_it->z);
		//p -= m_centroid;
		Vector3f point = coord_trans *p;

		if (point.x() < min[0])
			min[0] = point.x();
		if (point.x() > max[0])
			max[0] = point.x();
		if (point.y() < min[1])
			min[1] = point.y();
		if (point.y() > max[1])
			max[1] = point.y();
		if (point.z() < min[2])
			min[2] = point.z();
		if (point.z() > max[2])
			max[2] = point.z();
	}

	x_length = max[0] - min[0];
	y_length = max[1] - min[1];
	z_length = max[2] - min[2];

	Vector3f local_centroid((max[0] + min[0]) / 2.0, (max[1] + min[1]) / 2.0, (max[2] + min[2]) / 2.0);
	m_centroid = local_axes_mat * local_centroid;

	triangulate();
}