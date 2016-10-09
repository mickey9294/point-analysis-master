#include "papart.h"

using namespace Eigen;
PAPart::PAPart(OBB * obb) : m_cluster_no(0), num_of_samples(1000)
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

	m_obb = new OBB(obb);
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
	m_obb = new OBB(part.getOBB());
	m_vertices = part.getVertices();
	m_samples = part.getSamples();
	m_vertices_normals = part.getVerticesNormals();
	num_of_samples = part.num_of_samples;
	m_cuboid_surface_to_sample_correspondence = part.get_cuboid_surface_to_sample_correspondences();
	m_sample_to_cuboid_surface_correspondence = part.get_sample_to_cuboid_surface_correspondences();
}

PAPart::PAPart() : m_cluster_no(0), m_label(0), m_obb(NULL), num_of_samples(1000)
{
	m_rotate.setIdentity();
	m_translate.setZero();
	m_scale.setZero();
	m_height.setZero();
	m_axes.setIdentity();
}


PAPart::~PAPart()
{
	if (m_obb != NULL)
		delete(m_obb);
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

	/* Write the label and the cluster number */
	out << m_label << " " << m_cluster_no << endl;

	/* Write the rotation matrix */
	for (int i = 0; i < m_rotate.rows(); i++)
		out << m_rotate(i, 0) << " " << m_rotate(i, 1) << " " << m_rotate(i, 2) << endl;

	/* Write the translation vector */
	out << m_translate(0) << " " << m_translate(1) << " " << m_translate(2) << endl;

	/* Write the scale in 3 dimension */
	out << m_scale(0) << " " << m_scale(1) << " " << m_scale(2) << endl;

	/* Write the height */
	out << m_height(0) << " " << m_height(1) << " " << m_height(2) << " " << m_height(3) << endl;

	/* Write the axes */
	for (int i = 0; i < m_axes.rows(); i++)
		out << m_axes(i, 0) << " " << m_axes(i, 1) << " " << m_axes(i, 2) << endl;

	/* Write the vertices belong to this part */
	assert(m_vertices.size() == m_vertices_indices.size());
	assert(m_vertices.size() == m_vertices_normals.size());
	out << m_vertices.size() << endl;
	int vert_idx = 0;
	for (vector<int>::iterator it = m_vertices_indices.begin(); it != m_vertices_indices.end(); ++it)
	{
		/* Write the index of the vertex(point) */
		out << *it << " ";

		/* Write the coordinates of the vertex(point) */
		Eigen::Vector3f point = m_vertices[vert_idx];
		out << point.x() << " " << point.y() << " " << point.z() << " ";
		
		/* Write the coordinates of the normal */
		Eigen::Vector3f norm = m_vertices_normals[vert_idx];
		out << norm.x() << " " << norm.y() << " " << norm.z() << endl;
		vert_idx++;
	}

	/* Write the sample points */
	out << m_samples.size() << endl;  /* Write the number of the samples */
	for (std::vector<SamplePoint>::iterator sample_it = m_samples.begin(); sample_it != m_samples.end(); ++sample_it)
		out << sample_it->toString() << endl;

	/* Write the sample-to-cuboid-surface-correspondence */
	out << m_sample_to_cuboid_surface_correspondence.size() << endl;  /* Write the number of all correspondences */
	for (int i = 0; i < m_sample_to_cuboid_surface_correspondence.size(); i++)
		out << m_sample_to_cuboid_surface_correspondence[i] << endl;

	/* Write the cuboid-surface-to-sample-correspondences */
	out << m_cuboid_surface_to_sample_correspondence.size() << endl;
	for (int i = 0; i < m_cuboid_surface_to_sample_correspondence.size(); i++)
		out << m_cuboid_surface_to_sample_correspondence[i] << endl;

	/* Write the OBB */
	if (m_obb == NULL)
	{
		/* Write the OBB absence flag 0 */
		out << "0" << endl;
	}
	else
	{
		/* Write the OBB existence flat 1 */
		out << "1" << endl;

		/* Write the information of the OBB */
		m_obb->writeToFile(out);
	}

	out.close();
}

PAPart::PAPart(string path)
{
	ifstream in(path.c_str());

	if (in.is_open())
	{
		char buffer[256];

		/* Read label and cluster no */
		in.getline(buffer, 128);
		QString label_str(buffer);
		m_label = label_str.section(' ', 0, 0).toInt();
		m_cluster_no = label_str.section(' ', 1, 1).toInt();

		/* Read the rotation matrix */
		for (int i = 0; i < 3; i++)
		{
			in.getline(buffer, 128);
			QString rot_str(buffer);
			m_rotate(i, 0) = rot_str.section(' ', 0, 0).toFloat();
			m_rotate(i, 1) = rot_str.section(' ', 1, 1).toFloat();
			m_rotate(i, 2) = rot_str.section(' ', 2, 2).toFloat();
		}

		/* Read the translate vector */
		in.getline(buffer, 128);
		QString trans_str(buffer);
		m_translate(0) = trans_str.section(' ', 0, 0).toFloat();
		m_translate(1) = trans_str.section(' ', 1, 1).toFloat();
		m_translate(2) = trans_str.section(' ', 2, 2).toFloat();

		/* Read the scale vector */
		in.getline(buffer, 128);
		QString scale_str(buffer);
		m_scale(0) = scale_str.section(' ', 0, 0).toFloat();
		m_scale(1) = scale_str.section(' ', 1, 1).toFloat();
		m_scale(2) = scale_str.section(' ', 2, 2).toFloat();

		/* Read the height vector */
		in.getline(buffer, 128);
		QStringList h_str = QString(buffer).split(' ');
		for (int i = 0; i < h_str.size(); i++)
			m_height(i) = h_str.at(i).toFloat();

		/* Read the 3 axes */
		for (int i = 0; i < 3; i++)
		{
			in.getline(buffer, 128);
			QString axis_str(buffer);
			m_axes(i, 0) = axis_str.section(' ', 0, 0).toFloat();
			m_axes(i, 1) = axis_str.section(' ', 1, 1).toFloat();
			m_axes(i, 2) = axis_str.section(' ', 2, 2).toFloat();
		}

		/* Read the vertices indices */
		in.getline(buffer, 128);
		int num_vertices = std::atoi(buffer);
		m_vertices_indices.resize(num_vertices);
		m_vertices.resize(num_vertices);
		m_vertices_normals.resize(num_vertices);
		for (int i = 0; i < num_vertices; i++)
		{
			in.getline(buffer, 128);
			QStringList line_list = QString(buffer).split(' ');

			m_vertices_indices[i] = line_list[0].toInt();

			Eigen::Vector3f vertex;
			Eigen::Vector3f normal;
			for (int j = 0; j < 3; j++)
			{
				vertex[j] = line_list[1 + j].toFloat();
				normal[j] = line_list[4 + j].toFloat();
			}

			m_vertices[i] = vertex;
			m_vertices_normals[i] = normal;
		}

		/* Read the sample points */
		in.getline(buffer, 16);
		int num_samples = std::atoi(buffer);
		m_samples.resize(num_samples);
		for (int i = 0; i < num_samples; i++)
		{
			in.getline(buffer, 256);
			std::string sample_str(buffer);
			m_samples[i] = SamplePoint(sample_str);
		}

		/* Read the sample-to-cuboid-surface-correspondences */
		in.getline(buffer, 16);
		int num_sample_to_surface_corres = std::atoi(buffer);
		m_sample_to_cuboid_surface_correspondence.resize(num_sample_to_surface_corres);
		for (int i = 0; i < num_sample_to_surface_corres; i++)
		{
			in.getline(buffer, 16);
			m_sample_to_cuboid_surface_correspondence[i] = std::atoi(buffer);
		}

		/* Read the cuboid-surface-to-sample-correspondences */
		in.getline(buffer, 16);
		int num_surface_to_sample_corres = std::atoi(buffer);
		m_cuboid_surface_to_sample_correspondence.resize(num_surface_to_sample_corres);
		for (int i = 0; i < num_surface_to_sample_corres; i++)
		{
			in.getline(buffer, 16);
			m_cuboid_surface_to_sample_correspondence[i] = std::atoi(buffer);
		}

		/* Read the OBB */
		in.getline(buffer, 4);
		int flag = std::atoi(buffer);
		if (flag == 0)
			m_obb = NULL;
		else
			m_obb = new OBB(in);

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

	if (m_obb != NULL)
		delete(m_obb);
	m_obb = obb;

	if (m_samples.size() < param_min_num_cuboid_sample_points)
		samplePoints();
	else
		create_grid_points_on_obb_surface();

	return obb;
}

OBB * PAPart::getOBB() const
{
	return m_obb;
}

void PAPart::setOBB(OBB * obb)
{
	m_obb = obb;
	updateFromOBB();
}

vector<int>::iterator PAPart::vertices_begin()
{
	return m_vertices_indices.begin();
}

vector<int>::iterator PAPart::vertices_end()
{
	return m_vertices_indices.end();
}

void PAPart::setLabel(int label)
{
	m_label = label;

	if (m_obb != NULL)
		m_obb->setLabel(label);
}

std::vector<Vector3f> PAPart::getVertices() const
{
	return m_vertices;
}

std::vector<Vector3f> & PAPart::getAllVertices()
{
	return m_vertices;
}

std::vector<Vector3f> PAPart::getVerticesNormals() const
{
	return m_vertices_normals;
}

std::vector<SamplePoint> PAPart::getSamples() const
{
	return m_samples;
}

void PAPart::getSamplePoints(std::vector<Eigen::Vector3f> & sample_points) const
{
	sample_points.clear();
	sample_points.reserve(m_samples.size());

	for (std::vector<SamplePoint>::const_iterator sample_it = m_samples.begin(); sample_it != m_samples.end(); ++sample_it)
	{
		Vector3f point = sample_it->getVertex();
		sample_points.push_back(point);
	}
}

SamplePoint PAPart::getSample(int index)
{
	return m_samples[index];
}

int PAPart::numOfSamples() const
{
	return m_samples.size();
}

void PAPart::setVertices(QVector<Vector3f> vertices)
{
	m_vertices.clear();
	m_vertices = vertices.toStdVector();
}

void PAPart::setVertices(std::vector<Vector3f> vertices)
{
	m_vertices.clear();
	m_vertices = vertices;
}

void PAPart::setVerticesNormals(QVector<Vector3f> vertices_normals)
{
	m_vertices_normals.clear();
	m_vertices_normals = vertices_normals.toStdVector();
}

void PAPart::setVerticesNormals(std::vector<Vector3f> vertices_normals)
{
	m_vertices_normals.clear();
	m_vertices_normals = vertices_normals;
}

void PAPart::samplePoints()
{
	num_of_samples = NUM_OF_SAMPLES;

	m_samples.clear();

	if (m_vertices.size() > NUM_OF_SAMPLES)  /* If the points in the part is greater than 1000, than do downsampling */
	{
		//std::cout << "downsample." << std::endl;
		downsample();
	}
	else   /* If the points in the part is less than 1000, then regard the current vertices as sample points */
	{
		//cout << "upsample." << endl;
		//upsample();
		num_of_samples = m_vertices.size();
		m_samples.resize(num_of_samples);

		for (int vert_index = 0; vert_index < num_of_samples; ++vert_index)
		{
			Eigen::Vector3f vert = m_vertices[vert_index];
			Eigen::Vector3f norm = m_vertices_normals[vert_index];
			SamplePoint sample(vert, norm);
			m_samples[vert_index] = sample;
		}
	}

	num_of_samples = m_samples.size();

	if (m_obb != NULL)
	{
		create_grid_points_on_obb_surface();
	}
}

void PAPart::update_sample_correspondences()
{
	m_sample_to_cuboid_surface_correspondence.clear();
	m_cuboid_surface_to_sample_correspondence.clear();

	m_sample_to_cuboid_surface_correspondence.resize(m_samples.size(), -1);
	m_cuboid_surface_to_sample_correspondence.resize(m_obb->sampleCount(), -1);

	/* X: sample point of the model; Y: sample points on the oriented bounding box */
	int num_X_points = m_samples.size();
	int num_Y_points = m_obb->sampleCount();

	if (num_X_points == 0 || num_Y_points == 0)
		return;

	Eigen::MatrixXd X_points(3, num_X_points);
	Eigen::MatrixXd Y_points(3, num_Y_points);

	// FIXME:
	// The type of indices should integer.
	// But, it causes compile errors in the 'ICP::get_closest_points' function.
	Eigen::MatrixXd X_indices(1, num_X_points);
	Eigen::MatrixXd Y_indices(1, num_Y_points);

	for (unsigned int X_point_index = 0; X_point_index < num_X_points; ++X_point_index)
	{
		for (unsigned int i = 0; i < 3; ++i)
			X_points.col(X_point_index)(i) = getSample(X_point_index)[i];
		X_indices.col(X_point_index)(0) = X_point_index;
	}

	for (unsigned int Y_point_index = 0; Y_point_index < num_Y_points; ++Y_point_index)
	{
		for (unsigned int i = 0; i < 3; ++i)
			Y_points.col(Y_point_index)(i) = m_obb->getSample(Y_point_index)[i];
		Y_indices.col(Y_point_index)(0) = Y_point_index;
	}

	ANNpointArray X_ann_points = NULL;
	ANNkd_tree* X_ann_kd_tree = ICP::create_kd_tree(X_points, X_ann_points);
	assert(X_ann_kd_tree);

	ANNpointArray Y_ann_points = NULL;
	ANNkd_tree* Y_ann_kd_tree = ICP::create_kd_tree(Y_points, Y_ann_points);
	assert(Y_ann_kd_tree);

	// X -> Y.
	Eigen::MatrixXd closest_Y_indices;
	ICP::get_closest_points(Y_ann_kd_tree, X_points, Y_indices, closest_Y_indices);
	assert(closest_Y_indices.cols() == num_X_points);

	// Y -> X.
	Eigen::MatrixXd closest_X_indices;
	ICP::get_closest_points(X_ann_kd_tree, Y_points, X_indices, closest_X_indices);
	assert(closest_X_indices.cols() == num_Y_points);

	// NOTE:
	// X: sample points, Y: cuboid surface_points.
	for (unsigned int X_point_index = 0; X_point_index < num_X_points; ++X_point_index)
	{
		assert(closest_Y_indices.col(X_point_index)(0) < num_Y_points);
		m_sample_to_cuboid_surface_correspondence[X_point_index] =
			static_cast<int>(closest_Y_indices.col(X_point_index)(0));
	}

	for (unsigned int Y_point_index = 0; Y_point_index < num_Y_points; ++Y_point_index)
	{
		assert(closest_X_indices.col(Y_point_index)(0) < num_X_points);
		m_cuboid_surface_to_sample_correspondence[Y_point_index] =
			static_cast<int>(closest_X_indices.col(Y_point_index)(0));
	}

	if (X_ann_points) annDeallocPts(X_ann_points);
	if (Y_ann_points) annDeallocPts(Y_ann_points);
	delete X_ann_kd_tree;
	delete Y_ann_kd_tree;
}

const std::vector<int> & PAPart::get_sample_to_cuboid_surface_correspondences() const
{
	return m_sample_to_cuboid_surface_correspondence;
}

int PAPart::get_sample_to_cuboid_surface_correspondences(int point_index)
{
	if (m_obb->sampleCount() == 0)
		return -1;

	assert(point_index < m_samples.size());
	return m_sample_to_cuboid_surface_correspondence[point_index];
}

const std::vector<int> & PAPart::get_cuboid_surface_to_sample_correspondences() const
{
	return m_cuboid_surface_to_sample_correspondence;
}

int PAPart::get_cuboid_surface_to_sample_correspondences(const int point_index)
{
	assert(point_index < m_obb->sampleCount());
	return m_cuboid_surface_to_sample_correspondence[point_index];
}

void PAPart::create_random_points_on_obb_surface()
{
	assert(m_obb != NULL);
	assert(m_samples.size() > 0);

	m_obb->setNumOfSamples(m_samples.size());

	m_obb->createRandomSamples();
	update_sample_correspondences();
}

void PAPart::create_grid_points_on_obb_surface()
{
	assert(m_obb != NULL);
	assert(m_samples.size() > 0);
	
	m_obb->setNumOfSamples(m_samples.size());

	m_obb->createGridSamples();
	
	update_sample_correspondences();
}

void PAPart::clearVertices()
{
	m_vertices_indices.clear();
	m_vertices.clear();
	m_vertices_normals.clear();

	clearSamples();
}

void PAPart::clearSamples()
{
	m_samples.clear();
	num_of_samples = NUM_OF_SAMPLES;
	m_sample_to_cuboid_surface_correspondence.clear();
	m_cuboid_surface_to_sample_correspondence.clear();

	if (m_obb != NULL)
		m_cuboid_surface_to_sample_correspondence.resize(m_obb->sampleCount(), -1);
}

void PAPart::addVertex(int index, Vector3f &vertex, Vector3f &vertex_normal)
{
	m_vertices_indices.push_back(index);
	m_vertices.push_back(vertex);
	m_vertices_normals.push_back(vertex_normal);
}

std::vector<SamplePoint>::iterator PAPart::samples_begin()
{
	return m_samples.begin();
}

std::vector<SamplePoint>::iterator PAPart::samples_end()
{
	return m_samples.end();
}

void PAPart::setTranslation(Vector3f centroid)
{
	m_translate = centroid;

	if (m_obb != NULL)
		m_obb->setCentroid(centroid);
}

void PAPart::updateFromOBB()
{
	assert(m_obb);

	//m_label = m_obb->getLabel();
	Eigen::Vector3f centroid = m_obb->getCentroid();
	m_translate = Vector3f(centroid.x(), centroid.y(), centroid.z());
	QVector3D scale = m_obb->getScale();
	m_scale = Vector3f(scale.x(), scale.y(), scale.z());
	Vector3f x_axis = m_obb->getXAxis();
	Vector3f y_axis = m_obb->getYAxis();
	Vector3f z_axis = m_obb->getZAxis();

	Matrix3f base_axes = Matrix3f::Identity();
	m_rotate.col(0) = x_axis;
	m_rotate.col(1) = y_axis;
	m_rotate.col(2) = z_axis;

	const Vector3f gravity(0.0, -1.0, 0.0);
	Matrix<float, 3, 4> Rt;
	Rt.block<3, 3>(0, 0) = m_rotate;
	Rt.block<3, 1>(0, 3) = m_translate;
	Matrix4f Sb;
	Sb << m_scale[0], 0.0f, 0.0f, 0.0f,
		0.0f, m_scale[1], 0.0f, 0.0f,
		0.0f, 0.0f, m_scale[2], 0.0f,
		0.0f, 0.0f, 0.0f, 1.0f;

	m_height = (gravity.transpose() * Rt * Sb).transpose();

	m_axes.col(0) = x_axis;
	m_axes.col(1) = y_axis;
	m_axes.col(2) = z_axis;
}

void PAPart::updateOBB()
{
	assert(m_obb);
	m_obb->updateWithNewPoints(m_vertices);

	updateFromOBB();
}

void PAPart::update_axes_center_size_corner_points()
{
	const int num_face_corners = OBB::k_num_face_corners;
	Eigen::Matrix3d axes;

	for (int axis_index = 0; axis_index < 3; ++axis_index)
	{
		Eigen::MatrixXd A(num_face_corners, 3);
		double sum_length = 0;

		for (int face_corner_index = 0; face_corner_index < OBB::k_num_face_corners; ++face_corner_index)
		{
			std::bitset<3> bits;
			bits[(axis_index + 1) % 3] = ((face_corner_index / 2) == 0);
			bits[(axis_index + 2) % 3] = ((face_corner_index % 2) == 0);

			bits[axis_index] = true;
			unsigned int pos_corner_index = static_cast<unsigned int>(bits.to_ulong());
			Eigen::Vector3d pos_corner_point = m_obb->getVertex(pos_corner_index).cast<double>();

			bits[axis_index] = false;
			unsigned int neg_corner_index = static_cast<unsigned int>(bits.to_ulong());
			Eigen::Vector3d neg_corner_point = m_obb->getVertex(neg_corner_index).cast<double>();

			Eigen::Vector3d direction = pos_corner_point - neg_corner_point;
			sum_length += direction.norm();
			direction.normalize();

			for (int i = 0; i < 3; i++)
				A.row(face_corner_index)[i] = direction[i];
		}

		if (sum_length == 0)
			sum_length += MIN_CUBOID_SIZE;

		// The first column of matrix V in SVD is the vector maximizing
		// cos(angle) with all columns in the given matrix A.
		Eigen::JacobiSVD<Eigen::MatrixXd> svd(A, Eigen::ComputeFullU | Eigen::ComputeFullV);
		axes.col(axis_index) = svd.matrixV().col(0);

		// Weight each axis based on the length of cuboid edges.
		axes.col(axis_index) = sum_length * axes.col(axis_index);

		// FIXME:
		// Hack. Check whether each axis is flipped...
		Eigen::Vector3d axis;
		Eigen::Matrix3f obb_axes = m_obb->getAxes();
		for (unsigned int i = 0; i < 3; ++i)
			axis[i] = obb_axes.col(axis_index)[i];

		if (axes.col(axis_index).dot(axis) < 0)
			axes.col(axis_index) = -axes.col(axis_index);
	}

	/* Find the nearest orthogonal matrix */
	Eigen::JacobiSVD<Eigen::Matrix3d> svd(axes, Eigen::ComputeFullU | Eigen::ComputeFullV);
	Eigen::Matrix3d new_axes = svd.matrixU() * svd.matrixV().transpose();
	new_axes.colwise().normalize();

	m_obb->setAxes(new_axes);

	//update_center_size_corner_points();
	m_obb->updateWithNewPoints(m_vertices);

	updateFromOBB();
}

void PAPart::update_center_size_corner_points()
{
	assert(m_obb);
	Matrix3f obb_axes = m_obb->getAxes();

	/* Find center and size minimizing the error */
	Eigen::MatrixXd A = Eigen::MatrixXd::Zero(3 * OBB::k_num_corners, 6);
	Eigen::VectorXd b = Eigen::VectorXd::Zero(3 * OBB::k_num_corners);

	for (int corner_index = 0; corner_index < OBB::k_num_corners; ++corner_index)
	{
		std::bitset<3> bits(corner_index);

		A.block<3, 3>(3 * corner_index, 0).setIdentity();

		for (int axis_index = 0; axis_index < 3; ++axis_index)
		{
			for (int i = 0; i < 3; i++)
			{
				if (!bits[axis_index])
					A(3 * corner_index + i, 3 + axis_index) = -0.5 * obb_axes.col(axis_index)[i];
				else
					A(3 * corner_index + i, 3 + axis_index) = +0.5 * obb_axes.col(axis_index)[i];

				b(3 * corner_index + i) = m_obb->getVertex(corner_index)[i];
			}
		}
	}

	/* Least square */
	Eigen::VectorXd x = A.colPivHouseholderQr().solve(b);
	assert(x.rows() == 6);
	Eigen::Vector3d centroid, sscale;
	for (unsigned int axis_index = 0; axis_index < 3; ++axis_index)
	{
		centroid[axis_index] = x(axis_index);
		sscale[axis_index] = x(3 + axis_index);
	}
	m_obb->setCentroid(centroid);
	m_obb->setScale(sscale);
	cout << "update_center_size_corner_points(): scale = " << sscale.transpose() << endl;

	m_obb->updateCorners();

	updateFromOBB();
}

void PAPart::ICP_adjust_OBB()
{
	assert(m_obb);
	assert(m_vertices.size() > 0);

	if (m_samples.size() < param_min_num_cuboid_sample_points)
	{
		//cout << "The part has " << m_samples.size() << "samplep points, resample points on the part." << endl;
		samplePoints();
		//cout << "done." << endl;
	}

	int surface_sample_size = m_obb->sampleCount();
	int sample_size = m_samples.size();

	assert(sample_size == surface_sample_size);

	
	MatrixXd sample_points(3, sample_size);
	MatrixXd surface_sample_points(3, surface_sample_size);

	/* Create data matrix for surface sample points on OBB */
	//cout << "Create data matrix for surface samples on OBB" << endl;
	int idx = 0;
	for (QVector<SamplePoint>::iterator surface_sample_it = m_obb->samples_begin();
		surface_sample_it != m_obb->samples_end(); ++surface_sample_it)
	{
		Vector3d point = surface_sample_it->getVertex().cast<double>();
		surface_sample_points.col(idx++) = point;
	}
	//cout << "done." << endl;

	/* Create data matrix for the points sampled on the parts */
	//cout << "Create data matrix for surface samples on parts" << endl;
	idx = 0;
	for (std::vector<SamplePoint>::iterator sample_it = m_samples.begin(); sample_it != m_samples.end(); sample_it++)
	{
		Vector3d point = sample_it->getVertex().cast<double>();
		sample_points.col(idx++) = point;
	}
	//cout << "done." << endl;

	Matrix3d rotation_mat;
	Vector3d translation_vec;

	/* Ececute ICP procedure */
	//cout << "Run ICP procedure" << endl;
	double error = ICP::run_iterative_closest_points(surface_sample_points, sample_points, rotation_mat, translation_vec);

	//cout << "Transform OBB with result of ICP." << endl;
	m_obb->transform(rotation_mat, translation_vec, m_vertices);
	updateFromOBB();
	//cout << "Create grid points on OBB surface." << endl;
	create_grid_points_on_obb_surface();
	//cout << "done." << endl;
}

int PAPart::num_cuboid_surface_points()
{
	assert(m_obb);
	return m_obb->sampleCount();
}

void PAPart::upsample()
{
	std::vector<SC_PointVectorPair> points;

	for (int i = 0; i < m_vertices.size(); i++)
	{
		Eigen::Vector3f p = m_vertices[i];
		Eigen::Vector3f n = m_vertices_normals[i];

		SC_Point point(p.x(), p.y(), p.z());
		SC_Vector norm(n.x(), n.y(), n.z());
		SC_PointVectorPair pair(point, norm);
		points.push_back(pair);
	}

	//Algorithm parameters
	const double sharpness_angle = 25;   // control sharpness of the result.
	const double edge_sensitivity = 0;    // higher values will sample more points near the edges          
	const double neighbor_radius = 0;  // initial size of neighborhood.
	const std::size_t number_of_output_points = 1000;
	//Run algorithm 
	cout << "Do edge aware upsample." << endl;
	CGAL::edge_aware_upsample_point_set<Concurrency_tag>(
		points.begin(),
		points.end(),
		std::back_inserter(points),
		CGAL::First_of_pair_property_map<SC_PointVectorPair>(),
		CGAL::Second_of_pair_property_map<SC_PointVectorPair>(),
		sharpness_angle,
		edge_sensitivity,
		neighbor_radius,
		number_of_output_points);
	cout << "done." << endl;

	m_samples.clear();
	m_samples.resize(points.size());

	int idx = 0;
	for (std::vector<SC_PointVectorPair>::iterator it = points.begin(); it != points.end() && idx < 1000; ++it)
	{
		SC_Point p = it->first;
		SC_Vector n = it->second;

		SamplePoint sample(p.x(), p.y(), p.z(), n.x(), n.y(), n.z());
		m_samples[idx++] = sample;
	}

	num_of_samples = m_samples.size();
}

void PAPart::downsample()
{
	std::vector<SC_Point> points;

	for (int i = 0; i < m_vertices.size(); i++)
	{
		Eigen::Vector3f p = m_vertices[i];
		Eigen::Vector3f n = m_vertices_normals[i];

		SC_Point point(p.x(), p.y(), p.z());
		points.push_back(point);
	}

	std::vector<SC_Point> output;

	/* Parameters */
	const double retain_percentage = (double)num_of_samples / m_vertices.size() * 100.0;
	const double neighbor_radius = -0.5;

	CGAL::wlop_simplify_and_regularize_point_set
		<Concurrency_tag>
		(points.begin(),
		points.end(),
		std::back_inserter(output),
		retain_percentage,
		neighbor_radius
		);

	m_samples.clear();
	m_samples.resize(output.size());

	int idx = 0;
	for (std::vector<SC_Point>::iterator it = output.begin(); it != output.end() && idx < num_of_samples; ++it)
	{
		SamplePoint sample(it->x(), it->y(), it->z());
		m_samples[idx++] = sample;
	}

	num_of_samples = m_samples.size();
}

void PAPart::draw(float scale)
{
	glColor4f(COLORS[m_label][0], COLORS[m_label][1], COLORS[m_label][2], 1.0);
	glBegin(GL_POINTS);

	for (std::vector<SamplePoint>::iterator sample_it = m_samples.begin(); sample_it != m_samples.end(); ++sample_it)
		glVertex3f(scale * sample_it->x(), scale * sample_it->y(), scale * sample_it->z());
	glEnd();

	if (m_obb != NULL)
	{
		m_obb->draw(scale);
		m_obb->drawSamples(scale);
	}

	/* Draw correspondences from part samples to obb samples */
	//glLineWidth(0.7);
	//glColor4f(0.3, 0.3, 0.3, 0.8);
	//glBegin(GL_LINES);
	//for (int i = 0; i < m_samples.size(); i++)
	//{
	//	int obb_sample_index = get_sample_to_cuboid_surface_correspondences(i);
	//	glVertex3f(scale * m_samples[i].x(), scale * m_samples[i].y(), scale * m_samples[i].z());
	//	glVertex3f(scale * m_obb->getSample(obb_sample_index).x(), scale * m_obb->getSample(obb_sample_index).y(), scale * m_obb->getSample(obb_sample_index).z());
	//}
	//glEnd();

	///* Draw correspondences from obb sample to part samples */
	//glBegin(GL_LINES);
	//for (int i = 0; i < m_obb->sampleCount(); i++)
	//{
	//	int part_sample_index = get_cuboid_surface_to_sample_correspondences(i);
	//	glVertex3f(scale * m_obb->getSample(i).x(), scale * m_obb->getSample(i).y(), scale * m_obb->getSample(i).z());
	//	glVertex3f(scale * m_samples[part_sample_index].x(), scale * m_samples[part_sample_index].y(), scale * m_samples[part_sample_index].z());
	//}
	//glEnd();
}

void PAPart::outputSamples()
{
	assert(m_obb);

	int num_part_samples = m_samples.size();
	int num_obb_samples = m_obb->sampleCount();

	std::string out_path = "../data/parts_samples/" + std::to_string(m_label) + ".csv";
	std::ofstream out(out_path.c_str());

	for (int i = 0; i < std::max(num_part_samples, num_obb_samples); i++)
	{
		if (i < std::min(num_part_samples, num_obb_samples))
			out << m_samples[i].x() << "," << m_samples[i].y() << "," << m_samples[i].z() << ",,"
			<< m_obb->getSample(i).x() << "," << m_obb->getSample(i).y() << "," << m_obb->getSample(i).z() << endl;
		else if (i >= num_part_samples && i < num_obb_samples)
			out << ",,,," << m_obb->getSample(i).x() << "," << m_obb->getSample(i).y() << "," << m_obb->getSample(i).z() << endl;
		else if (i < num_part_samples && i >= num_obb_samples)
			out << m_samples[i].x() << "," << m_samples[i].y() << "," << m_samples[i].z() << endl;
	}
	
	out.close();
}