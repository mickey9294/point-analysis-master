#include "obb.h"

using namespace Eigen;

OBB::OBB(QObject *parent)
	: QObject(parent), m_draw_axes(true)
{
	x_axis = Vector3f(1.0, 0, 0);
	y_axis = Vector3f(0, 1.0, 0);
	z_axis = Vector3f(0, 0, 1.0);
	x_length = 1.0;
	y_length = 1.0;
	z_length = 1.0;
	m_centroid.setZero();
	m_num_of_samples = NUM_OF_SAMPLES;
}

OBB::OBB(Eigen::Vector3f xAxis, Eigen::Vector3f yAxis, Eigen::Vector3f zAxis, Eigen::Vector3f centroid,
	double xLength, double yLength, double zLength, int label, QObject *parent)
	: QObject(parent), m_draw_axes(true)
{
	x_axis = xAxis;
	y_axis = yAxis;
	z_axis = zAxis;
	x_axis.normalize();
	y_axis.normalize();
	z_axis.normalize();

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

	m_num_of_samples = NUM_OF_SAMPLES;

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
	m_samples = obb.getSamples();
	m_num_of_samples = obb.getNumOfSamples();
	m_draw_axes = obb.isDrawAxes();
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
	m_samples = obb->getSamples();
	m_num_of_samples = obb->getNumOfSamples();
	m_draw_axes = obb->isDrawAxes();
}

OBB::OBB(std::ifstream & in)
	: m_draw_axes(true)
{
	char buffer[256];
	/* Laod the axes */
	in.getline(buffer, 128);
	QString line(buffer);
	x_axis[0] = line.section(' ', 0, 0).toFloat();
	x_axis[1] = line.section(' ', 1, 1).toFloat();
	x_axis[2] = line.section(' ', 2, 2).toFloat();
	in.getline(buffer, 128);
	line = QString(buffer);
	y_axis[0] = line.section(' ', 0, 0).toFloat();
	y_axis[1] = line.section(' ', 1, 1).toFloat();
	y_axis[2] = line.section(' ', 2, 2).toFloat();
	in.getline(buffer, 128);
	line = QString(buffer);
	z_axis[0] = line.section(' ', 0, 0).toFloat();
	z_axis[1] = line.section(' ', 1, 1).toFloat();
	z_axis[2] = line.section(' ', 2, 2).toFloat();

	/* Load the lengths in 3 dimensions */
	in.getline(buffer, 128);
	line = QString(buffer);
	x_length = line.section(' ', 0, 0).toFloat();
	y_length = line.section(' ', 1, 1).toFloat();
	z_length = line.section(' ', 2, 2).toFloat();

	/* Load the centroid */
	in.getline(buffer, 128);
	line = QString(buffer);
	m_centroid[0] = line.section(' ', 0, 0).toFloat();
	m_centroid[1] = line.section(' ', 1, 1).toFloat();
	m_centroid[2] = line.section(' ', 2, 2).toFloat();

	/* Load the color */
	in.getline(buffer, 128);
	line = QString(buffer);
	m_color[0] = line.section(' ', 0, 0).toFloat();
	m_color[1] = line.section(' ', 1, 1).toFloat();
	m_color[2] = line.section(' ', 2, 2).toFloat();

	/* Load the label */
	in.getline(buffer, 128);
	m_label = std::atoi(buffer);

	/* Load 8 corner vertices */
	m_vertices.resize(k_num_corners);
	for (int i = 0; i < k_num_corners; i++)
	{
		in.getline(buffer, 128);
		line = QString(buffer);
		QStringList line_list = line.split(' ');
		Eigen::Vector3f corner(line_list[0].toFloat(), line_list[1].toFloat(), line_list[2].toFloat());
		m_vertices[i] = corner;
	}

	/* Load the faces and faces normals */
	int num_triangle_faces = 2 * k_num_faces;
	m_faces.resize(num_triangle_faces);
	m_faces_normals.resize(num_triangle_faces);
	for (int i = 0; i < num_triangle_faces; i++)
	{
		in.getline(buffer, 128);
		QStringList line_list = QString(buffer).split(' ');
		Eigen::Vector3i face;
		Eigen::Vector3f face_normal;
		for (int j = 0; j < 3; j++)
		{
			face[j] = line_list[j].toInt();
			face_normal[j] = line_list[3 + j].toFloat();
		}
		m_faces[i] = face;
		m_faces_normals[i] = face_normal;
	}

	/* Load the sample points */
	in.getline(buffer, 16);
	int num_samples = std::atoi(buffer);
	m_samples.resize(num_samples);
	for (int i = 0; i < num_samples; i++)
	{
		in.getline(buffer, 256);
		std::string sample_str(buffer);
		SamplePoint sample(sample_str);
		m_samples[i] = sample;
	}
}

OBB::~OBB()
{

}

const int OBB::k_face_corner_indices[k_num_faces][k_num_face_corners] = {
	{ 7, 4, 0, 3 }, // POSITIVE_X_AXIS.
	{ 6, 2, 1, 5 }, // NEGATIVE_X_AXIS.
	{ 5, 1, 0, 4 }, // POSITIVE_Y_AXIS.
	{ 6, 7, 3, 2 }, // NEGATIVE_Y_AXIS.
	{ 2, 3, 0, 1 }, // POSITIVE_Z_AXIS.
	{ 6, 5, 4, 7 }  // NEGATIVE_Z_AXIS.
};

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
	updateCorners();

	Eigen::Vector3f face_normals[6] = {
		x_axis, y_axis, z_axis, -x_axis, -y_axis, -z_axis
	};

	m_faces.resize(12);

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
	
	computeFaceNormals();
}

QVector<Eigen::Vector3f> OBB::getVertices() const
{
	return m_vertices;
}

Eigen::Vector3f OBB::getVertex(int index) const
{
	return m_vertices[index];
}

void OBB::setColor(QVector3D color)
{
	m_color = color;
}

int OBB::samplePoints(int num_of_samples)
{
	using namespace Utils_sampling;

	if (!m_samples.empty())
		m_samples.clear();

	if (vertexCount() < 4)
		triangulate();

	if (vertexCount() > 0)
	{
		if (num_of_samples != m_num_of_samples)
			num_of_samples = m_num_of_samples;

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

		m_samples.resize(samples_pos.size());

		int idx = 0;
		for (std::vector<Vec3>::iterator it = samples_pos.begin(); it != samples_pos.end(); ++it)
		{
			Vec3 point = *it;
			SamplePoint sample_point(point.x, point.y, point.z);
			m_samples[idx++] = sample_point;
		}

		return samples_pos.size();
	}
	else
		return 0;
}

int OBB::createRandomSamples()
{
	m_samples.clear();
	m_samples.reserve(m_num_of_samples);

	Real all_faces_area = 0;
	for (int face_index = 0; face_index < k_num_faces; ++face_index)
		all_faces_area += getFaceArea(face_index);
	assert(all_faces_area > 0);

	/* Sample points on each face */
	static SimpleRandomCong_t rng_cong;
	simplerandom_cong_seed(&rng_cong, CUBOID_SURFACE_SAMPLING_RANDOM_SEED);

	Matrix3f axes = getAxes();

	for (int face_index = 0; face_index < k_num_faces; ++face_index)
	{
		const int *corner_indices = k_face_corner_indices[face_index];
		std::array<Vector3f, k_num_face_corners> corner_point;
		for (int i = 0; i < k_num_face_corners; i++)
			corner_point[i] = m_vertices[corner_indices[i]];

		Vector3f normal = axes.col(face_index / 2);
		if ((face_index % 2) != 0)
			normal = -normal;

		Real face_area = getFaceArea(face_index);
		int num_face_points = std::round(face_area / all_faces_area * m_num_of_samples);

		for (int point_index = 0; point_index < num_face_points && m_samples.size() < m_num_of_samples; point_index++)
		{
			Real w1 = static_cast<Real>(simplerandom_cong_next(&rng_cong))
				/ std::numeric_limits<uint32_t>::max();
			Real w2 = static_cast<Real>(simplerandom_cong_next(&rng_cong))
				/ std::numeric_limits<uint32_t>::max();

			Vector3f p1 = w1 * (corner_point[1] - corner_point[0]) + corner_point[0];
			Vector3f p2 = w1 * (corner_point[2] - corner_point[3]) + corner_point[3];
			Vector3f point = w2 * (p2 - p1) + p1;

			Real sum_corner_weights = 0;
			std::array<Real, k_num_corners> corner_weights;
			corner_weights.fill(0.0);

			for (int i = 0; i < k_num_face_corners; i++)
			{
				corner_weights[corner_indices[i]] = (point - corner_point[i]).norm();
				sum_corner_weights += corner_weights[corner_indices[i]];  /* !!Here is different from the original code */
			}

			assert(sum_corner_weights > 0);
			for (int i = 0; i < k_num_face_corners; i++)
			{
				corner_weights[corner_indices[i]] =
					(sum_corner_weights - corner_weights[corner_indices[i]]) / sum_corner_weights;
			}

#ifdef DEBUG_TEST
			Vector3f same_point = Vector3f::Zero();
			for (int i = 0; i < k_num_corners; i++)
				same_point += corner_weights[i] * m_vertices[i];
			Real error = (same_point - point).norm();
			Utils::CHECK_NUMERICAL_ERROR(__FUNCTION__, error);
#endif
			SamplePoint sample(point, normal, face_index, corner_weights);
			m_samples.push_back(sample);
		}
	}

	return m_samples.size();
}

int OBB::createGridSamples()
{
	m_samples.clear();
	m_samples.reserve(m_num_of_samples);

	Real all_faces_area = 0;
	for (int face_index = 0; face_index < k_num_faces; ++face_index)
		all_faces_area += getFaceArea(face_index);

	/* NOTE:
	   The number of points may not be exactly the same with the given num_of_samples. */
	for (int face_index = 0; face_index < k_num_faces; ++face_index)
	{
		const int *corner_indices = k_face_corner_indices[face_index];
		std::array<Eigen::Vector3f, k_num_face_corners> corner_point;
		for (int i = 0; i < k_num_face_corners; i++)
			corner_point[i] = m_vertices[corner_indices[i]];

		Eigen::Vector3f normal = getAxes().col(face_index / 2);
		if ((face_index % 2) != 0)
			normal = -normal;

		Real face_area = getFaceArea(face_index);
		Real num_face_points = (face_area / all_faces_area * m_num_of_samples);

		Real axis_length[2];
		axis_length[0] = (corner_point[1] - corner_point[0]).norm() +
			(m_vertices[2] - m_vertices[3]).norm();
		axis_length[1] = (corner_point[3] - corner_point[0]).norm() +
			(corner_point[2] - corner_point[1]).norm();

		if (axis_length[0] <= MIN_CUBOID_SIZE || axis_length[1] <= MIN_CUBOID_SIZE)
			continue;

		Real ratio = std::sqrt(num_face_points / (axis_length[0] * axis_length[1]));

		unsigned int num_axis_points[2];
		for (unsigned int i = 0; i < 2; ++i)
		{
			num_axis_points[i] = static_cast<unsigned int>(std::round(ratio * axis_length[i]));
			num_axis_points[i] = std::min(num_axis_points[i], static_cast<unsigned int>(num_face_points / 2));
			num_axis_points[i] = std::max(num_axis_points[i], 2u);
		}

		for (int point_index_1 = 0; point_index_1 < num_axis_points[0]; ++point_index_1)
		{
			Real w1 = static_cast<Real>(point_index_1) / (num_axis_points[0] - 1);

			for (int point_index_2 = 0; point_index_2 < num_axis_points[1]; ++point_index_2)
			{
				Real w2 = static_cast<Real>(point_index_2) / (num_axis_points[1] - 1);

				Eigen::Vector3f p1 = w1 * (corner_point[1] - corner_point[0]) + corner_point[0];
				Eigen::Vector3f p2 = w1 * (corner_point[2] - corner_point[3]) + corner_point[3];
				Eigen::Vector3f point = w2 * (p2 - p1) + p1;

				Real sum_corner_weights = 0;
				std::array<Real, k_num_corners> corner_weights;
				corner_weights.fill(0);

				corner_weights[corner_indices[0]] = (1 - w1)*(1 - w2);
				corner_weights[corner_indices[1]] = (w1)*(1 - w2);
				corner_weights[corner_indices[2]] = (w1)*(w2);
				corner_weights[corner_indices[3]] = (1 - w1)*(w2);

#ifdef DEBUG_TEST
				Eigen::Vector3f same_point = Eigen::Vector3f::Zero();
				for (unsigned int i = 0; i < k_num_corners; ++i)
					same_point += corner_weights[i] * m_vertices[i];
				Real error = (same_point - point).norm();
				Utils::CHECK_NUMERICAL_ERROR(__FUNCTION__, error);
#endif

				SamplePoint cuboid_surface_point(point, normal, face_index, corner_weights);
				m_samples.push_back(cuboid_surface_point);
			}
		}
	}

	/* If the number of sample points on the OBB is less than the number it shuold have, sample some point more */
	/* Sample points on each face */
	//static SimpleRandomCong_t rng_cong;
	//simplerandom_cong_seed(&rng_cong, CUBOID_SURFACE_SAMPLING_RANDOM_SEED);
	//if (m_samples.size() < m_num_of_samples)
	//{
	//	int diff = m_num_of_samples - m_samples.size();
	//	
	//	int num_points_more_on_each_face = diff / k_num_faces;

	//	int second_diff = diff - num_points_more_on_each_face * k_num_faces;

	//	if (num_points_more_on_each_face > 0 || second_diff > 0)
	//	{
	//		for (int face_index = 0; face_index < k_num_faces; face_index++)
	//		{
	//			const int *corner_indices = k_face_corner_indices[face_index];
	//			std::array<Vector3f, k_num_face_corners> corner_point;
	//			for (int i = 0; i < k_num_face_corners; i++)
	//				corner_point[i] = m_vertices[corner_indices[i]];

	//			Vector3f normal = getAxes().col(face_index / 2);
	//			if ((face_index % 2) != 0)
	//				normal = -normal;

	//			int num_face_points = num_points_more_on_each_face;
	//			if (face_index < second_diff)
	//				num_face_points++;

	//			for (int point_index = 0; point_index < num_face_points && m_samples.size() < m_num_of_samples; point_index++)
	//			{
	//				Real w1 = static_cast<Real>(simplerandom_cong_next(&rng_cong))
	//					/ std::numeric_limits<uint32_t>::max();
	//				Real w2 = static_cast<Real>(simplerandom_cong_next(&rng_cong))
	//					/ std::numeric_limits<uint32_t>::max();

	//				Vector3f p1 = w1 * (corner_point[1] - corner_point[0]) + corner_point[0];
	//				Vector3f p2 = w1 * (corner_point[2] - corner_point[3]) + corner_point[3];
	//				Vector3f point = w2 * (p2 - p1) + p1;

	//				Real sum_corner_weights = 0;
	//				std::array<Real, k_num_corners> corner_weights;
	//				corner_weights.fill(0.0);

	//				for (int i = 0; i < k_num_face_corners; i++)
	//				{
	//					corner_weights[corner_indices[i]] = (point - corner_point[i]).norm();
	//					sum_corner_weights += corner_weights[corner_indices[i]];  /* !!Here is different from the original code */
	//				}

	//				assert(sum_corner_weights > 0);
	//				for (int i = 0; i < k_num_face_corners; i++)
	//				{
	//					corner_weights[corner_indices[i]] =
	//						(sum_corner_weights - corner_weights[corner_indices[i]]) / sum_corner_weights;
	//				}

	//				SamplePoint sample(point, normal, face_index, corner_weights);
	//				m_samples.push_back(sample);
	//			}
	//		}
	//	}
	//}
	//else  /* If the number of sample points is more than it should have, randomly delete some points */
	//{
	//	int diff = m_samples.size() - m_num_of_samples;

	//	srand(time(NULL));

	//	for (int i = 0; i < diff; i++)
	//	{
	//		int remove_index = rand() % m_samples.size();
	//		m_samples.remove(remove_index);
	//	}
	//}

	//int sur_samp_size = m_samples.size();
	//int should_num = m_num_of_samples;
	return m_samples.size();
}

QVector<Eigen::Vector3f> OBB::getSamplePoints() const
{
	QVector<Eigen::Vector3f> sample_points;

	sample_points.resize(m_samples.size());
	int sample_idx = 0;
	for (QVector<SamplePoint>::const_iterator sample_it = m_samples.begin(); sample_it != m_samples.end(); ++sample_it)
	{
		Eigen::Vector3f point = sample_it->getVertex();
		sample_points[sample_idx++] = point;
	}
	return sample_points;
}

void OBB::setSamplePoints(Eigen::MatrixXd samples_mat)
{
	if (!m_samples.empty())
		m_samples.clear();

	m_samples.resize(samples_mat.size());

	for (int i = 0; i < samples_mat.cols(); i++)
	{
		Eigen::Vector3d col_sample = samples_mat.col(i);
		SamplePoint sample((float)col_sample.x(), (float)col_sample.y(), (float)col_sample.z());
		m_samples[i] = sample;
	}
}

void OBB::setSamplePoints(pcl::PointCloud<pcl::PointXYZ> cloud)
{
	using namespace pcl;

	m_samples.clear();
	m_samples.resize(cloud.size());

	int sample_idx = 0;
	for (PointCloud<PointXYZ>::iterator point_it = cloud.begin(); point_it != cloud.end(); ++point_it)
	{
		SamplePoint sample(point_it->x, point_it->y, point_it->z);
		m_samples[sample_idx++] = sample;
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
	x_axis.normalize();
}

void OBB::setYAxis(Vector3f yAxis)
{
	y_axis = yAxis;
	y_axis.normalize();
}

void OBB::setZAxis(Vector3f zAxis)
{
	z_axis = zAxis;
	z_axis.normalize();
}

void OBB::setCentroid(Vector3f centroid)
{
	m_centroid = centroid;
}

void OBB::setXAxis(Vector3d xAxis)
{
	x_axis = Vector3f((float)xAxis.x(), (float)xAxis.y(), (float)xAxis.z());
	x_axis.normalize();
}

void OBB::setYAxis(Vector3d yAxis)
{
	y_axis = Vector3f((float)yAxis.x(), (float)yAxis.y(), (float)yAxis.z());
	y_axis.normalize();
}

void OBB::setZAxis(Vector3d zAxis)
{
	z_axis = Vector3f((float)zAxis.x(), (float)zAxis.y(), (float)zAxis.z());
	z_axis.normalize();
}

void OBB::setAxes(std::array<Vector3f, 3> new_axes)
{
	x_axis = new_axes[0];
	y_axis = new_axes[1];
	z_axis = new_axes[2];
	x_axis.normalize();
	y_axis.normalize();
	z_axis.normalize();

	computeFaceNormals();
}

void OBB::setAxes(Matrix3d new_axes)
{
	x_axis = new_axes.col(0).cast<float>();
	y_axis = new_axes.col(1).cast<float>();
	z_axis = new_axes.col(2).cast<float>();
	x_axis.normalize();
	y_axis.normalize();
	z_axis.normalize();

	computeFaceNormals();
}

void OBB::setCentroid(Vector3d centroid)
{
	m_centroid = Vector3f((float)centroid.x(), (float)centroid.y(), (float)centroid.z());
}

void OBB::setCorners(std::array<Vector3f, k_num_corners> new_corners)
{
	/* reset the corner vertices of the OBB */
	int vertex_idx = 0;
	for (std::array<Vector3f, k_num_corners>::iterator corner_it = new_corners.begin(); corner_it != new_corners.end(); ++corner_it)
		m_vertices[vertex_idx++] = *corner_it;

	/* Recompute the lenghth in each directions (recompute the scale) */
	x_length = (m_vertices[0] - m_vertices[1]).norm();
	y_length = (m_vertices[0] - m_vertices[3]).norm();
	z_length = (m_vertices[0] - m_vertices[4]).norm();

	//m_samples.clear();
}

void OBB::setScale(Vector3d scale)
{
	x_length = scale.x();
	y_length = scale.y();
	z_length = scale.z();
}

void OBB::computeFaceNormals()
{
	Eigen::Vector3f face_normals[6] = {
		x_axis, y_axis, z_axis, -x_axis, -y_axis, -z_axis
	};

	m_faces_normals.resize(12);
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

void OBB::draw(float scale)
{
	/* Draw the oriented box */
	if (m_vertices.size() < 3)
		triangulate();

	glColor4f(COLORS[m_label][0], COLORS[m_label][1], COLORS[m_label][2], 0.3);

	glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
	glEnable(GL_BLEND);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
	//draw_box(cuboid->get_bbox_corners());
	drawCuboid(scale);
	glDisable(GL_BLEND);

	glColor4f(0.0f, 0.0f, 0.0f, 1.0f);
	glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
	glLineWidth(3.0f);
	//draw_box(cuboid->get_bbox_corners());
	drawCuboid(scale);
	glLineWidth(1.0f);


	/* Draw the local system axes */
	if (m_draw_axes)
	{
		/* draw x axis */
		glColor4f(COLORS[0][0], COLORS[0][1], COLORS[0][2], 1.0);
		glLineWidth(2.5);
		glBegin(GL_LINES);
		glVertex3f(scale * m_centroid.x(), scale * m_centroid.y(), scale * m_centroid.z());
		Vector3f x_end = m_centroid + 0.25 * x_axis;
		glVertex3f(scale * x_end.x(), scale * x_end.y(), scale * x_end.z());
		glEnd();
		/* draw y axis */
		glColor4f(COLORS[1][0], COLORS[1][1], COLORS[1][2], 1.0);
		glBegin(GL_LINES);
		glVertex3f(scale * m_centroid.x(), scale * m_centroid.y(), scale * m_centroid.z());
		Vector3f y_end = m_centroid + 0.25 * y_axis;
		glVertex3f(scale * y_end.x(), scale * y_end.y(), scale * y_end.z());
		glEnd();
		/* draw z axis */
		glColor4f(COLORS[2][0], COLORS[2][1], COLORS[2][2], 1.0);
		glBegin(GL_LINES);
		glVertex3f(scale * m_centroid.x(), scale * m_centroid.y(), scale * m_centroid.z());
		Vector3f z_end = m_centroid + 0.25 * z_axis;
		glVertex3f(scale * z_end.x(), scale * z_end.y(), scale * z_end.z());
		glEnd();
	}
}

void OBB::drawCuboid(float scale)
{
	for (unsigned int face_index = 0; face_index < k_num_faces; ++face_index)
	{
		Eigen::Vector3f face_normal = m_faces_normals[face_index];

		glBegin(GL_QUADS);
		glNormal3f(face_normal[0], face_normal[1], face_normal[2]);

		for (unsigned int i = 0; i < k_num_face_corners; ++i)
		{
			unsigned int corner_index = k_face_corner_indices[face_index][i];
			
			glVertex3f(
				scale * m_vertices[corner_index][0],
				scale * m_vertices[corner_index][1],
				scale * m_vertices[corner_index][2]);
		}
		glEnd();
	}
}

void OBB::drawSamples(float scale)
{
	//glColor4f(COLORS[10][0], COLORS[10][1], COLORS[10][2], 1.0);
	glColor4f(0.3, 0.30, 0.3, 1.0);
	glBegin(GL_POINTS);

	for (QVector<SamplePoint>::iterator sample_it = m_samples.begin(); sample_it != m_samples.end(); ++sample_it)
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

QVector<SamplePoint> OBB::getSamples() const
{
	return m_samples;
}

SamplePoint OBB::getSample(int index)
{
	return m_samples[index];
}

SamplePoint & OBB::getSampleReference(int index)
{
	return m_samples[index];
}

void OBB::translate(float x, float y, float z)
{
	Translation<float, 3> trans(x, y, z);
	m_centroid = trans * m_centroid;

	for (QVector<Vector3f>::iterator vertex_it = m_vertices.begin(); vertex_it != m_vertices.end(); ++vertex_it)
		*vertex_it = trans * (*vertex_it);

	for (QVector<SamplePoint>::iterator sample_it = m_samples.begin(); sample_it != m_samples.end(); ++sample_it)
		sample_it->setVertex(trans * sample_it->getVertex());
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
	m_samples.clear();
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
	m_samples.clear();
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
	x_axis.normalize();
	y_axis.normalize();
	z_axis.normalize();
	m_centroid = centroid_aug.block<3, 1>(0, 0);

	m_vertices.clear();
	m_faces.clear();
	m_faces_normals.clear();
	m_samples.clear();
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

void OBB::transform(Eigen::Matrix3d rotate_mat, Eigen::Vector3d trans_vec, const std::vector<Eigen::Vector3f> &cloud)
{
	Matrix3f rotate = rotate_mat.cast<float>();
	Vector3f translation = trans_vec.cast<float>();

	Vector4f rest(0, 0, 0, 1.0);

	Matrix4f transform;
	transform.block<3, 3>(0, 0) = rotate;
	transform.block<3, 1>(0, 3) = translation;
	transform.block<1, 4>(3, 0) = rest;

	pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud(new pcl::PointCloud<pcl::PointXYZ>);

	point_cloud->width = cloud.size();
	point_cloud->height = 1;
	point_cloud->is_dense = false;
	point_cloud->points.resize(point_cloud->width * point_cloud->height);

	int point_idx = 0;
	for (std::vector<Eigen::Vector3f>::const_iterator point_it = cloud.begin(); point_it != cloud.end(); ++point_it)
	{
		point_cloud->points[point_idx].x = point_it->x();
		point_cloud->points[point_idx].y = point_it->y();
		point_cloud->points[point_idx].z = point_it->z();
		point_idx++;
	}

	this->transform(transform, point_cloud);
}

int OBB::sampleCount() const
{
	return m_samples.size();
}

void OBB::drawLine(int vert_no_0, int vert_no_1, float scale)
{
	Vector3f vert0 = m_vertices[vert_no_0];
	Vector3f vert1 = m_vertices[vert_no_1];
	glVertex3f(scale * vert0.x(), scale * vert0.y(), scale * vert0.z());
	glVertex3f(scale * vert1.x(), scale * vert1.y(), scale * vert1.z());
}

void OBB::drawLine(const Eigen::Vector3f & vert0, const Eigen::Vector3f & vert1, float scale)
{
	glVertex3f(scale * vert0.x(), scale * vert0.y(), scale * vert0.z());
	glVertex3f(scale * vert1.x(), scale * vert1.y(), scale * vert1.z());
}

QVector<Vector3f>::iterator OBB::vertices_begin()
{
	return m_vertices.begin();
}

QVector<Vector3f>::iterator OBB::vertices_end()
{
	return m_vertices.end();
}

QVector<SamplePoint>::iterator OBB::samples_begin()
{
	return m_samples.begin();
}

QVector<SamplePoint>::iterator OBB::samples_end()
{
	return m_samples.end();
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
	m_samples.clear();
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

void OBB::normalize(Eigen::Vector3f centroid, double radius)
{
	assert(radius > 0);
	//for (QVector<Eigen::Vector3f>::iterator corner_it = m_vertices.begin(); corner_it != m_vertices.end(); ++corner_it)
	//{
	//	*corner_it = *corner_it - centroid;
	//	*corner_it = *corner_it / radius;
	//}

	for (QVector<SamplePoint>::iterator sample_it = m_samples.begin(); sample_it != m_samples.end(); ++sample_it)
	{
		sample_it->setVertex(sample_it->getVertex() - centroid);
		sample_it->setVertex(sample_it->getVertex() / radius);
	}

	m_centroid -= centroid;
	m_centroid /= radius;

	x_length /= radius;
	y_length /= radius;
	z_length /= radius;

	triangulate();
}

double OBB::getFaceArea(int face_index)
{
	const int * corner_indices = k_face_corner_indices[face_index];
	std::array<Vector3f, k_num_face_corners> corner_point;
	for (int i = 0; i < k_num_face_corners; ++i)
		corner_point[i] = m_vertices[corner_indices[i]];

	Vector3f p = corner_point[2] - corner_point[0];
	Vector3f q = corner_point[3] - corner_point[1];
	double area = 0.5 * (double)(p.cross(q).norm());

	return area;
}

void OBB::updateCorners()
{
	if (m_vertices.size() != k_num_corners)
		m_vertices.resize(k_num_corners);

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

	for (int i = 0; i < k_num_corners; i++)
		m_vertices[i] = translations[i] * m_centroid;
}

void OBB::writeToFile(std::ofstream & out)
{
	/* Write the axes */
	out << x_axis.x() << " " << x_axis.y() << " " << x_axis.z() << endl;
	out << y_axis.x() << " " << y_axis.y() << " " << y_axis.z() << endl;
	out << z_axis.x() << " " << z_axis.y() << " " << z_axis.z() << endl;

	/* Write the lengths in 3 dimension */
	out << x_length << " " << y_length << " " << z_length << endl;

	/* Write the centroid */
	out << m_centroid.x() << " " << m_centroid.y() << " " << m_centroid.z() << endl;

	/* Write the color */
	out << m_color.x() << " " << m_color.y() << " " << m_color.z() << endl;

	/* Write the label */
	out << m_label << endl;

	/* Write the corners of the OBB */
	for (QVector<Eigen::Vector3f>::iterator corner_it = m_vertices.begin(); corner_it != m_vertices.end(); ++corner_it)
		out << corner_it->x() << " " << corner_it->y() << " " << corner_it->z() << endl;

	/* Write the faces and faces normals */
	int num_triangle_faces = 2 * k_num_faces;
	for (int i = 0; i < num_triangle_faces; i++)
	{
		Eigen::Vector3i face = m_faces[i];
		Eigen::Vector3f face_norm = m_faces_normals[i];

		out << face.x() << " " << face.y() << " " << face.z() << " "
			<< face_norm.x() << " " << face_norm.y() << " " << face_norm.z() << endl;
	}

	/* Write the sample points */
	out << m_samples.size() << endl;  /* Write the number of samples */
	for (QVector<SamplePoint>::iterator sample_it = m_samples.begin(); sample_it != m_samples.end(); ++sample_it)
		out << sample_it->toString() << endl;
}

void OBB::updateWithNewPoints(const std::vector<Eigen::Vector3f> & points)
{
	m_vertices.clear();
	m_faces.clear();
	m_faces_normals.clear();
	m_samples.clear();

	/* Recompute the x_length, y_length and z_length */
	float min[3] = { 100.0, 100.0, 100.0 };
	float max[3] = { -100.0, -100.0, -100.0 };
	Matrix3f local_axes_mat = getAxes();
	Matrix3f coord_trans = local_axes_mat.inverse();
	for (std::vector<Eigen::Vector3f>::const_iterator point_it = points.begin(); point_it != points.end(); ++point_it)
	{
		Vector3f point = coord_trans * (*point_it);

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