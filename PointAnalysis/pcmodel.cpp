#include "pcmodel.h"

using namespace std;
using namespace Eigen;

PCModel::PCModel() 
	: Model(Model::ModelType::PointCloud)
{
	m_centroid.setZero();
}

PCModel::PCModel(const char *file_path, int normals_estimation_method)
	: Model(Model::ModelType::PointCloud)
{
	load_from_file(file_path, normals_estimation_method);
	m_input_filepath = std::string(file_path);
}

PCModel::PCModel(std::string file_path, int normals_estimation_method)
	: Model(Model::ModelType::PointCloud)
{
	load_from_file(file_path.c_str(), normals_estimation_method);
	m_input_filepath = file_path;
}

PCModel::PCModel(const PCModel &pc)
	: Model(Model::ModelType::PointCloud)
{
	m_vertices_list = pc.getVertices();
	m_normals_list = pc.getNormals();
	m_radius = pc.getRadius();
	m_input_filepath = pc.getInputFilepath();
	m_sdf = pc.getSdf();
	m_label_names = pc.getLabelNames();
	m_labels = pc.getVerticesLabels();
	m_centroid = pc.getCentroid();
}

//PCModel::PCModel(int nvertices, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::Normal>::Ptr normals)
//	: Model(Model::ModelType::PointCloud), m_count(0)
//{
//	m_data.resize(9 * nvertices);
//	//m_curvature.resize(nvertices);
//
//	for (int i = 0; i < nvertices; i++)
//	{
//		pcl::PointXYZ point = cloud->at(i);
//		pcl::Normal normal;
//		if (i < normals->size())
//			normal = normals->at(i);
//		else
//			normal = pcl::Normal(0, 0, 0);
//		QVector3D color(0.5, 0.5, 0.5);
//		add(QVector3D(point.x, point.y, point.z), QVector3D(normal.normal_x, normal.normal_y, normal.normal_z), color);
//	}
//
//	normalize();
//}

//PCModel::PCModel(int nvertices, QVector<float> data)
//	: Model(Model::ModelType::PointCloud), m_count(0)
//{
//	double xmean = 0, ymean = 0, zmean = 0;
//	m_data.resize(9 * nvertices);
//
//	for (int i = 0; i < nvertices; i++)
//	{
//		float *p = data.data() + i * 6;
//		GLfloat x = p[0];
//		GLfloat y = p[1];
//		GLfloat z = p[2];
//		GLfloat nx = p[3];
//		GLfloat ny = p[4];
//		GLfloat nz = p[5];
//		QVector3D color(0.5, 0.5, 0.5);
//		add(QVector3D(x, y, z), QVector3D(nx, ny, nz), color);
//	}
//
//	normalize();
//}

//PCModel::PCModel(int nvertices, QVector<float> data, QVector<int> labels)
//	: Model(Model::ModelType::PointCloud), m_count(0)
//{
//	double xmean = 0, ymean = 0, zmean = 0;
//	m_data.resize(9 * nvertices);
//	m_labels.resize(nvertices);
//	m_label_names.clear();
//
//	for (int i = 0; i < nvertices; i++)
//	{
//		float *p = data.data() + i * 6;
//		GLfloat x = p[0];
//		GLfloat y = p[1];
//		GLfloat z = p[2];
//		GLfloat nx = p[3];
//		GLfloat ny = p[4];
//		GLfloat nz = p[5];
//		QVector3D color;
//		int label = labels[i];
//		if (label <= 10 && label >= 0)
//			color = QVector3D(COLORS[labels[i]][0], COLORS[labels[i]][1], COLORS[labels[i]][2]);
//		else
//			color = QVector3D(COLORS[10][0], COLORS[10][1], COLORS[10][2]);
//		add(QVector3D(x, y, z), QVector3D(nx, ny, nz), color);
//
//		if (!m_label_names.contains(label))
//			m_label_names.push_back(label);
//	}
//
//	/* Sort m_label_names to the ascending order */
//	qSort(m_label_names.begin(), m_label_names.end());
//
//	normalize();
//
//	/* Set the labels for each point */
//	std::memcpy(m_labels.data(), labels.data(), nvertices * sizeof(int));
//}

PCModel::~PCModel()
{

}

void PCModel::load_from_file(const char *file_path, int normals_estimation_method)
{
	string path_string(file_path);
	string model_name = Utils::getModelName(path_string);
	string modified_path = "../data/models_modified/" + model_name + ".ply";

	ifstream pc_modified_in(modified_path.c_str());
	if (!pc_modified_in.is_open())  /* If the model has not ever been loaded and estimated normals */
	{
		ifstream pc_in(file_path);

		/* Read the vertices from .off file */
		if (pc_in.is_open())
		{
			char buffer[128];

			/* Data structure for computing the minimal bounding sphere */
			const int dimen = 3;
			PointVector S;
			vector<double> coords(dimen);

			pc_in.getline(buffer, 128);
			if (strcmp(buffer, "OFF") == 0)
			{
				pc_in.getline(buffer, 128);
				QString num_str(buffer);
				int nvertices = num_str.section(' ', 0, 0).toInt();

				m_vertices_list.resize(nvertices);
				m_normals_list.resize(nvertices);
				m_labels.resize(nvertices);
				m_labels.fill(10);

				for (int i = 0; i < nvertices; i++)
				{
					pc_in.getline(buffer, 128);
					QStringList vertex_str = QString(buffer).split(' ');

					float x = vertex_str[0].toFloat();
					float y = vertex_str[1].toFloat();
					float z = vertex_str[2].toFloat();

					coords[0] = x;
					coords[1] = y;
					coords[2] = z;

					S.push_back(MiniPoint(3, coords.begin()));
				}
			}


			if (S.size() > 0)    /* If the vertices are loaded successfully */
			{
				/* Compute the Miniball of the mesh */
				Miniball mb(dimen, S);
				double rad = mb.radius();
				Miniball::Coordinate_iterator center_it = mb.center_begin();

				if (normals_estimation_method == 0)    /* If use CGAL to compute the normals */
				{
					/* Normalize the point cloud and prepare for normals computation */
					PointList points_list;    /* The vertices container for normals estimation with CGAL */
					for (PointVector::iterator point_it = S.begin(); point_it != S.end(); ++point_it)
					{
						/* Normalize the point(vertex) */
						float x = (point_it->operator[](0) - center_it[0]) / rad;
						float y = (point_it->operator[](1) - center_it[1]) / rad;
						float z = (point_it->operator[](2) - center_it[2]) / rad;

						/* Create PointVectorPair for normals estimation */
						Vector nullVector;
						Point3 point(x, y, z);
						PointVectorPair point_vector(point, nullVector);
						/* Add the point to points list wichi will be sent to normal estimation process */
						points_list.push_back(point_vector);
					}

					/* Set the centroid to the origin point and set the radius to 1.0 */
					m_centroid.setZero();
					m_radius = 1.0;

					/* Estimates normals direction.
					   Note: pca_estimate_normals() requires an iterator over points
					   as well as property maps to access each point's position and normal. */
					qDebug() << "Estimating normals direction...";
					const int nb_neighbors = 18; // K-nearest neighbors = 3 rings
					CGAL::jet_estimate_normals<Concurrency_tag>(points_list.begin(), points_list.end(),
						CGAL::First_of_pair_property_map<PointVectorPair>(),
						CGAL::Second_of_pair_property_map<PointVectorPair>(),
						18);
					qDebug() << "Normals estimation done.";

					/* Orients normals.
					   Note: mst_orient_normals() requires an iterator over points
					   as well as property maps to access each point's position and normal. */
					qDebug() << "Orienting normals...";
					PointList::iterator unoriented_points_begin =
						CGAL::mst_orient_normals(points_list.begin(), points_list.end(),
						CGAL::First_of_pair_property_map<PointVectorPair>(),
						CGAL::Second_of_pair_property_map<PointVectorPair>(),
						16);
					qDebug() << "Normals orientation done.";

					/* Add vertices(point) and normals from points_list after computing normals to m_vertices_list and m_normals_list */
					int vertex_idx = 0;
					for (PointList::iterator point_it = points_list.begin(); point_it != points_list.end(); ++point_it)
					{
						Point3 point = point_it->first;
						Vector norm = point_it->second;

						Vector3f vertex(point.x(), point.y(), point.z());
						Vector3f normal(norm.x(), norm.y(), norm.z());

						m_vertices_list[vertex_idx] = vertex;
						m_normals_list[vertex_idx++] = normal;
					}
				}
				else    /* If use PCL to compute the normals */
				{
					pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);  /* The points container for normals computation with PCL */

					/* Normalize the point cloud and prepare for normals computation */
					for (PointVector::iterator point_it = S.begin(); point_it != S.end(); ++point_it)
					{
						/* Normalize the point(vertex) */
						float x = (point_it->operator[](0) - center_it[0]) / rad;
						float y = (point_it->operator[](1) - center_it[1]) / rad;
						float z = (point_it->operator[](2) - center_it[2]) / rad;

						cloud->push_back(pcl::PointXYZ(x, y, z));
					}

					/* Set the centroid to the origin point and set the radius to 1.0 */
					m_centroid.setZero();
					m_radius = 1.0;

					qDebug() << "Computing points normals...";
					/* Create the normal estimation class, and pass the input dataset to it */
					pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
					ne.setInputCloud(cloud);

					/* Create an empty kdtree representation, and pass it to the normal estimation object.
					   Its content will be filled inside the object, based on the given input dataset (as no other search surface is given). */
					pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
					ne.setSearchMethod(tree);

					pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);    /* The output normals container */

					/* Use all neighbors in a sphere of 0.1 radius */
					ne.setRadiusSearch(0.1);
					/* Compute the normals (as well as curvatures, but not used here) */
					ne.compute(*cloud_normals);
					qDebug() << "Normals computation done.";

					/*  Add vertices(point) and normals from cloud and cloud_normals to m_vertices_list and m_normals_list */
					int vertex_idx = 0;
					for (pcl::PointCloud<pcl::PointXYZ>::iterator point_it = cloud->begin(); point_it != cloud->end(); ++point_it)
					{
						Vector3f vertex(point_it->x, point_it->y, point_it->z);
						m_vertices_list[vertex_idx++] = vertex;
					}
					int normal_idx = 0;
					for (pcl::PointCloud<pcl::Normal>::iterator norm_it = cloud_normals->begin(); norm_it != cloud_normals->end(); ++norm_it)
					{
						Vector3f normal(norm_it->normal_x, norm_it->normal_y, norm_it->normal_z);
						m_normals_list[normal_idx++] = normal;
					}
				}
			}

			output(modified_path.c_str());

			pc_in.close();
		}
	}
	else  /* If the model has been loaded and estimated normals */
	{
		char buffer[128];

		pc_modified_in.getline(buffer, 16);
		assert(strcmp(buffer, "ply") == 0 || strcmp(buffer, "PLY") == 0);

		pc_modified_in.getline(buffer, 32);

		pc_modified_in.getline(buffer, 64);
		QString line(buffer);
		int num_vertices = line.section(' ', 2, 2).toInt();

		for (int i = 0; i < 7; i++)
			pc_modified_in.getline(buffer, 64);

		m_vertices_list.resize(num_vertices);
		m_normals_list.resize(num_vertices);
		m_centroid.setZero();
		m_radius = 1.0;
		m_labels.resize(num_vertices);
		m_labels.fill(10);

		for (int i = 0; i < num_vertices; i++)
		{
			pc_modified_in.getline(buffer, 128);
			QStringList line_list = QString(buffer).split(' ');

			Vector3f vertex;
			Vector3f normal;

			int idx = 0;
			for (QStringList::iterator it = line_list.begin(); it != line_list.end() && idx < 6; ++it)
			{
				if (idx < 3)
					vertex[idx++] = it->toFloat();
				else
				{
					normal[idx - 3] = it->toFloat();
					idx++;
				}
			}

			m_vertices_list[i] = vertex;
			m_normals_list[i] = normal;
		}
	}
}

Eigen::Vector3f PCModel::getCentroid() const
{
	return m_centroid;
}

void PCModel::draw(float scale)
{
	assert(m_vertices_list.size() == m_normals_list.size());

	glBegin(GL_POINTS);
	int label_idx = 0;
	QVector<Vector3f>::iterator vertex_it, normal_it;
	for (vertex_it = m_vertices_list.begin(), normal_it = m_normals_list.begin();
		vertex_it != m_vertices_list.end() && normal_it != m_normals_list.end(); ++vertex_it, ++normal_it)
	{
		int label = label_idx < m_labels.size() ? m_labels[label_idx] : 10;
		label_idx++;

		glColor4f(COLORS[label][0], COLORS[label][1], COLORS[label][2], 1.0);
		glNormal3f(normal_it->x(), normal_it->y(), normal_it->z());
		glVertex3f(scale * vertex_it->x(), scale * vertex_it->y(), scale * vertex_it->z());
	}
	/*for (int i = 0; i < vertexCount(); i++)
	{
		int label = i < m_labels.size() ? m_labels[i] : 10;
		glColor4f(COLORS[label][0], COLORS[label][1], COLORS[label][2], 1.0);
		Vector3f normal = m_normals_list[i];
		Vector3f vertex = m_vertices_list[i];
		glNormal3f(normal.x(), normal.y(), normal.z());
		glVertex3f(vertex.x(), vertex.y(), vertex.z());
	}*/
	glEnd();
}

void PCModel::output(const char *filename)
{
	ofstream out(filename);
	if (out.is_open())
	{
		string file_format = Utils::getFileFormat(filename);

		int progress_count;

		if (file_format.compare("ply") == 0 || file_format.compare("PLY") == 0)
		{
			/* Write the header */
			out << "ply" << std::endl;
			out << "format ascii 1.0" << std::endl;
			string pointsNum = std::to_string(vertexCount());
			string elementVertex = "element vertex " + pointsNum;
			out << elementVertex << std::endl;
			out << "property float x" << std::endl;
			out << "property float y" << std::endl;
			out << "property float z" << std::endl;
			out << "property float nx" << std::endl;
			out << "property float ny" << std::endl;
			out << "property float nz" << std::endl;
			out << "end_header" << std::endl;

			/* Write the vertices and normals */
			int onePercent = vertexCount() * 0.01;
			progress_count = 1;
			for (int i = 0; i < vertexCount(); i++)
			{
				if (i >= onePercent * progress_count)
				{
					emit(outputProgressReport(progress_count));
					progress_count++;
				}
				
				Vector3f vertex = m_vertices_list[i];
				Vector3f normal = m_normals_list[i];

				out << vertex.x() << " " << vertex.y() << " " << vertex.z() << " "
					<< normal.x() << " " << normal.y() << " " << normal.z() << std::endl;
			}
			
		}
		else if (file_format.compare("off") == 0 || file_format.compare("OFF") == 0)
		{
			/* Write the header */
			out << "OFF" << endl;
			out << vertexCount() << " 0 0" << endl;

			/* Write the vertices */
			int onePercent = vertexCount() * 0.01;
			progress_count = 1;
			int i = 0;
			for (QVector<Vector3f>::iterator vertex_it = m_vertices_list.begin(); vertex_it != m_vertices_list.end(); ++vertex_it)
			{
				if (i >= onePercent * progress_count)
				{
					emit(outputProgressReport(progress_count));
					progress_count++;
				}

				out << vertex_it->x() << " " << vertex_it->y() << " " << vertex_it->z() << endl;

				i++;
			}
		}

		if (progress_count < 100)
			emit(outputProgressReport(100));

		out.close();
	}
}

void PCModel::normalize()
{
	int nvertices = m_vertices_list.size();
	const int dimen = 3;
	PointVector S;
	vector<double> coords(dimen);

	for (QVector<Vector3f>::iterator vertex_it = m_vertices_list.begin(); vertex_it != m_vertices_list.end(); ++vertex_it)
	{
		coords[0] = vertex_it->x();
		coords[1] = vertex_it->y();
		coords[2] = vertex_it->z();

		S.push_back(MiniPoint(dimen, coords.begin()));
	}

	/* Compute the Miniball of the mesh */
	Miniball mb(dimen, S);
	double rad = mb.radius();
	Miniball::Coordinate_iterator center_it = mb.center_begin();
	Vector3f center(center_it[0], center_it[1], center_it[2]);

	for (QVector<Vector3f>::iterator vertex_it = m_vertices_list.begin(); vertex_it != m_vertices_list.end(); ++vertex_it)
	{
		float x = vertex_it->x();
		float y = vertex_it->y();
		float z = vertex_it->z();
		vertex_it->operator-=(center);
		vertex_it->operator/=(rad);
	}

	m_centroid.setZero();
	m_radius = 1.0;
}

void PCModel::setInputFilename(const char *name)
{
	m_input_filepath = string(name);
}

void PCModel::setInputFilename(std::string name)
{
	m_input_filepath = std::string(name);
}

std::string PCModel::getInputFilepath() const
{
	return m_input_filepath;
}

QVector<int> PCModel::getVerticesLabels() const
{
	return QVector<int>(m_labels);
}

Vector3f PCModel::getVertexNormal(int index)
{
	return m_normals_list[index];
}

void PCModel::setLabels(QVector<int> labels)
{
	emit addDebugText("Set the labels of PCModel.");
	m_label_names.clear();
	m_labels.clear();
	m_labels.resize(labels.size());
	QSet<int> labels_set;
	//m_labels = labels;
	for (int i = 0; i < vertexCount(); i++)
	{
		m_labels[i] = labels[i];

		//GLfloat *point = m_data.data() + i * 9;
		int label = m_labels[i];
		/*point[6] = COLORS[m_labels[i]][0];
		point[7] = COLORS[m_labels[i]][1];
		point[8] = COLORS[m_labels[i]][2];*/

		if (!labels_set.contains(label))
			labels_set.insert(label);
	}

	/* Add labels from label_set to m_label_names */
	m_label_names.resize(labels_set.size());
	int label_idx = 0;
	for (QSet<int>::iterator label_it = labels_set.begin(); label_it != labels_set.end(); ++label_it)
		m_label_names[label_idx++] = *label_it;

	/* Sort the m_label_names to the ascending order */
	qSort(m_label_names.begin(), m_label_names.end());

	emit onLabelsChanged();
}

void PCModel::receiveSignalTest()
{
	emit addDebugText("Signal test succeed!");
}

void PCModel::setSdf(QVector<double> sdf)
{
	int size = sdf.size();
	m_sdf.resize(size);
	std::memcpy(m_sdf.data(), sdf.data(), size * sizeof(double));
}

QVector<double> PCModel::getSdf() const
{
	return m_sdf;
}

QVector<int> PCModel::getLabelNames() const
{
	return m_label_names;
}

int PCModel::numOfClasses()
{
	return m_label_names.size();
}

void PCModel::rotate(float angle, float x, float y, float z)
{
	float _angle = angle / 180.0 * PI;
	AngleAxis<float> rotation(_angle, Vector3f(x, y, z));

	for (QVector<Vector3f>::iterator vertex_it = m_vertices_list.begin(); vertex_it != m_vertices_list.end(); ++vertex_it)
		*vertex_it = rotation * (*vertex_it);

	for (QVector<Vector3f>::iterator norm_it = m_normals_list.begin(); norm_it != m_normals_list.end(); ++norm_it)
		*norm_it = rotation * (*norm_it);

	m_centroid = rotation * m_centroid;
}

QVector<Vector3f> PCModel::getVertices() const
{
	return m_vertices_list;
}

QVector<Vector3f> PCModel::getNormals() const
{
	return m_normals_list;
}

Vector3f & PCModel::operator[](int index)
{
	return m_vertices_list[index];
}

Vector3f PCModel::at(int index)
{
	return m_vertices_list.at(index);
}

Vector3f PCModel::getVertex(int index)
{
	if (index < m_vertices_list.size())
		return m_vertices_list[index];
	else
		return Vector3f::Zero();
}

Vector3f PCModel::getNormal(int index)
{
	if (index < m_normals_list.size())
		return m_normals_list[index];
	else
		return Vector3f::Zero();
}
