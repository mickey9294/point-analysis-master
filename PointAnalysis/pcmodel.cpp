#include "pcmodel.h"

using namespace std;

PCModel::PCModel() : m_count(0), max(0)
{
	center = QVector3D(0, 0, 0);
}

PCModel::PCModel(int nvertices, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::Normal>::Ptr normals)
	: m_count(0), max(0)
{
	m_data.resize(9 * nvertices);
	//m_curvature.resize(nvertices);

	for (int i = 0; i < nvertices; i++)
	{
		pcl::PointXYZ point = cloud->at(i);
		pcl::Normal normal;
		if (i < normals->size())
			normal = normals->at(i);
		else
			normal = pcl::Normal(0, 0, 0);
		QVector3D color(0.5, 0.5, 0.5);
		add(QVector3D(point.x, point.y, point.z), QVector3D(normal.normal_x, normal.normal_y, normal.normal_z), color);
	}

	normalize();
	/*QMatrix4x4 rot_mat;
	rot_mat.setToIdentity();
	rot_mat.rotate(-90, 1, 0, 0);
	transform(rot_mat);*/
}

PCModel::PCModel(int nvertices, QVector<float> data)
	: m_count(0), max(0)
{
	double xmean = 0, ymean = 0, zmean = 0;
	m_data.resize(9 * nvertices);

	for (int i = 0; i < nvertices; i++)
	{
		float *p = data.data() + i * 6;
		GLfloat x = p[0];
		GLfloat y = p[1];
		GLfloat z = p[2];
		GLfloat nx = p[3];
		GLfloat ny = p[4];
		GLfloat nz = p[5];
		QVector3D color(0.5, 0.5, 0.5);
		add(QVector3D(x, y, z), QVector3D(nx, ny, nz), color);
	}

	normalize();
	QMatrix4x4 rot_mat;
	rot_mat.setToIdentity();
	rot_mat.rotate(-90, 1, 0, 0);
	transform(rot_mat);
}

PCModel::PCModel(int nvertices, QVector<float> data, QVector<int> labels)
	: m_count(0), max(0)
{
	double xmean = 0, ymean = 0, zmean = 0;
	m_data.resize(9 * nvertices);
	m_labels.resize(nvertices);
	m_label_names.clear();

	for (int i = 0; i < nvertices; i++)
	{
		float *p = data.data() + i * 6;
		GLfloat x = p[0];
		GLfloat y = p[1];
		GLfloat z = p[2];
		GLfloat nx = p[3];
		GLfloat ny = p[4];
		GLfloat nz = p[5];
		QVector3D color;
		int label = labels[i];
		if (label <= 10 && label >= 0)
			color = QVector3D(COLORS[labels[i]][0], COLORS[labels[i]][1], COLORS[labels[i]][2]);
		else
			color = QVector3D(COLORS[10][0], COLORS[10][1], COLORS[10][2]);
		add(QVector3D(x, y, z), QVector3D(nx, ny, nz), color);

		if (!m_label_names.contains(label))
			m_label_names.push_back(label);
	}

	normalize();
	/*QMatrix4x4 rot_mat;
	rot_mat.setToIdentity();
	rot_mat.rotate(-90, 1, 0, 0);
	transform(rot_mat);*/

	/* Set the labels for each point */
	std::memcpy(m_labels.data(), labels.data(), nvertices * sizeof(int));
}

PCModel::~PCModel()
{

}

void PCModel::add(const QVector3D &v, const QVector3D &n, const QVector3D &c)
{
	GLfloat *p = m_data.data() + m_count;
	*p++ = v.x();
	*p++ = v.y();
	*p++ = v.z();
	*p++ = n.x();
	*p++ = n.y();
	*p++ = n.z();
	*p++ = c.x();
	*p++ = c.y();
	*p++ = c.z();

	m_count += 9;
}

void PCModel::transform(QMatrix4x4 transMatrix)
{
	for (int i = 0; i < m_count; i += 9)
	{
		GLfloat *p = m_data.data() + i;
		QVector4D point(p[0], p[1], p[2], 1.0);
		QVector4D normal(p[3], p[4], p[5], 0.0);
		QVector4D newPoint = transMatrix * point;
		p[0] = newPoint.x();
		p[1] = newPoint.y();
		p[2] = newPoint.z();
		QVector4D newNorm = transMatrix * normal;
		p[3] = newNorm.x();
		p[4] = newNorm.y();
		p[5] = newNorm.z();
	}
	center = transMatrix * center;
}

QVector3D PCModel::getCenter()
{
	return center;
}

void PCModel::output(const char *filename)
{
	ofstream out(filename);
	if (out.is_open())
	{
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

		int onePercent = m_count * 0.01;
		int progress_count = 1;
		for (int i = 0; i < m_count; i += 9)
		{
			if (i >= onePercent * progress_count)
			{
				emit(outputProgressReport(progress_count));
				progress_count++;
			}
			GLfloat *p = m_data.data() + i;
			out << p[0] << " " << p[1] << " " << p[2] << " "
				<< p[3] << " " << p[4] << " " << p[5] << std::endl;
		}
		if (progress_count < 100)
			emit(outputProgressReport(100));

		out.close();
	}
}

void PCModel::clear()
{
	m_data.clear();
	m_count = 0;
	center.setX(0);
	center.setY(0);
	center.setZ(0);
}


void PCModel::normalize()
{
	/* Data structure for Miniball computation */
	const int n = vertexCount();                        // number of points
	const int d = 3;                         // dimension of points
	PointVector S;
	vector<double> coords(d);

	GLfloat *p = m_data.data();

	for (int i = 0; i < vertexCount(); i++)
	{
		GLfloat *point = p + i * 9;

		coords[0] = point[0];
		coords[1] = point[1];
		coords[2] = point[2];
		S.push_back(MiniPoint(d, coords.begin()));
	}

	/* Compute the Miniball of point cloud */
	Miniball mb(d, S);
	double rad = mb.radius();
 	Miniball::Coordinate_iterator center_it = mb.center_begin();
	QVector3D mbs_center(center_it[0], center_it[1], center_it[2]);
	qDebug() << "Radius =" << rad
		<< "; Center :(" << mbs_center.x() << "," << mbs_center.y() << "," << mbs_center.z() << ")";
	
	for (int i = 0; i < vertexCount(); i++)
	{
		GLfloat *point = p + i * 9;
		point[0] = (point[0] - mbs_center.x()) / rad;
		point[1] = (point[1] - mbs_center.y()) / rad;
		point[2] = (point[2] - mbs_center.z()) / rad;

		float divisor = sqrt(point[3] * point[3] + point[4] * point[4] + point[5] * point[5]);
		point[3] /= divisor;
		point[4] /= divisor;
		point[5] /= divisor;
	}

	center.setX(0);
	center.setY(0);
	center.setZ(0);
	radius = 1.0;
}

void PCModel::setInputFilename(const char *name)
{
	inputfilename = string(name);
}

void PCModel::setInputFilename(std::string name)
{
	inputfilename = std::string(name);
}

std::string PCModel::getInputFilename()
{
	return inputfilename;
}

QVector<int> PCModel::getLabels()
{
	return QVector<int>(m_labels);
}

void PCModel::setLabels(QVector<int> labels)
{
	emit addDebugText("Set the labels of PCModel.");
	m_label_names.clear();
	m_labels.clear();
	m_labels.resize(labels.size());
	//m_labels = labels;
	for (int i = 0; i < vertexCount(); i++)
	{
		m_labels[i] = labels[i];

		GLfloat *point = m_data.data() + i * 9;
		int label = m_labels[i];
		point[6] = COLORS[m_labels[i]][0];
		point[7] = COLORS[m_labels[i]][1];
		point[8] = COLORS[m_labels[i]][2];

		if (!m_label_names.contains(label))
			m_label_names.push_back(label);
	}

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

QVector<double> PCModel::getSdf()
{
	return m_sdf;
}

QList<int> PCModel::getLabelNames()
{
	return m_label_names;
}

int PCModel::numOfClasses()
{
	return m_label_names.size();
}