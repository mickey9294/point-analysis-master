#include "meshpcmodel.h"

MeshPcModel::MeshPcModel(QObject *parent)
	: QObject(parent), Model(Model::ModelType::MeshPointCloud)
{

}

MeshPcModel::~MeshPcModel()
{

}

MeshPcModel::MeshPcModel(const char * file_path, QObject *parent)
	: QObject(parent), Model(Model::ModelType::MeshPointCloud)
{
	m_input_filepath = file_path;
	load_from_file(file_path);
	samplePoints();
}

MeshPcModel::MeshPcModel(std::string file_path, QObject *parent)
	: QObject(parent), Model(Model::ModelType::MeshPointCloud)
{
	m_input_filepath = file_path;
	load_from_file(file_path.c_str());
	samplePoints();
}

MeshPcModel::MeshPcModel(const MeshPcModel &other)
	: Model(Model::ModelType::MeshPointCloud)
{
	m_vertices_list = other.getVertices();
	m_faces_list = other.getFaces();
	m_vertices_normals = other.getVerticesNormals();
	m_faces_normals = other.getFacesNormals();
	m_centroid = other.getCentroid();
	m_radius = other.getRadius();
	m_samples_labels = other.getVerticesLabels();
	m_label_names = other.getLabelNames();
	m_input_filepath = other.getInputFilepath();
	m_samples = other.getSamples();

}

using namespace std;
using namespace Eigen;

QVector<Eigen::Vector3f> MeshPcModel::getVertices() const
{
	return m_vertices_list;
}

QVector<Eigen::Vector3i> MeshPcModel::getFaces() const
{
	return m_faces_list;
}

QVector<Eigen::Vector3f> MeshPcModel::getFacesNormals() const
{
	return m_faces_normals;
}

QVector<Eigen::Vector3f> MeshPcModel::getVerticesNormals() const
{
	return m_vertices_normals;
}

Eigen::Vector3f MeshPcModel::getVertexNormal(int index)
{
	return m_vertices_normals[index];
}

std::string MeshPcModel::getInputFilepath() const
{
	return m_input_filepath;
}

QVector<int> MeshPcModel::getVerticesLabels() const
{
	return m_samples_labels;
}

QVector<int> MeshPcModel::getLabelNames() const
{
	return m_label_names;
}

double MeshPcModel::getRadius() const
{
	return m_radius;
}

Eigen::Vector3f MeshPcModel::getCentroid() const
{
	return m_centroid;
}

int MeshPcModel::numOfClasses()
{
	return 8;
}

void MeshPcModel::output(const char * file_path)
{
	std::ofstream out(file_path);
	if (out.is_open())
	{
		QString q_file_path(file_path);
		QString format = q_file_path.section('.', -1, -1);

		if (format == "pts" || format == "PTS")
		{
			for (QVector<SamplePoint>::iterator sample_it = m_samples.begin(); sample_it != m_samples.end(); ++sample_it)
			{
				Vector3f bary = sample_it->getBary();
				Vector3f point = sample_it->getVertex();
				out << sample_it->getFaceIndex() << " "
					<< bary[0] << " " << bary[1] << " " << bary[2] << " "
					<< point[0] << " " << point[1] << " " << point[2] << endl;
			}
		}
		else if (format == "off" || format == "OFF")
		{
			out << "OFF" << std::endl;
			out << m_samples.size() << " 0 0" << std::endl;
			for (QVector<SamplePoint>::iterator sample_it = m_samples.begin(); sample_it != m_samples.end(); ++sample_it)
			{
				out << sample_it->x() << " " << sample_it->y() << " " << sample_it->z() << std::endl;
			}
		}

		out.close();
	}
}

QVector<SamplePoint> MeshPcModel::getSamples() const
{
	return m_samples;
}

void MeshPcModel::load_from_file(const char *file_path)
{
	if (!m_vertices_list.empty())
		m_vertices_list.clear();
	if (!m_faces_list.isEmpty())
		m_faces_list.clear();
	if (!m_label_names.empty())
		m_label_names.clear();

	ifstream mesh_in(file_path);

	if (mesh_in.is_open())
	{
		/* Load the vertices and triangle faces from .off file */
		char buffer[128];

		mesh_in.getline(buffer, 128);
		if (strcmp(buffer, "OFF") == 0)
		{
			mesh_in.getline(buffer, 128);
			QString nums_str(buffer);
			int nvertices = nums_str.section(' ', 0, 0).toInt();
			int nfaces = nums_str.section(' ', 1, 1).toInt();

			assert(nvertices > 0);

			m_vertices_list.resize(nvertices);
			m_vertices_normals.resize(nvertices);
			m_faces_list.resize(nfaces);
			m_faces_normals.resize(nfaces);

			/* Data structure for computing the minimal bounding sphere */
			const int dimen = 3;
			PointVector S;
			vector<double> coords(dimen);

			/* Read vertices to the points container for bounding sphere computation */
			for (int i = 0; i < nvertices; i++)
			{
				mesh_in.getline(buffer, 128);
				QStringList vertex_str = QString(buffer).split(' ');
				float x = vertex_str[0].toFloat();
				float y = vertex_str[1].toFloat();
				float z = vertex_str[2].toFloat();

				Vector3f vertex(x, y, z);
				m_vertices_list[i] = vertex;

				coords[0] = x;
				coords[1] = y;
				coords[2] = z;

				S.push_back(MiniPoint(3, coords.begin()));
			}

			/* Compute the Miniball of the mesh */
			Miniball mb(dimen, S);
			double rad = mb.radius();
			Miniball::Coordinate_iterator center_it = mb.center_begin();
			Vector3f center(center_it[0], center_it[1], center_it[2]);

			//m_centroid = center;
			//m_radius = rad;

			for (QVector<Eigen::Vector3f>::iterator vert_it = m_vertices_list.begin(); vert_it != m_vertices_list.end(); ++vert_it)
			{
				vert_it->operator-=(center);
				vert_it->operator/=(rad);
			}
			m_centroid.setZero();
			m_radius = 1.0;

			/* Read faces(triangle face) and compute normals of faces */
			QVector<QList<int>> vertex_faces_list(nvertices);   /* The i-th component is a list of the faces which contain the i-th vertex */
			for (int i = 0; i < nfaces; i++)
			{
				mesh_in.getline(buffer, 128);
				QStringList face_str = QString(buffer).split(' ');
				int v0 = face_str[1].toInt();
				int v1 = face_str[2].toInt();
				int v2 = face_str[3].toInt();

				m_faces_list[i] = Vector3i(v0, v1, v2);

				/* Compute the normal of the triangle face */
				Vector3f vertex0 = m_vertices_list[v0];
				Vector3f vertex1 = m_vertices_list[v1];
				Vector3f vertex2 = m_vertices_list[v2];
				Vector3f vec0 = vertex0 - vertex1;
				Vector3f vec1 = vertex2 - vertex1;
				Vector3f normal = vec0.cross(vec1);
				normal.normalize();
				m_faces_normals[i] = normal;

				/* Add the face to the faces list of corresponding vertices */
				vertex_faces_list[v0].push_back(i);
				vertex_faces_list[v1].push_back(i);
				vertex_faces_list[v2].push_back(i);
			}

			/* Compute the normals of vertices by computing the average of normals of faces which contain this vertex */
			int vertex_idx = 0;
			for (QVector<QList<int>>::iterator faces_list_it = vertex_faces_list.begin(); faces_list_it != vertex_faces_list.end(); ++faces_list_it)
			{
				Vector3f normal = Vector3f::Zero();
				for (QList<int>::iterator face_it = faces_list_it->begin(); face_it != faces_list_it->end(); ++face_it)
					normal += m_faces_normals[*face_it];

				normal.normalize();
				m_vertices_normals[vertex_idx++] = normal;
			}
		}
		mesh_in.close();
	}
}

int MeshPcModel::vertexCount() const
{
	return m_vertices_list.size();
}

int MeshPcModel::faceCount() const
{
	return m_faces_list.size();
}

int MeshPcModel::sampleCount() const
{
	return m_samples.size();
}

void MeshPcModel::samplePoints()
{
	std::string model_name = Utils::getModelName(m_input_filepath);
	std::string pts_path = "../data/points/" + model_name + ".pts";
	std::ifstream pts_in(pts_path.c_str());
	if (pts_in.is_open())
	{
		std::cout << "Load sample points from " << pts_path << std::endl;
		char buffer[128];
		m_samples.clear();

		while (!pts_in.eof())
		{
			pts_in.getline(buffer, 128);

			if (strlen(buffer) > 0)
			{
				QStringList line_list = QString(buffer).split(' ');

				QStringList::iterator line_it = line_list.begin();
				 
				int face_index = line_it->toInt();
				line_it++;

				Eigen::Vector3f bary_coord;
				for (int i = 0; i < 3 && line_it != line_list.end(); i++, line_it++)
					bary_coord[i] = line_it->toFloat();

				Eigen::Vector3f position;
				for (int i = 0; i < 3 && line_it != line_list.end(); i++, line_it++)
					position[i] = line_it->toFloat();

				SamplePoint sample_point(position, Vector3f::Zero(), bary_coord);
				m_samples.push_back(sample_point);
			}
		}

		m_samples.squeeze();
		m_samples_labels.resize(m_samples.size());
		m_samples_labels.fill(10);
		pts_in.close();
	}
	else
	{
		const int nsamples = 5000;
		std::cout << "Sample " << nsamples << " points on the mesh." << std::endl;
		createRandom(nsamples);
		m_samples_labels.resize(m_samples.size());
		m_samples_labels.fill(-1);

		//rotate(90, 1, 0, 0);
		std::string output_path = "../data/points/" + model_name + ".pts";
		output(output_path.c_str());
	}
}

double MeshPcModel::TriangleArea(const Eigen::Vector3f &pA, const Eigen::Vector3f &pB, const Eigen::Vector3f &pC)
{
	//calculate the lengths of sides pA-pB and pA-pC
	Eigen::Vector3f v1 = pB - pA;
	double a = v1.norm();
	Eigen::Vector3f v2 = pC - pA;
	double b = v2.norm();
	//calculate the angle between sides pA-pB and pA-pC
	if (a == 0 || b == 0)
		return 0;
	double cos_angle = v1.dot(v2) / (a * b);

	//avoid accumulative error
	if (cos_angle>1)
		cos_angle = 1;
	else if (cos_angle<-1)
		cos_angle = -1;

	double angle = acos(cos_angle);
	double area = a*b*sin(angle) / 2;

	return area;
}

double MeshPcModel::faceArea(int index)
{
	Vector3i face = m_faces_list[index];
	Vector3f v0 = m_vertices_list[face[0]];
	Vector3f v1 = m_vertices_list[face[1]];
	Vector3f v2 = m_vertices_list[face[2]];

	return TriangleArea(v0, v1, v2);
}

void MeshPcModel::createRandom(int nPnts)
{
	int nFaces = faceCount();
	m_samples.reserve(nPnts);

	/* compute face areas */
	double * faceProb = new double[nFaces];
	double totalArea = 0;
	for (int i = 0; i<nFaces; i++)
	{
		faceProb[i] = faceArea(i);
		totalArea += faceProb[i];
	}

	// order faces randomly
	std::vector<int> faceOrder;
	for (int i = 0; i<nFaces; i++)
		faceOrder.push_back(i);

	for (int i = 0; i<nFaces; i++)
	{
		int r = rand() % nFaces;
		int fID = faceOrder[r];
		faceOrder[r] = faceOrder[i];
		faceOrder[i] = fID;
	}

	// keep adding faces
	double cumProb = 0;
	int nAdded = 0;
	for (int i = 0; i < (int)faceOrder.size(); i++)
	{
		int fID = faceOrder[i];
		cumProb += (faceProb[fID] / totalArea);
		int nExpected = (int)ceil(cumProb * (double)nPnts);
		//		std::cout<<i<<": "<<cumProb<<" -> "<<nExpected<<std::endl;
		while (nAdded < nExpected && nAdded < nPnts)
		{
			RNScalar r1 = sqrt(RNRandomScalar());
			RNScalar r2 = RNRandomScalar();
			RNScalar t0 = (1.0 - r1);
			RNScalar t1 = r1 * (1.0 - r2);
			//pnts->AddPoint(ShapePoint(mesh, fID, t0, t1, pnts, pnts->NumPoints()));
			float b0 = t0;
			float b1 = t1;
			float b2 = 1 - t0 - t1;

			Vector3f sample_pos = Vector3f::Zero();
			Vector3i face = m_faces_list[fID];
			sample_pos += b0 * m_vertices_list[face[0]];
			sample_pos += b1 * m_vertices_list[face[1]];
			sample_pos += b2 * m_vertices_list[face[2]];

			Vector3f sample_norm = m_faces_normals[fID];

			SamplePoint sample_point(sample_pos, sample_norm, Vector3f(b0, b1, b2));
			sample_point.setFaceIndex(fID);
			m_samples.push_back(sample_point);

			nAdded++;
		}
		if (nAdded == nPnts)
			break;
	}
	delete[] faceProb;
}

void MeshPcModel::rotate(float angle, float x, float y, float z)
{
	float _angle = angle / 180.0 * PI;
	AngleAxis<float> rotation(_angle, Vector3f(x, y, z));

	/* Rotate each vertex in the mesh */
	for (QVector<Vector3f>::iterator vertex_it = m_vertices_list.begin(); vertex_it != m_vertices_list.end(); ++vertex_it)
		*vertex_it = rotation * (*vertex_it);

	/* Rotate normals of the vertices */
	for (QVector<Vector3f>::iterator vertex_normal_it = m_vertices_normals.begin(); vertex_normal_it != m_vertices_normals.end(); ++vertex_normal_it)
		*vertex_normal_it = rotation * (*vertex_normal_it);

	/* Rotate normals of the faces */
	for (QVector<Vector3f>::iterator face_normal_it = m_faces_normals.begin(); face_normal_it != m_faces_normals.end(); ++face_normal_it)
		*face_normal_it = rotation * (*face_normal_it);

	/* Rotate the sample points and their normals */
	for (QVector<SamplePoint>::iterator sample_it = m_samples.begin(); sample_it != m_samples.end(); ++sample_it)
	{
		sample_it->setVertex(rotation * sample_it->getVertex());
		sample_it->setNormal(rotation * sample_it->getNormal());
	}

	m_centroid = rotation * m_centroid;
}

void MeshPcModel::draw(float scale)
{
	/*glLineWidth(1.0);
	glColor4f(0.0, 1.0, 0.2, 1.0);
	glBegin(GL_LINES);
	for (QVector<Eigen::Vector3i>::iterator face_it = m_faces_list.begin(); face_it != m_faces_list.end(); ++face_it)
	{
		drawLine(face_it->x(), face_it->y(), scale);
		drawLine(face_it->z(), face_it->y(), scale);
		drawLine(face_it->x(), face_it->z(), scale);
	}
	glEnd();*/

	drawSamples(scale);
}

void MeshPcModel::drawSamples(float scale)
{
	assert(m_samples.size() == m_samples_labels.size());
	glPointSize(1.5);
	glBegin(GL_POINTS);
	int sample_idx = 0;
	for (QVector<SamplePoint>::iterator sample_it = m_samples.begin(); sample_it != m_samples.end(); ++sample_it)
	{
		int label = m_samples_labels[sample_idx++];
		if (label >= 0)
			glColor4f(COLORS[label][0], COLORS[label][1], COLORS[label][2], 1.0);
		else
			glColor4f(COLORS[10][0], COLORS[10][1], COLORS[2][0], 1.0);

		glNormal3f(sample_it->nx(), sample_it->ny(), sample_it->nz());
		glVertex3f(scale * sample_it->x(), scale * sample_it->y(), scale * sample_it->z());
	}
	glEnd();
}

void MeshPcModel::drawLine(int vertex_no_0, int vertex_no_1, float scale)
{
	Vector3f vert0 = m_vertices_list[vertex_no_0];
	Vector3f vert1 = m_vertices_list[vertex_no_1];
	glVertex3f(scale * vert0.x(), scale * vert0.y(), scale * vert0.z());
	glVertex3f(scale * vert1.x(), scale * vert1.y(), scale * vert1.z());
}

QVector<SamplePoint>::iterator MeshPcModel::samples_begin()
{
	return m_samples.begin();
}

QVector<SamplePoint>::iterator MeshPcModel::samples_end()
{
	return m_samples.end();
}

void MeshPcModel::setLabels(QVector<int> labels)
{
	m_samples_labels.resize(labels.size());

	for (int i = 0; i < labels.size(); i++)
		m_samples_labels[i] = labels[i];

	emit onLabelsChanged();
}