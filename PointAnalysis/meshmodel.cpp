#include "meshmodel.h"

using namespace Eigen;

MeshModel::MeshModel(QObject *parent)
	: QObject(parent), Model(Model::ModelType::Mesh)
{
	m_sample_count = 0;
	m_centroid.setZero();
}

MeshModel::MeshModel(const MeshModel &mesh)
	: Model(Model::ModelType::Mesh)
{
	m_vertices_list = mesh.getVertices();
	m_faces_list = mesh.getFaces();
	m_centroid = mesh.getCentroid();
	m_input_filepath = mesh.getInputFilepath();
	m_vertices_labels = mesh.getVerticesLabels();
	m_faces_labels = mesh.getFacesLabels();
	m_label_names = mesh.getLabelNames();
	m_parts_samples = mesh.getPartsSamples();
	m_vertices_normals = mesh.getVerticesNormals();
	m_faces_normals = mesh.getFacesNormals();
	m_sample_count = mesh.sampleCount();
}

MeshModel::MeshModel(const char *file_path)
	: Model(Model::ModelType::Mesh), m_sample_count(0)
{
	m_input_filepath = std::string(file_path);
	load_from_file(file_path);
	rotate(-90, 1.0, 0.0, 0.0);
	if (faceCount() > 0 && vertexCount() > 0)
		samplePoints();
}

MeshModel::MeshModel(std::string file_path)
	: Model(Model::ModelType::Mesh), m_sample_count(0)
{
	m_input_filepath = file_path;
	load_from_file(file_path.c_str());
	rotate(-90, 1.0, 0.0, 0.0);
	if (faceCount() > 0 && vertexCount() > 0)
		samplePoints();
}

MeshModel::~MeshModel()
{

}

QVector<Vector3f> MeshModel::getVertices() const
{
	return m_vertices_list;
}

QVector<Vector3i> MeshModel::getFaces() const
{
	return m_faces_list;
}

QVector<Vector3f> MeshModel::getVerticesNormals() const
{
	return m_vertices_normals;
}

QVector<Vector3f> MeshModel::getFacesNormals() const
{
	return m_faces_normals;
}

using namespace std;

void MeshModel::load_from_file(const char *file_path)
{
	if (!m_vertices_list.empty())
		m_vertices_list.clear();
	if (!m_faces_list.isEmpty())
		m_faces_list.clear();
	if (!m_vertices_labels.empty())
		m_vertices_labels.clear();
	if (!m_faces_labels.empty())
		m_faces_labels.clear();
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
			
			/* Normalize the vertices using the minmal bounding sphere */
			int vertex_idx = 0;
			for (PointVector::iterator point_it = S.begin(); point_it != S.end(); ++point_it)
			{
				Vector3f vertex(point_it->operator[](0), point_it->operator[](1), point_it->operator[](2));
				vertex -= center;
				vertex /= rad;
				m_vertices_list[vertex_idx++] = vertex;
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
			vertex_idx = 0;
			for (QVector<QList<int>>::iterator faces_list_it = vertex_faces_list.begin(); faces_list_it != vertex_faces_list.end(); ++faces_list_it)
			{
				Vector3f normal = Vector3f::Zero();
				for (QList<int>::iterator face_it = faces_list_it->begin(); face_it != faces_list_it->end(); ++face_it)
					normal += m_faces_normals[*face_it];

				normal.normalize();
				m_vertices_normals[vertex_idx++] = normal;
			}

			/* Load labels from .seg file */
			m_vertices_labels.resize(nvertices);
			m_faces_labels.resize(nfaces);
			QSet<int> labels_set;

			std::string seg_file_path = Utils::getSegFilename(file_path);
			ifstream seg_in(seg_file_path.c_str());
			if (seg_in.is_open())
			{
				char buffer[8];

				for (int i = 0; i < nfaces; i++)
				{
					seg_in.getline(buffer, 8);
					int label = atoi(buffer);
					Vector3i face = m_faces_list[i];

					m_faces_labels[i] = label;

					m_vertices_labels[face.x()] = label;
					m_vertices_labels[face.y()] = label;
					m_vertices_labels[face.z()] = label;

					if (!labels_set.contains(label))
						labels_set.insert(label);
				}

				/* Generate m_label_names from labels_set */
				m_label_names.resize(labels_set.size());
				int label_idx = 0;
				for (QSet<int>::iterator label_it = labels_set.begin(); label_it != labels_set.end(); ++label_it)
					m_label_names[label_idx++] = *label_it;
				/* Sort m_label_names to ascending order */
				qSort(m_label_names.begin(), m_label_names.end());

				seg_in.close();
			}
		}
		mesh_in.close();
	}
}

void MeshModel::samplePoints()
{
	using namespace Utils_sampling;

	m_sample_count = 0;
	QMap<int, vector<Vec3>> parts_verts;
	QMap<int, vector<Vec3>> parts_norms;
	QMap<int, vector<int>> parts_tris;
	QMap<int, QMap<int, int>> origin_new_maps;  /* For each part, there is a map indicating the index in the points vector of a point in original points list */
	QMap<int, int> parts_vertices_count;

	for (QVector<int>::iterator label_it = m_label_names.begin(); label_it != m_label_names.end(); ++label_it)
	{
		origin_new_maps.insert(*label_it, QMap<int, int>());
		parts_verts.insert(*label_it, vector<Vec3>());
		parts_norms.insert(*label_it, vector<Vec3>());
		parts_tris.insert(*label_it, vector<int>());
		parts_vertices_count.insert(*label_it, 0);
	}

	const int NUM_OF_SAMPLES = 1000;
	const int NULL_PART_INDEX = -1;

	/* Segment mesh into different parts according to the faces labels */
	int face_idx = 0;
	for (QVector<Vector3i>::iterator face_it = m_faces_list.begin(); face_it != m_faces_list.end(); ++face_it)
	{
		assert(face_idx < m_faces_labels.size());

		int label = m_faces_labels[face_idx++];
		int v0_idx = face_it->x();
		int v1_idx = face_it->y();
		int v2_idx = face_it->z();

		int vertex_idx_0 = origin_new_maps[label].value(v0_idx, NULL_PART_INDEX);
		if (vertex_idx_0 == NULL_PART_INDEX)
		{
			/* put the vertex to the corresponding part vertices list */
			Vector3f vertex = m_vertices_list[v0_idx];
			parts_verts[label].push_back(Vec3(vertex.x(), vertex.y(), vertex.z()));
			/* put the normal of the vertex to the corresponding part normals list */
			Vector3f normal = m_vertices_normals[v0_idx];
			parts_norms[label].push_back(Vec3(normal.x(), normal.y(), normal.z()));
			/* record the index of the vertex in current part vertices list */
			origin_new_maps[label].insert(v0_idx, parts_vertices_count.value(label));
			parts_tris[label].push_back(parts_vertices_count.value(label));
			parts_vertices_count[label]++;
		}
		else
			parts_tris[label].push_back(vertex_idx_0);

		int vertex_idx_1 = origin_new_maps[label].value(v1_idx, NULL_PART_INDEX);
		if (vertex_idx_1 == NULL_PART_INDEX)
		{
			Vector3f vertex = m_vertices_list[v1_idx];
			parts_verts[label].push_back(Vec3(vertex.x(), vertex.y(), vertex.z()));
			Vector3f normal = m_vertices_normals[v1_idx];
			parts_norms[label].push_back(Vec3(normal.x(), normal.y(), normal.z()));
			origin_new_maps[label].insert(v1_idx, parts_vertices_count.value(label));
			parts_tris[label].push_back(parts_vertices_count.value(label));
			parts_vertices_count[label]++;
		}
		else
			parts_tris[label].push_back(vertex_idx_1);

		int vertex_idx_2 = origin_new_maps[label].value(v2_idx, NULL_PART_INDEX);
		if (vertex_idx_2 == NULL_PART_INDEX)
		{
			Vector3f vertex = m_vertices_list[v2_idx];
			parts_verts[label].push_back(Vec3(vertex.x(), vertex.y(), vertex.z()));
			Vector3f normal = m_vertices_normals[v2_idx];
			parts_norms[label].push_back(Vec3(normal.x(), normal.y(), normal.z()));
			origin_new_maps[label].insert(v2_idx, parts_vertices_count.value(label));
			parts_tris[label].push_back(parts_vertices_count.value(label));
			parts_vertices_count[label]++;
		}
		else
			parts_tris[label].push_back(vertex_idx_2);
	}

	/* Sample on each distinct part */
	for (QMap<int, vector<Vec3>>::iterator part_it = parts_verts.begin(); part_it != parts_verts.end(); ++part_it)
	{
		int label = part_it.key();
		vector<Vec3> verts = part_it.value();
		vector<Vec3> nors = parts_norms[label];
		vector<int> tris = parts_tris[label];

		vector<Vec3> samples_pos;
		vector<Vec3> samples_nors;

		poisson_disk(0, NUM_OF_SAMPLES, verts, nors, tris, samples_pos, samples_nors);

		assert(samples_pos.size() == samples_nors.size());

		QVector<SamplePoint> samples(samples_pos.size());
		int sample_idx = 0;
		vector<Vec3>::iterator vert_it, norm_it;
		for (vert_it = samples_pos.begin(), norm_it = samples_nors.begin(); vert_it != samples_pos.end() && norm_it != samples_nors.end(); ++vert_it, ++norm_it)
		{
			SamplePoint sample(vert_it->x, vert_it->y, vert_it->z, norm_it->x, norm_it->y, norm_it->z);
			samples[sample_idx++] = sample;
			m_sample_count++;
		}
		m_parts_samples.insert(label, samples);
	}
}

void MeshModel::draw(int scale)
{
	using namespace Eigen;

	glBegin(GL_TRIANGLES);
	int face_idx = 0;
	for (QVector<Vector3i>::iterator face_it = m_faces_list.begin(); face_it != m_faces_list.end(); ++face_it)
	{
		int label = m_faces_labels[face_idx];
		int v0_idx = face_it->x();
		int v1_idx = face_it->y();
		int v2_idx = face_it->z();

		Vector3f v0 = m_vertices_list[v0_idx];
		Vector3f v1 = m_vertices_list[v1_idx];
		Vector3f v2 = m_vertices_list[v2_idx];

		Vector3f normal = m_faces_normals[face_idx++];

		glColor4f(COLORS[label][0], COLORS[label][1], COLORS[label][2], 0.8);
		glNormal3f(normal.x(), normal.y(), normal.z());

		glVertex3f(scale * v0.x(), scale * v0.y(), scale * v0.z());
		glVertex3f(scale * v1.x(), scale * v1.y(), scale * v1.z());
		glVertex3f(scale * v2.x(), scale * v2.y(), scale * v2.z());
	}
	glEnd();

	/* Draw the normals of the vertices */
	/*glBegin(GL_LINES);
	glColor4f(0.5, 0.5, 0.5, 1.0);
	for (int i = 0; i < vertexCount(); i++)
	{
		Vector3f vertex = scale * m_vertices_list[i];
		Vector3f normal = m_vertices_normals[i];
		Vector3f norm_end = vertex + normal;

		glVertex3f(vertex.x(), vertex.y(), vertex.z());
		glVertex3f(norm_end.x(), norm_end.y(), norm_end.z());
	}
	glEnd();*/

	/* Draw the normals of the faces */
	/*glBegin(GL_LINES);
	for (int i = 0; i < faceCount(); i++)
	{
		Vector3i face = m_faces_list[i];
		Vector3f v0 = m_vertices_list[face.x()];
		Vector3f v1 = m_vertices_list[face.y()];
		Vector3f v2 = m_vertices_list[face.z()];

		Vector3f normal = m_faces_normals[i];
		Vector3f mid = ((v0 + v1) / 2.0 + v2) / 2.0 * scale;
		Vector3f end = mid + normal;

		glVertex3f(mid.x(), mid.y(), mid.z());
		glVertex3f(end.x(), end.y(), end.z());
	}
	glEnd();*/

	/*glPointSize(4.0);
	glBegin(GL_POINTS);
	Vector3f v0 = m_vertices_list[1616];
	Vector3f v1 = m_vertices_list[174];
	Vector3f v2 = m_vertices_list[770];
	glVertex3f(scale * v0.x(), scale * v0.y(), scale * v0.z());
	glVertex3f(scale * v1.x(), scale * v1.y(), scale * v1.z());
	glVertex3f(scale * v2.x(), scale * v2.y(), scale * v2.z());
	glEnd();

	glBegin(GL_LINES);
	glColor4f(COLORS[0][0], COLORS[0][1], COLORS[0][2], 1.0);
	glVertex3f(0.0, 0, 0);
	glVertex3f(3.0, 0, 0);
	glColor4f(COLORS[1][0], COLORS[1][1], COLORS[1][2], 1.0);
	glVertex3f(0, 0, 0);
	glVertex3f(0, 3.0, 0);
	glColor4f(COLORS[2][0], COLORS[2][1], COLORS[2][2], 1.0);
	glVertex3f(0, 0, 0);
	glVertex3f(0, 0, 3.0);
	glColor4f(0.7, 0.7, 0.7, 1.0);
	Vector3f normal = 3 * m_faces_normals[2279];
	Vector3f start = scale * v0;
	Vector3f end = start + normal;
	glVertex3f(start.x(), start.y(), start.z());
	glVertex3f(end.x(), end.y(), end.z());
	glEnd();*/
}

void MeshModel::drawSamples(int scale)
{
	/* Draw the sample points */
	glBegin(GL_POINTS);
	for (Parts_Samples::iterator part_it = m_parts_samples.begin(); part_it != m_parts_samples.end(); ++part_it)
	{
		int label = part_it.key();

		glColor4f(0.8, 0.8, 0.8, 1.0);
		for (QVector<SamplePoint>::iterator vertex_it = part_it->begin(); vertex_it != part_it->end(); ++vertex_it)
		{
			glNormal3f(vertex_it->nx(), vertex_it->ny(), vertex_it->nz());
			glVertex3f(scale * vertex_it->x(), scale * vertex_it->y(), scale * vertex_it->z());
		}
	}
	glEnd();

	/* Draw the normals of sample points */
	/*glBegin(GL_LINES);
	for (Parts_Samples::iterator part_it = m_parts_samples.begin(); part_it != m_parts_samples.end(); ++part_it)
	{
		int label = part_it.key();

		glColor4f(0.5, 0.5, 0.5, 1.0);
		for (QVector<Sample>::iterator sample_it = part_it->begin(); sample_it != part_it->end(); ++sample_it)
		{
			Vector3f vertex = scale * sample_it->getVertex();
			Vector3f normal = sample_it->getNormal();
			Vector3f norm_end = vertex + normal;
			glVertex3f(vertex.x(), vertex.y(), vertex.z());
			glVertex3f(norm_end.x(), norm_end.y(), norm_end.z());
		}
	}
	glEnd();*/
}

void MeshModel::normalize()
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

int MeshModel::vertexCount() const
{
	return m_vertices_list.size();
}

int MeshModel::faceCount()
{
	return m_faces_list.size();
}

Vector3f MeshModel::getCentroid() const
{
	return m_centroid;
}

void MeshModel::rotate(float angle, float x, float y, float z)
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
	for (Parts_Samples::iterator part_it = m_parts_samples.begin(); part_it != m_parts_samples.end(); ++part_it)
	{
		for (QVector<SamplePoint>::iterator sample_it = part_it->begin(); sample_it != part_it->end(); ++sample_it)
		{
			sample_it->setVertex(rotation * sample_it->getVertex());
			sample_it->setNormal(rotation * sample_it->getNormal());
		}
	}

	m_centroid = rotation * m_centroid;
}

void MeshModel::output(const char *file_path)
{
	ofstream mesh_out(file_path);

	if (mesh_out.is_open())
	{
		string file_format = Utils::getFileFormat(file_path);
		if (file_format.compare("OFF") == 0)
		{
			/* Write the header */
			mesh_out << "OFF" << endl;
			mesh_out << m_vertices_list.size() << " " << m_faces_list.size() << " 0" << endl;

			/* Write the vertices */
			for (QVector<Vector3f>::iterator vertex_it = m_vertices_list.begin(); vertex_it != m_vertices_list.end(); ++vertex_it)
				mesh_out << vertex_it->x() << " " << vertex_it->y() << " " << vertex_it->z() << endl;

			/* Write the faces */
			for (QVector<Vector3i>::iterator face_it = m_faces_list.begin(); face_it != m_faces_list.end(); ++face_it)
				mesh_out << "3 " << face_it->x() << " " << face_it->y() << " " << face_it->z() << endl;
		}

		mesh_out.close();
	}
}

std::string MeshModel::getInputFilepath() const
{
	return m_input_filepath;
}

QVector<int> MeshModel::getVerticesLabels() const
{
	return m_vertices_labels;
}

QVector<int> MeshModel::getFacesLabels() const
{
	return m_faces_labels;
}

QVector<int> MeshModel::getLabelNames() const
{
	return m_label_names;
}

Parts_Samples MeshModel::getPartsSamples() const
{
	return m_parts_samples;
}

Vector3f MeshModel::getVertexNormal(int index)
{
	return m_vertices_normals[index];
}

int MeshModel::sampleCount() const
{
	return m_sample_count;
}

int MeshModel::numOfClasses()
{
	return m_label_names.size();
}