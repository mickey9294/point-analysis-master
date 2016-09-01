#ifndef MESHMODEL_H
#define MESHMODEL_H

#include <QObject>
#include <Eigen\Core>
#include <Eigen/Geometry>
#include <qvector.h>
#include <qmap.h>
#include <string>
#include <fstream>
#include <assert.h>
#include <vector>
#include <QtOpenGL>
#include <qset.h>
#include <qalgorithms.h>
#include "Seb.h"
#include "utils.h"
#include "model.h"
#include <utils_sampling.h>
#include "SamplePoint.h"

typedef Seb::Point<float> MiniPoint;
typedef std::vector<MiniPoint> PointVector;
typedef Seb::Smallest_enclosing_ball<float> Miniball;

typedef QMap<int, QVector<SamplePoint>> Parts_Samples;

class MeshModel : public QObject, public Model
{
	Q_OBJECT

public:
	MeshModel(QObject *parent = 0);
	MeshModel(const MeshModel &mesh);
	MeshModel(const char * file_path);
	MeshModel(std::string file_path);
	~MeshModel();

	QVector<Eigen::Vector3f> getVertices() const;
	QVector<Eigen::Vector3i> getFaces() const;
	QVector<Eigen::Vector3f> getVerticesNormals() const;
	QVector<Eigen::Vector3f> getFacesNormals() const;
	int vertexCount() const;
	int sampleCount() const;
	int faceCount();
	Eigen::Vector3f getCentroid() const;
	double getRadius() const { return m_radius; }
	void draw(int scale);
	void drawSamples(int scale);
	void samplePoints();
	std::string getInputFilepath() const;
	QVector<int> getVerticesLabels() const;
	QVector<int> getFacesLabels() const;
	QVector<int> getLabelNames() const;
	Parts_Samples getPartsSamples() const;
	Eigen::Vector3f getVertexNormal(int index);
	void rotate(float angle, float x, float y, float z);
	void output(const char *file_path);
	int numOfClasses();
	QVector<Eigen::Vector3f>::iterator vertices_begin() { return m_vertices_list.begin(); }
	QVector<Eigen::Vector3f>::iterator vertices_end() { return m_vertices_list.end(); }
	QVector<Eigen::Vector3f>::iterator vertices_normals_begin() { return m_vertices_normals.begin(); }
	QVector<Eigen::Vector3f>::iterator vertices_normals_end() { return m_vertices_normals.end(); }
	QVector<Eigen::Vector3i>::iterator faces_begin() { return m_faces_list.begin(); }
	QVector<Eigen::Vector3i>::iterator faces_end() { return m_faces_list.end(); }
	QVector<Eigen::Vector3f>::iterator faces_normals_begin() { return m_faces_normals.begin(); }
	QVector<Eigen::Vector3f>::iterator faces_normals_end() { return m_faces_normals.end(); }
	Parts_Samples::iterator samples_begin() { return m_parts_samples.begin(); }
	Parts_Samples::iterator samples_end() { return m_parts_samples.end(); }

private:
	QVector<Eigen::Vector3f> m_vertices_list;
	QVector<Eigen::Vector3f> m_vertices_normals;
	QVector<Eigen::Vector3i> m_faces_list;
	QVector<Eigen::Vector3f> m_faces_normals;
	Eigen::Vector3f m_centroid;
	QVector<int> m_vertices_labels;
	QVector<int> m_faces_labels;
	QVector<int> m_label_names;
	std::string m_input_filepath;
	Parts_Samples m_parts_samples;
	int m_sample_count;
	double m_radius;

	void load_from_file(const char * file_path);
	void normalize();
	
};

#endif // MESHMODEL_H
