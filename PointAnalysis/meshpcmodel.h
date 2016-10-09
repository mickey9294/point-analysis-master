#ifndef MESHPCMODEL_H
#define MESHPCMODEL_H

#include <QObject>
#include "model.h"
#include <string>
#include <qvector.h>
#include <QtOpenGL>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include "SamplePoint.h"
#include <iostream>
#include <fstream>
#include <RNScalar.h>
#include "utils.h"

class MeshPcModel : public QObject, public Model
{
	Q_OBJECT

public:
	MeshPcModel(QObject *parent = 0);
	MeshPcModel(const MeshPcModel &other);
	MeshPcModel(const char * file_path, QObject *parent = 0);
	MeshPcModel(std::string file_path, QObject *parent = 0);
	~MeshPcModel();

	void draw(float scale);
	void rotate(float angle, float x, float y, float z);
	std::string getInputFilepath() const;
	QVector<Eigen::Vector3f> getVertices() const;
	QVector<Eigen::Vector3i> getFaces() const;
	Eigen::Vector3f getVertexNormal(int index);
	QVector<Eigen::Vector3f> getVerticesNormals() const;
	QVector<Eigen::Vector3f> getFacesNormals() const;
	QVector<SamplePoint> getSamples() const;
	int vertexCount() const;
	int faceCount() const;
	int sampleCount() const;
	QVector<int> getVerticesLabels() const;
	QVector<int> getLabelNames() const;
	void output(const char *file_path);
	Eigen::Vector3f getCentroid() const;
	int numOfClasses();
	double getRadius() const;
	void samplePoints();
	double faceArea(int index);

	QVector<SamplePoint>::iterator samples_begin();
	QVector<SamplePoint>::iterator samples_end();

	public slots:
	void setLabels(QVector<int> labels);

signals:
	void onLabelsChanged();

private:
	QVector<Eigen::Vector3f> m_vertices_list;
	QVector<Eigen::Vector3i> m_faces_list;
	QVector<Eigen::Vector3f> m_vertices_normals;
	QVector<Eigen::Vector3f> m_faces_normals;
	Eigen::Vector3f m_centroid;
	QVector<int> m_samples_labels;
	QVector<int> m_label_names;
	std::string m_input_filepath;
	QVector<SamplePoint> m_samples;
	double m_radius;

	void load_from_file(const char * file_path);
	double TriangleArea(const Eigen::Vector3f &pA, const Eigen::Vector3f &pB, const Eigen::Vector3f &pC);
	void createRandom(int nPnts);
	void drawLine(int vertex_no_0, int vertex_no_1, float scale);
	void drawSamples(float scale);
};

#endif // MESHPCMODEL_H
