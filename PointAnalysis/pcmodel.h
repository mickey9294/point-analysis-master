#ifndef PCMODEL_H
#define PCMODEL_H

#include <QObject>
#include <qopengl.h>
#include <QVector>
#include <QVector3D>
#include <QList>
#include <cstdlib>
#include <cmath>
#include <QMatrix4x4>
#include <QVector4D>
#include <fstream>
#include <iostream>
#include <vector>
#include <string>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include "Seb.h"

typedef Seb::Point<float> MiniPoint;
typedef std::vector<MiniPoint> PointVector;
typedef Seb::Smallest_enclosing_ball<float> Miniball;


/* Define 11 different colors */
const float COLORS[11][3] = {
	{ 1.0, 0.0, 0.0 },
	{ 0.0, 1.0, 0.0 },
	{ 0.0, 0.0, 1.0 },
	{ 1.0, 1.0, 0.0 },
	{ 0.0, 1.0, 1.0 },
	{ 1.0, 0.0, 1.0 },
	{ 0.5, 0.0, 0.5 },
	{ 1.0, 0.5, 0.25 },
	{ 0.5, 0.5, 0.0 },
	{ 0.0, 0.5, 0.5 },
	{ 0.5, 0.5, 0.5 }
};

class PCModel : public QObject
{
	Q_OBJECT

public:
	PCModel();
	PCModel(int nvertices, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::Normal>::Ptr cloud_normals);
	PCModel(int nvertices, QVector<float> data);
	PCModel(int nvertices, QVector<float> data, QVector<int> labels);
	~PCModel();

	const GLfloat *constData() const { return m_data.constData(); }
	GLfloat *data() { return m_data.data(); }
	int count() const { return m_count; }
	int vertexCount() const { return m_count / 9; }
	//void addFrame(PCModel model, QMatrix4x4 trans);
	QVector3D getCenter();
	void output(const char *filename);
	void clear();
	double getRadius() { return radius; }
	void setInputFilename(const char *name);
	void setInputFilename(std::string name);
	std::string getInputFilename();
	QVector<int> getLabels();
	QVector<double> getSdf();
	void setSdf(QVector<double> sdf);
	QList<int> getLabelNames();
	int numOfClasses();

	public slots:
	void setLabels(QVector<int> labels);
	void receiveSignalTest();

signals:
	void outputProgressReport(int progress);
	void onLabelsChanged();
	void addDebugText(QString text);

private:
	QVector<GLfloat> m_data;
	QVector<int> m_labels;
	int m_count;
	QVector3D center;  /* the center of the minimal bounding sphere */
	double max;
	double radius;  /* the radius of the minimal bounding sphere */
	std::string inputfilename;
	QVector<double> m_sdf;
	QList<int> m_label_names;

	void add(const QVector3D &v, const QVector3D &n, const QVector3D &c);
	void transform(QMatrix4x4 transMatrix);
	void normalize();
};

#endif // PCMODEL_H
