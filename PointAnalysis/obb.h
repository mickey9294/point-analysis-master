#ifndef OBB_H
#define OBB_H

#include <QObject>
#include <QVector3D>
#include <qvector.h>
#include <Eigen\Core>
#include "utils_sampling.hpp"
#include "pcmodel.h"
//#include "utils.h"

class OBB : public QObject
{
	Q_OBJECT

public:
	OBB(QObject *parent = 0);
	OBB(const OBB &obb);
	OBB(Eigen::Vector3f xAxis, Eigen::Vector3f yAxis, Eigen::Vector3f zAxis, Eigen::Vector3f centroid, double xLength, double yLength, double zLength, int label, QObject *parent = 0);
	~OBB();

	Eigen::Vector3f getXAxis() const;
	Eigen::Vector3f getYAxis() const;
	Eigen::Vector3f getZAxis() const;
	QVector3D getScale() const;
	QVector<Eigen::Vector3f> getAxes() const;
	void triangulate();
	const float *constData() const { return m_data.constData(); }
	float *data() { return m_data.data(); }
	int count() const { return m_count; }
	int vertexCount() const { return m_count / 12 * 3; }
	int facetCount() const { return m_count / 12; }
	int getLabel() const{ return m_label; }
	QVector3D getColor() const { return QVector3D(m_color); }
	Eigen::Vector3f getCentroid() const { return m_centroid; }
	QVector<QVector3D> getVertices() const;
	void setLabel(int label) { m_label = label; }
	void setColor(QVector3D color);
	int samplePoints(int num_of_samples = 0);
	QVector<Eigen::Vector3f> getSamplePoints() const;
	void setSamplePoints(Eigen::MatrixXf samples_mat);
	void setXAxis(Eigen::Vector3f xAxis);
	void setYAxis(Eigen::Vector3f yAxis);
	void setZAxis(Eigen::Vector3f zAxis);
	void setCentroid(Eigen::Vector3f centroid);
	void setXAxis(Eigen::Vector3d xAxis);
	void setYAxis(Eigen::Vector3d yAxis);
	void setZAxis(Eigen::Vector3d zAxis);
	void setCentroid(Eigen::Vector3d centroid);

private:
	Eigen::Vector3f x_axis;
	Eigen::Vector3f y_axis;
	Eigen::Vector3f z_axis;
	double x_length;
	double y_length;
	double z_length;
	Eigen::Vector3f m_centroid;
	QVector3D m_color;
	QVector<float> m_data;
	int m_count;
	int m_label;
	QVector<QVector3D> m_vertices;
	QVector<Eigen::Vector3f> m_sample_points;

	void add(QVector3D v0, QVector3D v1, QVector3D v2, Eigen::Vector3f normal);
	QVector3D eigen_vector3f_to_qvector3d(Eigen::Vector3f vec);
};

#endif // OBB_H
