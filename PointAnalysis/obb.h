#ifndef OBB_H
#define OBB_H

#include <QObject>
#include <QVector3D>
#include <qvector.h>
#include "pcmodel.h"

class OBB : public QObject
{
	Q_OBJECT

public:
	OBB(QObject *parent = 0);
	OBB(QVector3D xAxis, QVector3D yAxis, QVector3D zAxis, QVector3D centroid, double xLength, double yLength, double zLength, int label, QObject *parent = 0);
	~OBB();

	QVector3D getXAxis();
	QVector3D getYAxis();
	QVector3D getZAxis();
	QVector3D getScale();
	QVector<QVector3D> getAxes();
	void triangulate();
	const float *constData() const { return m_data.constData(); }
	float *data() { return m_data.data(); }
	int count() const { return m_count; }
	int vertexCount() const { return m_count / 12 * 3; }
	int facetCount() const { return m_count / 12; }
	int getLabel() { return m_label; }
	QVector3D getColor() { return QVector3D(m_color); }
	QVector3D getCentroid() { return QVector3D(m_centroid); }

private:
	QVector3D x_axis;
	QVector3D y_axis;
	QVector3D z_axis;
	double x_length;
	double y_length;
	double z_length;
	QVector3D m_centroid;
	QVector3D m_color;
	QVector<float> m_data;
	int m_count;
	int m_label;

	void add(QVector3D v0, QVector3D v1, QVector3D v2, QVector3D normal);
};

#endif // OBB_H
