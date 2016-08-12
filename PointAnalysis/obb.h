#ifndef OBB_H
#define OBB_H

#include <QObject>
#include <QVector3D>
#include <QtOpenGL>
#include <qvector.h>
#include <Eigen\Core>
#include <Eigen\Geometry>
#include <assert.h>
#include <utils_sampling.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include "model.h"
#include "utils.h"

typedef QVector<Eigen::Vector3f> Samples_Vec;
Q_DECLARE_METATYPE(Samples_Vec)

class OBB : public QObject
{
	Q_OBJECT

public:
	OBB(QObject *parent = 0);
	OBB(const OBB &obb);
	OBB(Eigen::Vector3f xAxis, Eigen::Vector3f yAxis, Eigen::Vector3f zAxis, Eigen::Vector3f centroid, 
		double xLength, double yLength, double zLength, int label, QObject *parent = 0);
	~OBB();

	Eigen::Vector3f getXAxis() const;
	Eigen::Vector3f getYAxis() const;
	Eigen::Vector3f getZAxis() const;
	QVector3D getScale() const;
	Eigen::Matrix3f getAxes() const;
	Eigen::Matrix<float, 4, 3> getAugmentedAxes();
	void triangulate();
	int vertexCount() const;
	int faceCount() const;
	int sampleCount() const;
	int getLabel() const{ return m_label; }
	QVector3D getColor() const { return QVector3D(m_color); }
	Eigen::Vector3f getCentroid() const { return m_centroid; }
	QVector<Eigen::Vector3f> getVertices() const;
	QVector<Eigen::Vector3i> getFaces() const;
	QVector<Eigen::Vector3f> getFacesNormals() const;
	QVector<Eigen::Vector3f> getSamples() const;
	void setLabel(int label) { m_label = label; }
	void setColor(QVector3D color);
	int samplePoints(int num_of_samples = 0);
	QVector<Eigen::Vector3f> getSamplePoints() const;
	void setSamplePoints(Eigen::MatrixXd samples_mat);
	void setSamplePoints(pcl::PointCloud<pcl::PointXYZ> cloud);
	void setXAxis(Eigen::Vector3f xAxis);
	void setYAxis(Eigen::Vector3f yAxis);
	void setZAxis(Eigen::Vector3f zAxis);
	void setCentroid(Eigen::Vector3f centroid);
	void setXAxis(Eigen::Vector3d xAxis);
	void setYAxis(Eigen::Vector3d yAxis);
	void setZAxis(Eigen::Vector3d zAxis);
	void setCentroid(Eigen::Vector3d centroid);
	void draw(int scale);
	void drawSamples(int scale);
	void translate(float x, float y, float z);
	void rotate(float angle, float x, float y, float z);
	void rotate(float angle, float x, float y, float z, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
	void rotate(Eigen::Matrix3d rotate_mat, Eigen::Vector3d translate_vec, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
	void transform(Eigen::Matrix4f transform_mat, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
	QVector<Eigen::Vector3f>::iterator samples_begin();
	QVector<Eigen::Vector3f>::iterator samples_end();
	void normalize(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);  /* used to adjust the obb to have the consensus orientation in training process */

private:
	Eigen::Vector3f x_axis;
	Eigen::Vector3f y_axis;
	Eigen::Vector3f z_axis;
	double x_length;
	double y_length;
	double z_length;
	Eigen::Vector3f m_centroid;
	QVector3D m_color;
	//QVector<float> m_data;
	//int m_count;
	int m_label;
	QVector<Eigen::Vector3f> m_vertices;
	QVector<Eigen::Vector3i> m_faces;
	QVector<Eigen::Vector3f> m_faces_normals;
	QVector<Eigen::Vector3f> m_sample_points;

	//void add(QVector3D v0, QVector3D v1, QVector3D v2, Eigen::Vector3f normal);
	QVector3D eigen_vector3f_to_qvector3d(Eigen::Vector3f vec);
	void drawLine(int vertex_no_0, int vertex_no_1, float scale);
};

#endif // OBB_H
