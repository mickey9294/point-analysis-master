#ifndef POINTFEATURETHREAD_H
#define POINTFEATURETHREAD_H

#include <QThread>
#include <QDebug>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/search/kdtree.h>
#include <pcl/common/impl/centroid.hpp>
#include <Eigen/src/Core/MatrixBase.h>
#include <Eigen\src\Eigenvalues\EigenSolver.h>
#include "PAPoint.h"
#include "utils.h"

class PointFeatureThread : public QThread
{
	Q_OBJECT

public:
	PointFeatureThread(int super, int id, pcl::PointCloud<pcl::PointXYZ>::Ptr c, pcl::PointCloud<pcl::Normal>::Ptr n,
		pcl::search::KdTree<pcl::PointXYZ>::Ptr tree, float co, double rad, int start, int e, QObject *parent = 0);
	~PointFeatureThread();

signals:
	void estimateCompleted(int id, QList<QVector<double>> points_feats);
	void addDebugText(QString text);

protected:
	void run();

private:
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
	pcl::PointCloud<pcl::Normal>::Ptr normals;
	pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree;
	int begin;
	int end;
	int id;
	int superid;
	float coef;
	double radius;

	void estimate(QList<QVector<double>> &points_feats);
};

#endif // POINTFEATURETHREAD_H
