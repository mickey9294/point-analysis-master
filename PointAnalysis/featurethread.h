#ifndef FEATURETHREAD_H
#define FEATURETHREAD_H

#include <QThread>
#include <QDebug>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/search/kdtree.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/common/impl/centroid.hpp>
#include <pcl/features/normal_3d.h>
#include <boost/smart_ptr/shared_ptr.hpp>
#include <QVector>
#include "PAPoint.h"
#include "utils.h"
#include "pointfeaturethread.h"
#include "constants.h"

class FeatureThread : public QThread
{
	Q_OBJECT

public:
	FeatureThread(int id, pcl::PointCloud<pcl::PointXYZ>::Ptr c, pcl::PointCloud<pcl::Normal>::Ptr n, 
		double radius, double coef, QObject *parent = 0);
	~FeatureThread();

	void setInputFilename(QString filename);

	public slots:
	void receiveFeatures(int id, QList<QVector<double>> points_feats);
	void nextEstimateStep();
	void onDebugTextAdded(QString text);

signals:
	void estimateCompleted(int id, QVector<QVector<double>> points_feats);
	void firstStepCompleted();
	void addDebugText(QString text);

protected:
	void run();

private:
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
	pcl::PointCloud<pcl::Normal>::Ptr normals;
	pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree;
	double radius;
	double coefficient;
	int id;
	QVector<QVector<double>> cloud_feats;
	QVector<PointFeatureThread *> subthreads;
	int finish_count;
	QString input_filename;

	void estimate();
};

#endif // FEATURETHREAD_H
