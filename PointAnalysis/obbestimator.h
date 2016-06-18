#ifndef OBBESTIMATOR_H
#define OBBESTIMATOR_H

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <cmath>
#include "obb.h"
#include <pcl/common/pca.h>
#include <Eigen\Core>
#include <qvector.h>
#include <qdebug.h>
#include "pcmodel.h"

class OBBEstimator
{
public:
	OBBEstimator();
	OBBEstimator(int label, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
	~OBBEstimator();

	OBB * computeOBB();
	QVector<OBB *> computeOBBCandidates();
	void reset(int label, pcl::PointCloud <pcl::PointXYZ>::Ptr cloud);

private:
	pcl::PointCloud<pcl::PointXYZ>::Ptr m_cloud;
	int m_label;
};

#endif