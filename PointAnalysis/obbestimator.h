#ifndef OBBESTIMATOR_H
#define OBBESTIMATOR_H

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <iostream>
#include <cmath>
#include "obb.h"
#include <pcl/common/pca.h>
#include <Eigen\Core>
#include <qvector.h>
#include <qdebug.h>
#include "pcmodel.h"
#include "utils.h"
#include "ICP.h"

class OBBEstimator
{
public:
	OBBEstimator();
	OBBEstimator(int label, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
	~OBBEstimator();

	OBB * computeOBB();
	OBB * computeOBB_PCA();
	QVector<OBB *> computeOBBCandidates();
	void reset(int label, pcl::PointCloud <pcl::PointXYZ>::Ptr cloud);

private:
	pcl::PointCloud<pcl::PointXYZ>::Ptr m_cloud;
	int m_label;

	void ICP_procedure(OBB *obb);
};

#endif