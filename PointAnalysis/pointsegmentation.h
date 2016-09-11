#ifndef POINTSEGMENTATION_H
#define POINTSEGMENTATION_H

#include <QObject>
#include "PartsStructure.h"
#include <iostream>
#include <MRFEnergy.h>
#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>
#include "constants.h"

class PointSegmentation : public QObject
{
	Q_OBJECT

public:
	PointSegmentation(QObject *parent = 0);
	~PointSegmentation();
	
	int segmentPoints(PartsStructure & parts_structure, const QVector<int> & label_names, QVector<int> &point_assignments_segmented, bool first_run);
};

#endif // POINTSEGMENTATION_H
