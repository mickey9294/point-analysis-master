#ifndef CANDIDATESGENERATOR_H
#define CANDIDATESGENERATOR_H

#include <QObject>
#include <iostream>
#include <QSharedPointer>
#include <fstream>
#include "definitions.h"
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <boost\graph\adjacency_list.hpp>
#include <boost/graph/connected_components.hpp>
#include "PAPointCloud.h"
#include "PAPoint.h"
#include "obbestimator.h"
#include "papart.h"
#include "utils.h"
#include "constants.h"

typedef boost::adjacency_list<boost::vecS, boost::vecS, boost::undirectedS> Graph;

class CandidatesGenerator : public QObject
{
	Q_OBJECT

public:
	CandidatesGenerator(QObject *parent = 0);
	~CandidatesGenerator();

	void generateCandidates(std::string model_name, QSharedPointer<PAPointCloud> pointcloud, 
		const QVector<QMap<int, float>> & distribution, Part_Candidates & part_candidates, QVector<int> & point_cluster_map);

signals:
	void setOBBs(QVector<OBB *> obbs);

private:
	int loadCandidatesFromFiles(std::string model_name, int npoints, Part_Candidates & part_candidates, QVector<int> & point_cluster_map);
	
};

#endif // CANDIDATESGENERATOR_H
