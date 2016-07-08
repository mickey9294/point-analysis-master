#ifndef GENCANDIDATESTHREAD_H
#define GENCANDIDATESTHREAD_H

#include <QThread>
#include <QVector>
#include <QMap>
#include <QList>
#include <QSet>
#include <QDebug>
#include <fstream>
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

typedef boost::adjacency_list<boost::vecS, boost::vecS, boost::undirectedS> Graph;
typedef QVector<PAPart> Part_Candidates;
Q_DECLARE_METATYPE(Part_Candidates)

class GenCandidatesThread : public QThread
{
	Q_OBJECT

public:
	GenCandidatesThread(QObject *parent = 0);
	GenCandidatesThread(int num_of_candidates, QObject *parent = 0);
	GenCandidatesThread(PAPointCloud *pointcloud, QVector<QMap<int, float>> distribution, QObject *parent = 0);
	~GenCandidatesThread();

	public slots:
	void onDebugTextAdded(QString text);
	
signals:
	void addDebugText(QString text);
	void genCandidatesDone(int num_of_candidates, Part_Candidates part_candidates);

protected:
	void run();

private:
	PAPointCloud * m_pointcloud;
	QVector<QMap<int, float>> m_distribution;
	int m_num_of_candidates;

	void generateCandidates();
	void loadCandidatesFromFiles();
};

#endif // GENCANDIDATESTHREAD_H
