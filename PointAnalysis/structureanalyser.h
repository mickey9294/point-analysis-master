#ifndef STRUCTUREANALYSER_H
#define STRUCTUREANALYSER_H

#include <QObject>
#include <QList>
#include <QMap>
#include <QVector>
#include <string>
#include <QDebug>
#include <shark/Data/Csv.h>
#include <shark/Data/Dataset.h> //importing the file
#include <shark/Algorithms/Trainers/RFTrainer.h> //the random forest trainer
#include <fstream>
#include <Eigen\Core>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <boost\graph\adjacency_list.hpp>
#include <boost/graph/connected_components.hpp>
#include "pcmodel.h"
#include "utils.h"
#include "papart.h"
#include "PAPointCloud.h"
#include "PAPoint.h"
#include "featureestimator.h"
#include "obbestimator.h"
#include "testpcthread.h"
#include "gencandidatesthread.h"
#include "energyfunctions.h"
#include "predictionthread.h"

typedef boost::adjacency_list<boost::vecS, boost::vecS, boost::undirectedS> Graph;

class StructureAnalyser : public QObject
{
	Q_OBJECT

public:
	StructureAnalyser(PCModel *pcModel, QObject *parent = 0);
	StructureAnalyser(QObject *parent = 0);
	~StructureAnalyser();

	void execute();
	void setPointCloud(PCModel *pcModel);

	public slots:
	void onDebugTextAdded(QString text);
	void initialize(PAPointCloud *pointcloud);
	void onClassificationDone(QVector<QMap<int, float>> distribution);
	void onPointLabelsGot(QVector<int> labels);
	void onGenCandidatesDone(int num_of_candidates, Part_Candidates part_candidates);
	void onPredictionDone(QMap<int, int> part_picked);
	//void onPredictionDone();

signals:
	void addDebugText(QString text);
	void sendOBBs(QVector<OBB *> obbs);

private:
	std::string m_modelClassName;
	Part_Candidates m_parts_candidates;
	QList<int> m_label_names;
	PCModel * m_pcModel;
	PAPointCloud *m_pointcloud;
	FeatureEstimator * m_fe;
	TestPCThread *m_testPCThread;
	bool classifier_loaded;
	GenCandidatesThread *m_genCandThread;
	EnergyFunctions *m_energy_functions;
	PredictionThread *m_predictionThread;

	void classifyPoints(PAPointCloud *pointcloud);
	
};

#endif // STRUCTUREANALYSER_H
