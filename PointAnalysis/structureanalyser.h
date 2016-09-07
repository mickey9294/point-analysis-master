#ifndef STRUCTUREANALYSER_H
#define STRUCTUREANALYSER_H

#include <QObject>
#include <QList>
#include <QMap>
#include <QVector>
#include <string>
#include <QDebug>
#include <qsharedpointer.h>
#include <shark/Data/Csv.h>
#include <qfileinfo.h>
#include <shark/Data/Dataset.h> //importing the file
#include <shark/Algorithms/Trainers/RFTrainer.h> //the random forest trainer
#include <fstream>
#include <Eigen\Core>
#include <assert.h>
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
#include "pointsegmentationthread.h"
#include "definitions.h"
#include "partssolver.h"
#include "PartsStructure.h"
#include "partposeoptthread.h"

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
	int numOfLabels();
	int getLabelName(const std::string label_paraphrase);

	public slots:
	void onDebugTextAdded(QString text);
	void initialize(PAPointCloud *pointcloud);
	void onClassificationDone(QVector<QMap<int, float>> distribution);
	void onPointLabelsGot(QVector<int> labels);
	void onGenCandidatesDone(int num_of_candidates, Part_Candidates part_candidates, QVector<int> point_cluster_map);
	void onPredictionDone(QMap<int, int> part_picked, std::vector<int> candidate_labels);
	void setOBBs(QVector<OBB *> obbs);
	void pointSegmentationDone(QVector<int> new_point_assignments);
	void onPartPoseOptimizationDone();
	//void onPredictionDone();

signals:
	void addDebugText(QString text);
	void sendOBBs(QVector<OBB *> obbs);
	void sendPartsStructure(PartsStructure *structure);

private:
	std::string m_model_name;
	std::string m_modelClassName;
	//Parts_Vector m_parts;
	PartsStructure m_parts_structure;
	Part_Candidates m_parts_candidates;
	//QVector<int> m_point_assignments;
	PCModel * m_pcModel;
	PAPointCloud *m_pointcloud;
	QSharedPointer<FeatureEstimator> m_fe;
	QSharedPointer<TestPCThread> m_testPCThread;
	bool classifier_loaded;
	QSharedPointer<GenCandidatesThread> m_genCandThread;
	QSharedPointer<EnergyFunctions> m_energy_functions;
	QSharedPointer<PredictionThread> m_predictionThread;
	QSharedPointer<PointSegmentationThread> m_segmentationThread;
	QSharedPointer<PartPoseOptThread> m_partPoseOptThread;
	int m_iteration;
	int m_null_label;

	QVector<int> m_label_names;
	QSharedPointer<CuboidJointNormalRelationPredictor> m_joint_normal_predictor;

	std::list<std::string> m_object_list;
	std::vector< std::list<CuboidFeatures *> > m_feature_list;
	std::vector< std::list<CuboidTransformation *> > m_transformation_list;

	void classifyPoints(PAPointCloud *pointcloud);
	void updateParts();  /* Update the points assignments to parts */
	
	void get_joint_normal_relations(std::vector<std::vector<CuboidJointNormalRelations *>> &relations);
	bool load_object_list(const std::string &_filename);
	bool load_features(const std::string &_filename_prefix);
	bool load_transformations(const std::string &_filename_prefix);
};

#endif // STRUCTUREANALYSER_H
