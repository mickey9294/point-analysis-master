#ifndef STRUCTUREANALYSER_H
#define STRUCTUREANALYSER_H

#include <QObject>
#include <QThread>
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
#include "energyfunctions.h"
#include "definitions.h"
#include "partssolver.h"
#include "PartsStructure.h"

#include "pointclassifier.h"
#include "candidatesgenerator.h"
#include "partspredictor.h"
#include "pointsegmentation.h"
#include "partssolver.h"

typedef boost::adjacency_list<boost::vecS, boost::vecS, boost::undirectedS> Graph;

class StructureAnalyser : public QThread
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

	void predict();

	public slots:
	void onDebugTextAdded(QString text);
	void initialize(PAPointCloud *pointcloud);
	void setOBBs(QVector<OBB *> obbs);

signals:
	void addDebugText(QString text);
	void sendOBBs(QVector<OBB *> obbs);
	void sendPartsStructure(Parts_Structure_Pointer structure);
	//void setPointcloudLabels(QVector<int> labels);

protected:
	void run();

private:
	std::string m_model_name;
	std::string m_modelClassName;

	PartsStructure m_parts_structure;

	PCModel * m_pcModel;
	QSharedPointer<PAPointCloud> m_pointcloud;
	QSharedPointer<FeatureEstimator> m_fe;

	QSharedPointer<EnergyFunctions> m_energy_functions;

	int m_iteration;
	int m_null_label;

	QSharedPointer<PointClassifier> m_point_classifier;
	QSharedPointer<CandidatesGenerator> m_candidates_generator;
	QSharedPointer<PartsPredictor> m_parts_predictor;
	QSharedPointer<PointSegmentation> m_point_segmentation;
	QSharedPointer<PartsSolver> m_parts_solver;

	QVector<int> m_label_names;

	std::list<std::string> m_object_list;
	std::vector< std::list<CuboidFeatures *> > m_feature_list;
	std::vector< std::list<CuboidTransformation *> > m_transformation_list;

	void classifyPoints(QVector<int> & pc_labels, QVector<QMap<int, float>> &distribution);
	void updateParts();  /* Update the points assignments to parts */
	void generateCandidates(const QVector<QMap<int, float>> & distribution, Part_Candidates & part_candidates, QVector<int> & point_cluster_map);
	void predictPartLabelsAndOrientations(const Part_Candidates & part_candidates, const QVector<int> & label_names,
		QMap<int, int> &parts_picked, std::vector<int> &candidate_labels);
	void segmentPoints();
	
	void get_joint_normal_relations(std::vector<std::vector<CuboidJointNormalRelations *>> &relations);
	bool load_object_list(const std::string &_filename);
	bool load_features(const std::string &_filename_prefix);
	bool load_transformations(const std::string &_filename_prefix);
};

#endif // STRUCTUREANALYSER_H
