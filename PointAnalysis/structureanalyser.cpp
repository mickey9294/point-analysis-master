#include "structureanalyser.h"

StructureAnalyser::StructureAnalyser(QObject *parent)
	: QObject(parent), m_fe(NULL), classifier_loaded(false), m_testPCThread(NULL), m_genCandThread(NULL), m_pointcloud(NULL),
	m_predictionThread(NULL)
{
	qRegisterMetaType<PAPointCloud *>("PAPointCloudPointer");
	qRegisterMetaType<QVector<QMap<int, float>>>("ClassificationDistribution");
	qRegisterMetaType<PAPart>("PAPart");
	qRegisterMetaType<Part_Candidates>("PartCandidates");

	m_modelClassName = "coseg_chairs_8";
	m_energy_functions = new EnergyFunctions(m_modelClassName);
}

StructureAnalyser::StructureAnalyser(PCModel *pcModel, QObject * parent)
	: QObject(parent), m_fe(NULL), classifier_loaded(false), m_testPCThread(NULL), m_genCandThread(NULL), m_pointcloud(NULL),
	m_predictionThread(NULL)
{
	qRegisterMetaType<PAPointCloud *>("PAPointCloudPointer");
	qRegisterMetaType<QVector<QMap<int, float>>>("ClassificationDistribution");
	qRegisterMetaType<PAPart>("PAPart");
	qRegisterMetaType<Part_Candidates>("PartCandidates");
	m_pcModel = pcModel;
	m_modelClassName = "coseg_chairs_8";
	m_energy_functions = new EnergyFunctions(m_modelClassName);
}

StructureAnalyser::~StructureAnalyser()
{
	if (m_fe != NULL)
	{
		delete(m_fe);
		m_fe = NULL;
	}

	if (m_testPCThread != NULL)
	{
		if (m_testPCThread->isRunning())
			m_testPCThread->terminate();
		delete(m_testPCThread);
		m_testPCThread = NULL;
	}

	if (m_genCandThread != NULL)
	{
		if (m_genCandThread->isRunning())
			m_genCandThread->terminate();
		delete(m_genCandThread);
		m_genCandThread = NULL;
	}

	if (m_predictionThread != NULL)
	{
		if (m_predictionThread->isRunning())
			m_predictionThread->terminate();
		delete(m_predictionThread);
		m_predictionThread = NULL;
	}

	if (m_pointcloud != NULL)
		delete(m_pointcloud);

	delete(m_energy_functions);
}

void StructureAnalyser::execute()
{
	if (m_fe != NULL)
	{
		delete(m_fe);
		m_fe = NULL;
	}

	/* Make sure the last run of TestPCThread is over */
	if (m_testPCThread != NULL && m_testPCThread->isRunning())
		m_testPCThread->terminate();

	/* Make sure the last run of GenCandidatesThread is over */
	if (m_genCandThread != NULL)
	{
		if (m_genCandThread->isRunning())
			m_genCandThread->terminate();
		delete(m_genCandThread);
		m_genCandThread = NULL;
	}

	if (m_pointcloud != NULL)
	{
		delete(m_pointcloud);
		m_pointcloud = NULL;
	}
	
	/* Check if the point cloud featrues have been estimated before */
	QString model_file_name = Utils::getModelName(QString::fromStdString(m_pcModel->getInputFilepath()));
	m_model_name = model_file_name.toStdString();
	std::string pcFile = "../data/features_test/" + m_model_name + ".csv";
	std::ifstream feat_file_in(pcFile.c_str());
	if (feat_file_in.is_open())    /* If there is already a features file of the point cloud */
	{
		initialize(NULL);
		feat_file_in.close();
	}
	else    /* If the point cloud has not been estimated, then estimate it */
	{
		m_fe = new FeatureEstimator(m_pcModel, FeatureEstimator::PHASE::TESTING, this);
		connect(m_fe, SIGNAL(addDebugText(QString)), this, SLOT(onDebugTextAdded(QString)));
		connect(m_fe, SIGNAL(estimateCompleted(PAPointCloud *)), this, SLOT(initialize(PAPointCloud *)));
		onDebugTextAdded("Estimating points features...");
		m_fe->estimateFeatures();
	}
}

void StructureAnalyser::onDebugTextAdded(QString text)
{
	emit addDebugText(text);
}

void StructureAnalyser::initialize(PAPointCloud *pointcloud)
{
	onDebugTextAdded("Points features estimation done.");
	onDebugTextAdded("Start initialization.");
	qDebug() << "Points features estimation done. Start initialization.";

	classifyPoints(pointcloud);
}

using namespace shark;
using namespace std;
void StructureAnalyser::classifyPoints(PAPointCloud *pointcloud)
{
	QString model_file_name = Utils::getModelName(QString::fromStdString(m_pcModel->getInputFilepath()));
	std::string pcFile = "../data/features_test/" + model_file_name.toStdString() + ".csv";

	if (pointcloud != NULL)    /* If there exists no features file of the point cloud */
	{
		m_pointcloud = pointcloud;
		onDebugTextAdded("Classify each point to a certain part label.");
		qDebug() << "Classify each point to a certain part label.";
		onDebugTextAdded("Load points features.");
		qDebug() << "Load points features.";

		/* Output the PAPointCloud to local file */
		pointcloud->writeToFile(pcFile.c_str());
	}
	else    /* If there already exists a features file of the point cloud */
	{
		ifstream features_in(pcFile.c_str());
		int nvertices = m_pcModel->vertexCount();
		m_pointcloud = new PAPointCloud(nvertices);

		char buffer[511];
		for (int i = 0; i < nvertices; i++)
		{
			features_in.getline(buffer, 511);
			QStringList line_data = QString(buffer).split(',');
			double feats[DIMEN];
			for (int j = 0; j < DIMEN; j++)
				feats[j] = line_data[j].toDouble();
			PAPoint papoint(feats);
			Eigen::Vector3f point = m_pcModel->operator[](i);
			GLfloat x = point[0];
			GLfloat y = point[1];
			GLfloat z = point[2];
			papoint.setPosition(x, y, z);
			m_pointcloud->at(i) = papoint;
		}
		m_pointcloud->setRadius(m_pcModel->getRadius());
	}

	/* Set the point cloud to EnergyFunctions object */
	m_energy_functions->setPointCloud(m_pointcloud);

	/* Create a thread to do the points classification */
	/* Check whether the point cloud has been classified */
	std::string prediction_path = "../data/predictions/" + m_model_name + ".txt";
	ifstream prediction_in(prediction_path.c_str());

	if (prediction_in.is_open())    /* If the point cloud has been classified */
	{
		prediction_in.close();
		if (m_testPCThread == NULL)
		{
			m_testPCThread = new TestPCThread(0, prediction_path, m_modelClassName, this);
			connect(m_testPCThread, SIGNAL(addDebugText(QString)), this, SLOT(onDebugTextAdded(QString)));
			connect(m_testPCThread, SIGNAL(classifyProbabilityDistribution(QVector<QMap<int, float>>)), this, SLOT(onClassificationDone(QVector<QMap<int, float>>)));
			connect(m_testPCThread, SIGNAL(setPCLabels(QVector<int>)), this, SLOT(onPointLabelsGot(QVector<int>)));
		}
		else
			m_testPCThread->setPredictionFilePath(prediction_path);
	}
	else    /* If the point cloud has not been classified */
	{
		if (m_testPCThread == NULL)
		{
			m_testPCThread = new TestPCThread(QString::fromStdString(m_model_name), m_modelClassName, this);
			connect(m_testPCThread, SIGNAL(addDebugText(QString)), this, SLOT(onDebugTextAdded(QString)));
			connect(m_testPCThread, SIGNAL(classifyProbabilityDistribution(QVector<QMap<int, float>>)), this, SLOT(onClassificationDone(QVector<QMap<int, float>>)));
			connect(m_testPCThread, SIGNAL(setPCLabels(QVector<int>)), this, SLOT(onPointLabelsGot(QVector<int>)));
		}
		else
			m_testPCThread->setPcName(QString::fromStdString(m_model_name));
	}

	m_testPCThread->start();
}

using namespace pcl;

void StructureAnalyser::onClassificationDone(QVector<QMap<int, float>> distribution)
{
	m_label_names = distribution[0].keys();
	/* Set the classification probability distribution to EnergyFunctions object */
	m_energy_functions->setDistributions(distribution);

	/* Create a thread to generate the part candidates */
	m_genCandThread = new GenCandidatesThread(m_pointcloud, m_model_name, distribution, this);
	//m_genCandThread = new GenCandidatesThread(m_model_name, 144, this);    /* Directly load candidates from local files */
	connect(m_genCandThread, SIGNAL(addDebugText(QString)), this, SLOT(onDebugTextAdded(QString)));
	connect(m_genCandThread, SIGNAL(genCandidatesDone(int, Part_Candidates)), this, SLOT(onGenCandidatesDone(int, Part_Candidates)));
	connect(m_genCandThread, SIGNAL(setOBBs(QVector<OBB *>)), this, SLOT(setOBBs(QVector<OBB *>)));
	m_genCandThread->start();
}

void StructureAnalyser::onPointLabelsGot(QVector<int> labels)
{
	m_pcModel->setLabels(labels);
}

void StructureAnalyser::onGenCandidatesDone(int num_of_candidates, Part_Candidates part_candidates)
{
	onDebugTextAdded("Genarating parts candidates has finished.");
	qDebug() << "Generating parts candidates has finished.";

	m_parts_candidates = part_candidates;

	onDebugTextAdded("There are " + QString::number(part_candidates.size()) + " part candidates in total.");
	qDebug("Threre are %d part candidates in total.", part_candidates.size());

	/* Do part labels and orientations prediction */
	onDebugTextAdded("Predict part labels and orientations.");
	qDebug() << "Predict part labels and orientations.";

	m_predictionThread = new PredictionThread(m_energy_functions, part_candidates, m_label_names, this);
	connect(m_predictionThread, SIGNAL(predictionDone(QMap<int, int>)), this, SLOT(onPredictionDone(QMap<int, int>)));
	//connect(m_predictionThread, SIGNAL(predictionDone()), this, SLOT(onPredictionDone()));
	m_predictionThread->start();
}

void StructureAnalyser::onPredictionDone(QMap<int, int> parts_picked)
{
	qDebug() << "Part labels and orientations prediction done.";

	int numLabels = m_label_names.size();
	QVector<OBB *> obbs(parts_picked.size());
	int i = 0;
	for (QMap<int, int>::iterator it = parts_picked.begin(); it != parts_picked.end(); ++it)
	{
		int label = it.key();
		int candidate_idx = it.value();
		PAPart part = m_parts_candidates[candidate_idx];
		OBB * obb = part.generateOBB();
		obb->setColor(QVector3D(COLORS[label][0], COLORS[label][1], COLORS[label][2]));
		obbs[i++] = obb;
	}

	emit sendOBBs(obbs);
}

//void StructureAnalyser::onPredictionDone()
//{
//	qDebug() << "onPredictionDone().";
//}


void StructureAnalyser::setPointCloud(PCModel *pcModel)
{
	if (m_testPCThread != NULL && m_testPCThread->isRunning())
		m_testPCThread->terminate();
	if (m_genCandThread != NULL && m_genCandThread->isRunning())
		m_genCandThread->terminate();
	
	m_pcModel = pcModel;
}

void StructureAnalyser::setOBBs(QVector<OBB *> obbs)
{
	qDebug() << "StructureAnalyser::setOBBs()";
	emit sendOBBs(obbs);
}