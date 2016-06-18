#include "structureanalyser.h"

StructureAnalyser::StructureAnalyser(QObject *parent)
	: QObject(parent), m_fe(NULL), classifier_loaded(false), m_testPCThread(NULL), m_genCandThread(NULL), m_pointcloud(NULL)
{
	qRegisterMetaType<PAPointCloud *>("PAPointCloudPointer");
	qRegisterMetaType<QVector<QMap<int, float>>>("ClassificationDistribution");
	qRegisterMetaType<PAPart>("PAPart");
	qRegisterMetaType<Part_Candidates>("PartCandidates");
}

StructureAnalyser::StructureAnalyser(PCModel *pcModel, QObject * parent)
	: QObject(parent), m_fe(NULL), classifier_loaded(false), m_testPCThread(NULL), m_genCandThread(NULL), m_pointcloud(NULL)
{
	qRegisterMetaType<PAPointCloud *>("PAPointCloudPointer");
	qRegisterMetaType<QVector<QMap<int, float>>>("ClassificationDistribution");
	qRegisterMetaType<PAPart>("PAPart");
	qRegisterMetaType<Part_Candidates>("PartCandidates");
	m_pcModel = pcModel;
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

	if (m_pointcloud != NULL)
		delete(m_pointcloud);

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
	QString model_file_name = Utils::getModelName(QString::fromStdString(m_pcModel->getInputFilename()));
	std::string pcFile = "../data/features_test/" + model_file_name.toStdString() + ".csv";
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
	QString model_file_name = Utils::getModelName(QString::fromStdString(m_pcModel->getInputFilename()));
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
			GLfloat *point = m_pcModel->data() + i * 9;
			GLfloat x = point[0];
			GLfloat y = point[1];
			GLfloat z = point[2];
			papoint.setPosition(x, y, z);
			m_pointcloud->at(i) = papoint;
		}
		m_pointcloud->setRadius(m_pcModel->getRadius());
	}

	/* Create a thread to do the points classification */
	m_testPCThread = new TestPCThread(model_file_name, this);
	connect(m_testPCThread, SIGNAL(addDebugText(QString)), this, SLOT(onDebugTextAdded(QString)));
	connect(m_testPCThread, SIGNAL(classifyProbabilityDistribution(QVector<QMap<int, float>>)), this, SLOT(onClassificationDone(QVector<QMap<int, float>>)));
	connect(m_testPCThread, SIGNAL(setPCLabels(QVector<int>)), this, SLOT(onPointLabelsGot(QVector<int>)));
	m_testPCThread->start();
}

using namespace pcl;

void StructureAnalyser::onClassificationDone(QVector<QMap<int, float>> distribution)
{
	/* Create a thread to generate the part candidates */
	m_genCandThread = new GenCandidatesThread(m_pointcloud, distribution, this);
	connect(m_genCandThread, SIGNAL(addDebugText(QString)), this, SLOT(onDebugTextAdded(QString)));
	connect(m_genCandThread, SIGNAL(genCandidatesDone(int, Part_Candidates)), this, SLOT(onGenCandidatesDone(int, Part_Candidates)));
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

	Part_Candidates::iterator cand_it;
	for (cand_it = part_candidates.begin(); cand_it != part_candidates.end(); ++cand_it)
	{
		int label = cand_it.key();
		QVector<PAPart> candidates = cand_it.value();
		onDebugTextAdded("Part-" + QString::number(label) + " has totally " + QString::number(candidates.size()) + " candidates parts.");
		qDebug("Part-%d has totally %d candidates parts.", label, candidates.size());
	}
}

void StructureAnalyser::setPointCloud(PCModel *pcModel)
{
	if (m_testPCThread != NULL && m_testPCThread->isRunning())
		m_testPCThread->terminate();
	if (m_genCandThread != NULL && m_genCandThread->isRunning())
		m_genCandThread->terminate();
	
	m_pcModel = pcModel;
}