#include "pointfeatureextractor.h"

PointFeatureExtractor::PointFeatureExtractor(QObject *parent)
	: QObject(parent), currentId(0)
{
	qRegisterMetaType<PAPointCloud *>("PAPointCloudPointer");
	qRegisterMetaType<PCModel *>("PCModelPointer");

	m_modelClassName = "coseg_chairs_3";

	connect(&loadThread, SIGNAL(loadPointsCompleted(PCModel *)), this, SLOT(receiveModel(PCModel *)));
	connect(&fe, SIGNAL(estimateCompleted(PAPointCloud *)), this, SLOT(oneEstimateCompleted(PAPointCloud *)));
	connect(&loadThread, SIGNAL(addDebugText(QString)), this, SLOT(onDebugTextAdded(QString)));
	connect(&fe, SIGNAL(addDebugText(QString)), this, SLOT(onDebugTextAdded(QString)));
	fe.setPhase(FeatureEstimator::PHASE::TRAINING);
	loadThread.setPhase(LoadThread::PHASE::TRAINING);
}

PointFeatureExtractor::PointFeatureExtractor(std::string modelClassName, QObject *parent)
	: QObject(parent), currentId(0), m_modelClassName(modelClassName)
{
	qRegisterMetaType<PAPointCloud *>("PAPointCloudPointer");
	qRegisterMetaType<PCModel *>("PCModelPointer");

	connect(&loadThread, SIGNAL(loadPointsCompleted(PCModel *)), this, SLOT(receiveModel(PCModel *)));
	connect(&fe, SIGNAL(estimateCompleted(PAPointCloud *)), this, SLOT(oneEstimateCompleted(PAPointCloud *)));
	connect(&loadThread, SIGNAL(addDebugText(QString)), this, SLOT(onDebugTextAdded(QString)));
	connect(&fe, SIGNAL(addDebugText(QString)), this, SLOT(onDebugTextAdded(QString)));
	fe.setPhase(FeatureEstimator::PHASE::TRAINING);
	loadThread.setPhase(LoadThread::PHASE::TRAINING);
}

PointFeatureExtractor::~PointFeatureExtractor()
{

}

void PointFeatureExtractor::execute()
{
	estimateFeatures();
}

using namespace std;
void PointFeatureExtractor::estimateFeatures()
{
	/* Specify the list of models to be estimated */
	std::string filelist = "../data/" + m_modelClassName + "_list.txt";
	ifstream in(filelist.c_str());
	if (in.is_open())
	{
		char buffer[256];
		while (!in.eof())
		{
			in.getline(buffer, 256);
			string file(buffer);
			if (file.length() > 0)
				fileList.push_back(file);
		}
		in.close();
	}

	/* Load 1st model from file */
	loadThread.setLoadFileName(fileList.at(currentId));
	QString stat_msg = "Loading model " + getModelName(currentId) + "...";
	emit reportStatus(stat_msg);
	emit addDebugText(stat_msg);
	loadThread.start();
}

string PointFeatureExtractor::getOutFilename(int index)
{
	string filename = "..\\data\\features\\" + m_modelClassName + "\\" + getModelName(index).toStdString() + ".csv";
	return filename;
}

QString PointFeatureExtractor::getModelName(int index)
{
	QStringList namelist = QString::fromStdString(fileList.at(index)).split("\\");
	int size = namelist.size();
	QString modelname = namelist[size - 3] + "_" + namelist[size - 1].section('.', 0, 0);
	return modelname;
}

void PointFeatureExtractor::receiveModel(PCModel *pcModel)
{
	QString modelname = getModelName(currentId);
	QString stat_msg = "Loading " + modelname + " done.";
	qDebug() << stat_msg;
	emit addDebugText(stat_msg);
	std::string file = fileList.at(currentId);
	pcModel->setInputFilename(file);
	emit showModel(pcModel);

	/* Check the label names, add the new label names to the list m_label_names */
	QList<int> label_names = pcModel->getLabelNames();
	QList<int>::iterator it;
	for (it = label_names.begin(); it != label_names.end(); it++)
	{
		if (!m_label_names.contains(*it))
			m_label_names.push_back(*it);
	}

	/* Set point cloud to be estimated to pcModel */
	fe.reset(pcModel);
	stat_msg = "Estimating Features of " + modelname + "...";
	emit reportStatus(stat_msg);
	emit addDebugText(stat_msg);
	/* Estimate point features */
	fe.estimateFeatures();
}

void PointFeatureExtractor::oneEstimateCompleted(PAPointCloud *cloud)
{
	/* Output the point features of the model into file */
	cloud->writeToFile(getOutFilename(currentId).c_str());
	QString stat_msg = "Model " + getModelName(currentId) + " estimation done.";
	qDebug() << stat_msg;
	emit reportStatus(stat_msg);
	emit addDebugText(stat_msg);

	/* Postprocess */
	currentId++;
	delete(cloud);

	if (currentId < fileList.size())    /* If there are models remain unestimated */
	{
		/* If loadThread is running, terminate it */
		if (loadThread.isRunning())
			loadThread.terminate();

		loadThread.setLoadFileName(fileList.at(currentId));
		stat_msg = "Loading model " + getModelName(currentId) + "...";
		emit reportStatus(stat_msg);
		emit addDebugText(stat_msg);
		loadThread.start();
	}
	else    /* If it is the last model */
	{
		emit reportStatus("Point features estimation done.");

		/* Sort the label names in m_label_names */
		qSort(m_label_names);

		/* Write the label names to local file */
		std::string label_names_path = "../data/label_names/" + m_modelClassName + "_labelnames.txt";
		ofstream label_names_out(label_names_path.c_str());

		if (label_names_out.is_open())
		{
			QList<int>::iterator it;
			for (it = m_label_names.begin(); it != m_label_names.end(); ++it)
				label_names_out << *it << endl;

			label_names_out.close();
		}
	}
}

void PointFeatureExtractor::onDebugTextAdded(QString text)
{
	emit addDebugText(text);
}