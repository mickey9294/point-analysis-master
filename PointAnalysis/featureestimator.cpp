#include "featureestimator.h"

FeatureEstimator::FeatureEstimator(QObject *parent)
	: QObject(parent)
{
	m_cloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
	m_normals = pcl::PointCloud<pcl::Normal>::Ptr(new pcl::PointCloud < pcl::Normal>);
}

FeatureEstimator::FeatureEstimator(PCModel *pcModel, PHASE phase, QObject *parent)
	: QObject(parent), finish_count(NUM_OF_THREADS), m_phase(phase)
{
	qDebug() << "Initializing the feature estimator...";
	emit addDebugText("Initializing the feature estimator...");
	m_cloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
	m_normals = pcl::PointCloud<pcl::Normal>::Ptr(new pcl::PointCloud < pcl::Normal>);
	m_pointcloudFile = QString(pcModel->getInputFilename().c_str());
	m_pointcloud = new PAPointCloud(pcModel->vertexCount());
	m_pointcloud->setRadius(pcModel->getRadius());

	qDebug() << "Before initialization, the size of cloud is" << m_cloud->size();
	qDebug("The cloud should have %d points.", pcModel->vertexCount());
	for (int i = 0; i < pcModel->vertexCount(); i++)
	{
		int gap = i * 9;
		GLfloat *point = pcModel->data();
		float x = point[gap + 0];
		float y = point[gap + 1];
		float z = point[gap + 2];
		float nx = point[gap + 3];
		float ny = point[gap + 4];
		float nz = point[gap + 5];
	
		pcl::PointXYZ p(x, y, z);
		m_cloud->push_back(p);
		pcl::Normal normal(nx, ny, nz);
		m_normals->push_back(normal);
		m_pointcloud->at(i).setPosition(x, y, z);
	}

	m_radius = pcModel->getRadius();
	m_points_labels = pcModel->getLabels();
	m_sdf = pcModel->getSdf();

	qDebug() << "After initialization, the size of cloud is" << m_cloud->size();

	qRegisterMetaType<QVector<QVector<double>>>("FeatureVector");
	qDebug() << "Initialization done.";
	emit addDebugText("Initialization done.");
}

void FeatureEstimator::reset(PCModel *pcModel)
{
	int num_of_threads = m_subthreads.size();
	for (int i = 0; i < num_of_threads; i++)
	{
		if (m_subthreads[i] != NULL)
		{
			if (m_subthreads[i]->isRunning())
				m_subthreads[i]->terminate();
			delete(m_subthreads[i]);
			m_subthreads[i] = NULL;
		}
	}
	m_subthreads.clear();
	if (m_cloud->size() > 0)
		m_cloud->clear();
	if (m_normals->size())
		m_normals->clear();

	qDebug() << "Reset the feature estimator...";
	emit addDebugText("Reset the feature estimator...");
	m_cloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
	m_normals = pcl::PointCloud<pcl::Normal>::Ptr(new pcl::PointCloud < pcl::Normal>);
	m_pointcloudFile = QString::fromStdString(pcModel->getInputFilename());
	m_pointcloud = new PAPointCloud(pcModel->vertexCount());

	for (int i = 0; i < pcModel->vertexCount(); i++)
	{
		int gap = i * 9;
		GLfloat *point = pcModel->data();
		float x = point[gap + 0];
		float y = point[gap + 1];
		float z = point[gap + 2];
		float nx = point[gap + 3];
		float ny = point[gap + 4];
		float nz = point[gap + 5];

		pcl::PointXYZ p(x, y, z);
		m_cloud->push_back(p);
		pcl::Normal normal(nx, ny, nz);
		m_normals->push_back(normal);
		m_pointcloud->at(i).setPosition(x, y, z);
	}

	m_radius = pcModel->getRadius();
	m_points_labels = pcModel->getLabels();
	m_sdf = pcModel->getSdf();

	finish_count = NUM_OF_THREADS;
	qDebug() << "Resetting done.";
	emit addDebugText("Resetting done.");
}

FeatureEstimator::~FeatureEstimator()
{
	int num_of_threads = m_subthreads.size();
	for (int i = 0; i < num_of_threads; i++)
	{
		if (m_subthreads[i] != NULL)
		{
			if (m_subthreads[i]->isRunning())
				m_subthreads[i]->terminate();
			delete(m_subthreads[i]);
			m_subthreads[i] = NULL;
		}
	}
	m_subthreads.clear();
}

void FeatureEstimator::estimateFeatures()
{
	finish_count = NUM_OF_THREADS;
	qDebug() << "Estimating the point features...";
	emit addDebugText("Estimating the point features...");
	/* Create subthread to estimate point features in 5 different search radius */
	qDebug() << "Create 6 FeatureThreads, each of which esitmate the point features in a particular search radius.";
	emit addDebugText("Create 6 FeatureThreads, each of which esitmate the point features in a particular search radius.");

	for (int i = 0; i < NUM_OF_THREADS; i++)
	{
		double coefficient = 0.1 * (i + 1);
		FeatureThread * thread = new FeatureThread(i, m_cloud, m_normals, m_radius, coefficient, this);
		connect(thread, SIGNAL(estimateCompleted(int, QVector<QVector<double>>)), this, SLOT(receiveFeatures(int, QVector<QVector<double>>)));
		connect(thread, SIGNAL(addDebugText(QString)), this, SLOT(onDebugTextAdded(QString)));
		/* If it is the thread computing sdf values, then sent the filename of the point cloud to it */
		if (m_phase == PHASE::TRAINING && i == NUM_OF_THREADS - 1)
			thread->setInputFilename(m_pointcloudFile);

		m_subthreads.push_back(thread);
		thread->start();
	}
}

void FeatureEstimator::receiveFeatures(int sid, QVector<QVector<double>> points_feats)
{
	qDebug("Receive features from FeatureThread-%d", sid);
	QString dtext = "Receive features from FeatureThread-" + QString::number(sid);
	emit addDebugText(dtext);

	int size = points_feats.size();
	
	if (sid < NUM_OF_THREADS - 1)   /* It is the thread which computes features based on neighborhood */
	{
		for (int i = 0; i < size; i++)
		{
			QVector<double> feat = points_feats[i];
			if (feat.size() >= 5)    
				m_pointcloud->at(i).setFeatures(sid, feat);
			
			//qDebug() << pointcloud[i].toString().c_str();
		}
	}
	else    /* It is the thread which computes height and sdf */
	{
		if (m_points_labels.size() > 0){
			const int BUFFER_SIZE = 10;
			char buffer[BUFFER_SIZE];
			for (int i = 0; i < size; i++)
			{
				/* Set the height and sdf for each point */
				QVector<double> feat = points_feats[i];
				m_pointcloud->at(i).setHeight(feat[0]);
				if (m_phase == PHASE::TRAINING)
					m_pointcloud->at(i).setSdf(m_sdf[i]);
				else
					m_pointcloud->at(i).setSdf(feat[1]);

				/* Set part label for each point */
				m_pointcloud->at(i).setLabel(m_points_labels[i]);
			}
		}
		else
		{
			for (int i = 0; i < size; i++)
			{
				/* Set the height and sdf */
				QVector<double> feat = points_feats[i];
				m_pointcloud->at(i).setHeight(feat[0]);
				m_pointcloud->at(i).setSdf(feat[1]);
			}
		}
	}

	finish_count--;
	if (finish_count == 0)
	{
		emit estimateCompleted(m_pointcloud);
		finish_count = NUM_OF_THREADS;
	}
}

void FeatureEstimator::onDebugTextAdded(QString text)
{
	emit addDebugText(text);
}

void FeatureEstimator::setPhase(PHASE phase)
{
	m_phase = phase;
}