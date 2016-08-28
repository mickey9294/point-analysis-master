#include "featureestimator.h"

FeatureEstimator::FeatureEstimator(QObject *parent)
	: QObject(parent)
{
	m_cloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
	m_normals = pcl::PointCloud<pcl::Normal>::Ptr(new pcl::PointCloud < pcl::Normal>);
}

FeatureEstimator::FeatureEstimator(Model *model, PHASE phase, QObject *parent)
	: QObject(parent), finish_count(NUM_FEATURE_THREAD), m_phase(phase)
{
	qDebug() << "Initializing the feature estimator...";
	emit addDebugText("Initializing the feature estimator...");
	m_cloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
	m_normals = pcl::PointCloud<pcl::Normal>::Ptr(new pcl::PointCloud < pcl::Normal>);
	m_pointcloudFile = QString(model->getInputFilepath().c_str());

	if (model->getType() == Model::ModelType::Mesh)    /* If it is a training mesh model */
	{
		MeshModel *mesh = (MeshModel *)model;
		if (mesh->sampleCount() <= 0)
			mesh->samplePoints();
		m_pointcloud = new PAPointCloud(mesh->sampleCount());

		int sample_idx = 0;
		for (Parts_Samples::iterator part_it = mesh->samples_begin(); part_it != mesh->samples_end(); ++part_it)
		{
			int label = part_it.key();
			//qDebug("Part_%d:", label);
			
			for (QVector<SamplePoint>::iterator sample_it = part_it->begin(); sample_it != part_it->end(); ++sample_it)
			{
				//qDebug("Add sample_%d.", sample_idx);
				pcl::PointXYZ p(sample_it->x(), sample_it->y(), sample_it->z());
				m_cloud->push_back(p);
				pcl::Normal normal(sample_it->nx(), sample_it->ny(), sample_it->nz());
				m_normals->push_back(normal);
				m_pointcloud->at(sample_idx).setPosition(sample_it->x(), sample_it->y(), sample_it->z());
				m_pointcloud->at(sample_idx).setNormal(sample_it->nx(), sample_it->ny(), sample_it->nz());
				m_pointcloud->at(sample_idx++).setLabel(label);
			}
		}
	}
	else    /* If it is a testing point cloud model */
	{
		PCModel *pc = (PCModel *)model;
		m_pointcloud = new PAPointCloud(pc->vertexCount());

		int vertex_idx = 0;
		QVector<Eigen::Vector3f>::iterator vertex_it, normal_it;
		for (vertex_it = pc->vertices_begin(), normal_it = pc->normals_begin();
			vertex_it != pc->vertices_end() && normal_it != pc->normals_end(); ++vertex_it, ++normal_it)
		{
			pcl::PointXYZ p(vertex_it->x(), vertex_it->y(), vertex_it->z());
			m_cloud->push_back(p);
			pcl::Normal normal(normal_it->x(), normal_it->y(), normal_it->z());
			m_normals->push_back(normal);
			m_pointcloud->at(vertex_idx).setPosition(vertex_it->x(), vertex_it->y(), vertex_it->z());
			m_pointcloud->at(vertex_idx).setNormal(normal_it->x(), normal_it->y(), normal_it->z());
			m_pointcloud->at(vertex_idx++).setLabel(10);
		}
	}

	m_pointcloud->setRadius(1.0);
	m_radius = 1.0;
	finish_count = NUM_FEATURE_THREAD;

	qRegisterMetaType<QVector<QVector<double>>>("FeatureVector");
	qDebug() << "Initialization done.";
	emit addDebugText("Initialization done.");
}

void FeatureEstimator::reset(Model *model)
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
	if (m_normals->size() > 0)
		m_normals->clear();

	qDebug() << "Reset the feature estimator...";
	emit addDebugText("Reset the feature estimator...");
	m_cloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
	m_normals = pcl::PointCloud<pcl::Normal>::Ptr(new pcl::PointCloud < pcl::Normal>);
	m_pointcloudFile = QString::fromStdString(model->getInputFilepath());

	if (model->getType() == Model::ModelType::Mesh)  /* If it is a training mesh model */
	{
		MeshModel * mesh = (MeshModel *)model;
		if (mesh->sampleCount() <= 0)
			mesh->samplePoints();
		m_pointcloud = new PAPointCloud(mesh->sampleCount());

		int sample_idx = 0;
		for (Parts_Samples::iterator part_it = mesh->samples_begin(); part_it != mesh->samples_end(); ++part_it)
		{
			int label = part_it.key();
			
			for (QVector<SamplePoint>::iterator sample_it = part_it->begin(); sample_it != part_it->end(); ++sample_it)
			{
				pcl::PointXYZ p((*sample_it).nx(), (*sample_it).y(), (*sample_it).z());
				m_cloud->push_back(p);
				pcl::Normal normal((*sample_it).nx(), (*sample_it).ny(), (*sample_it).nz());
				m_normals->push_back(normal);
				m_pointcloud->at(sample_idx).setPosition((*sample_it).x(), (*sample_it).y(), (*sample_it).z());
				m_pointcloud->at(sample_idx++).setLabel(label);
			}
		}
	}
	else    /* If it is a testing point cloud model */
	{
		PCModel *pc = (PCModel *)model;
		m_pointcloud = new PAPointCloud(pc->vertexCount());

		QVector<Eigen::Vector3f>::iterator vertex_it, normal_it;
		int vertex_idx = 0;
		for (vertex_it = pc->vertices_begin(), normal_it = pc->normals_begin(); 
			vertex_it != pc->vertices_end() && normal_it != pc->normals_end(); ++vertex_it, ++normal_it)
		{
			pcl::PointXYZ p(vertex_it->x(), vertex_it->y(), vertex_it->z());
			m_cloud->push_back(p);
			pcl::Normal normal(normal_it->x(), normal_it->y(), normal_it->z());
			m_normals->push_back(normal);
			m_pointcloud->at(vertex_idx).setPosition(vertex_it->x(), vertex_it->y(), vertex_it->z());
			m_pointcloud->at(vertex_idx).setLabel(10);
		}
	}

	m_pointcloud->setRadius(1.0);
	m_radius = 1.0;
	finish_count = NUM_FEATURE_THREAD;
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
	finish_count = NUM_FEATURE_THREAD;
	qDebug() << "Estimating the point features...";
	emit addDebugText("Estimating the point features...");
	/* Create subthread to estimate point features in 5 different search radius */
	qDebug() << "Create 6 FeatureThreads, each of which esitmate the point features in a particular search radius.";
	emit addDebugText("Create 6 FeatureThreads, each of which esitmate the point features in a particular search radius.");

	for (int i = 0; i < NUM_FEATURE_THREAD; i++)
	{
		double coefficient = 0.1 * (i + 1);
		FeatureThread * thread = new FeatureThread(i, m_cloud, m_normals, m_radius, coefficient, this);
		connect(thread, SIGNAL(estimateCompleted(int, QVector<QVector<double>>)), this, SLOT(receiveFeatures(int, QVector<QVector<double>>)));
		connect(thread, SIGNAL(addDebugText(QString)), this, SLOT(onDebugTextAdded(QString)));
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
	
	if (sid < NUM_FEATURE_THREAD - 1)   /* It is the thread which computes features based on neighborhood */
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
		for (int i = 0; i < size; i++)
		{
			/* Set the height and sdf for each point */
			QVector<double> feat = points_feats[i];
			m_pointcloud->at(i).setHeight(feat[0]);
			m_pointcloud->at(i).setSdf(feat[1]);
		}
	}

	finish_count--;
	/* If all the subthread have finished, send the result out */
	if (finish_count == 0)
	{
		emit estimateCompleted(m_pointcloud);
		finish_count = NUM_FEATURE_THREAD;
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