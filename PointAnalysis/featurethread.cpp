#include "featurethread.h"

FeatureThread::FeatureThread(int idno, pcl::PointCloud<pcl::PointXYZ>::Ptr c, pcl::PointCloud<pcl::Normal>::Ptr n,
	double rad, double coef, QObject *parent)
	: QThread(parent), finish_count(NUM_OF_SUBTHREAD)
{
	if (idno == 5)
		qDebug() << " ";
	qDebug("FeatureThread-%d is created.", idno);
	QString dtext = "FeatureThread-" + QString::number(id) + " id created";
	emit addDebugText(dtext);

	cloud = c;
	normals = n;
	radius = rad;
	id = idno;
	coefficient = coef;
	cloud_feats.resize(cloud->size());

	qRegisterMetaType<QVector<QVector<double>>>("FeatureVector");
	qRegisterMetaType<QList<QVector<double>>>("FeatureList");
	//qRegisterMetaType<pcl::search::KdTree<pcl::PointXYZ>::Ptr>("KdTreePointer");
	connect(this, SIGNAL(firstStepCompleted()), this, SLOT(nextEstimateStep()));
}

FeatureThread::~FeatureThread()
{
	for (int i = 0; i < subthreads.size(); i++)
	{
		if (subthreads[i] != NULL)
		{
			if (subthreads[i]->isRunning())
				subthreads[i]->terminate();
			delete(subthreads[i]);
			subthreads[i] = NULL;
		}
	}
	subthreads.clear();

	if (isRunning())
		terminate();
}

void FeatureThread::run()
{
	estimate();
}

void FeatureThread::estimate()
{
	if (id < 5){    /* If the thread is used to estimate features based on the neighborhood */
		pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
		ne.setInputCloud(cloud);

		// Create an empty kdtree representation, and pass it to the normal estimation object.
		// Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
		kdtree = pcl::search::KdTree<pcl::PointXYZ>::Ptr(new pcl::search::KdTree<pcl::PointXYZ>());
		ne.setSearchMethod(kdtree);
		// Output datasets
		pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);
		
		ne.setRadiusSearch(coefficient * radius);
		// Compute the features
		qDebug("FeatureThread-%d: Estimating normals and curvatures with radius %f...", id, coefficient * radius);
		QString dtext = "FeatureThread-" + QString::number(id) + ": Estimating normals and curvatures with radius "
			+ QString::number(coefficient * radius) + "...";
		emit addDebugText(dtext);

		ne.compute(*cloud_normals);
		qDebug("FeatureThread-%d: Normals and curvatures estimation done.", id);
		dtext = "FeatureThread-" + QString::number(id) + ": Normals and curvatures estimation done.";
		emit addDebugText(dtext);

		float curv = 0;
		/* Set curvature values to normals */
		for (int i = 0; i < normals->size(); i++)
		{
			curv = cloud_normals->at(i).curvature;
			if (!(curv >= 0 && curv <= 1.0))
				curv = FLOAT_INF;
			normals->at(i).curvature = curv;
		}

		emit firstStepCompleted();
	}
	else    /* If the thread is used to estimate height and sdf */
	{
		qDebug("FeatureThread-%d: Estimating heights and sdf values of points...", id);
		QString dtext = "FeatureThread-" + QString::number(id) + ": Estimating heights and sdf values of points...";
		emit addDebugText(dtext);

		
		//QVector<double> sdfs;
		//if (input_filename.length() > 0){    /* If it is processing the training data model */
		//	/* Get the file path of off mesh of the current mesh model */
		//	QString mesh_filepath = "../data/off_modified/" + Utils::getModelName(input_filename) + "_modified.off";
		//	emit addDebugText("Compute sdf values with " + mesh_filepath + ".");
		//	/* Compute the sdf values */
		//	sdfs = Utils::sdf_mesh(mesh_filepath);
		//}

		for (int i = 0; i < cloud->size(); i++)
		{
			double height = cloud->at(i).y;
			//double sdf = Utils::sdf(cloud, normals, i);
			QVector<double> feat(2);
			feat[0] = height;
			/* If it is processing the training data model, set sdf value to 0 and leave it to be computed by Feature estimator */
			if (input_filename.length() > 0)   
				feat[1] = 0;
			else
			{
				feat[1] = Utils::sdf(cloud, normals, i);    /* If it is processing the testing data model */
			}
			cloud_feats[i] = feat;
		}
		qDebug("FeatureThread-%d: Heights and sdf estimation done.", id);
		dtext = "FeatureThread-" + QString::number(id) + ": Heights and sdf estimation done.";
		emit addDebugText(dtext);

		emit estimateCompleted(id, cloud_feats);
	}
}

void FeatureThread::receiveFeatures(int sid, QList<QVector<double>> points_feats)
{
	qDebug("FeatureThread-%d receives features from PointFeatureThread-%d-%d.", id, id, sid);
	QString dtext = "FeatureThread-" + QString::number(id) + " receives features from PointFeatureThread-"
		+ QString::number(id) + "-" + QString::number(sid) + ".";
	emit addDebugText(dtext);

	int one = cloud->size() / NUM_OF_SUBTHREAD;
	for (int i = sid * one; i < sid * one + points_feats.size(); i++)
	{
		QVector<double> *feats = cloud_feats.data() + sid * one;
		for (int j = 0; j < points_feats.size(); j++)
			feats[j] = points_feats[j];
	}
	//delete(subthreads[sid]);
	//subthreads[sid] = NULL;

	finish_count--;
	if (finish_count == 0)
	{
		finish_count = NUM_OF_SUBTHREAD;
		emit estimateCompleted(id, cloud_feats);
	}
}

void FeatureThread::nextEstimateStep()
{
	/* Create 8 subthreads to estimate the point features */
	qDebug("FeatureThread-%d: Creating subthreads, each of which estimate part of points...", id);
	QString dtext = "FeatureThread-" + QString::number(id) + ": Creating subthreads, each of which estimate part of points...";
	emit addDebugText(dtext);

	int one = cloud->size() / NUM_OF_SUBTHREAD;
	for (int i = 0; i < NUM_OF_SUBTHREAD; i++)
	{
		int end = (i == NUM_OF_SUBTHREAD - 1) ? (cloud->size() - 1) : ((i + 1) * one - 1);
		int begin = i * one;
		PointFeatureThread * pointThread = new PointFeatureThread(id, i, cloud, normals, kdtree, coefficient, radius, begin, end, this);
		connect(pointThread, SIGNAL(estimateCompleted(int, QList<QVector<double>>)), this, SLOT(receiveFeatures(int, QList<QVector<double>>)));
		connect(pointThread, SIGNAL(addDebugText(QString)), this, SLOT(onDebugTextAdded(QString)));
		subthreads.push_back(pointThread);
		pointThread->start();
	}
}

void FeatureThread::onDebugTextAdded(QString text)
{
	emit addDebugText(text);
}

void FeatureThread::setInputFilename(QString filename)
{
	input_filename = filename;
}