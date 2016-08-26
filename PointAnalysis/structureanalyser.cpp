  #include "structureanalyser.h"

StructureAnalyser::StructureAnalyser(QObject *parent)
	: QObject(parent), m_fe(NULL), classifier_loaded(false), m_testPCThread(NULL), m_genCandThread(NULL), m_pointcloud(NULL),
	m_predictionThread(NULL), m_segmentationThread(NULL), m_iteration(0), m_null_label(-1)
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
	m_predictionThread(NULL), m_segmentationThread(NULL), m_iteration(0), m_null_label(-1)
{
	qRegisterMetaType<PAPointCloud *>("PAPointCloudPointer");
	qRegisterMetaType<QVector<QMap<int, float>>>("ClassificationDistribution");
	qRegisterMetaType<PAPart>("PAPart");
	qRegisterMetaType<Part_Candidates>("PartCandidates");
	m_pcModel = pcModel;
	m_modelClassName = "coseg_chairs_8";
	m_energy_functions = new EnergyFunctions(m_modelClassName);
	m_parts_structure.set_model(pcModel);
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

	/* Load basic information */
	bool ret = true;
	ret = ret & m_parts_structure.load_labels((label_info_path + label_info_filename).c_str());
	ret = ret & m_parts_structure.load_label_symmetries((label_info_path + label_symmetry_info_filename).c_str());
	/* Load symmetry groups */
	ret = ret & m_parts_structure.load_symmetry_groups((label_info_path + symmetry_group_info_filename).c_str());

	if (!ret)
	{
		do {
			std::cout << "Error: Cannot open label information files.";
			std::cout << '\n' << "Press the Enter key to continue.";
		} while (std::cin.get() != '\n');
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
		m_fe = new FeatureEstimator(m_pcModel, PHASE::TESTING, this);
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

	/* Release the memory of FeatureEstimator */
	if (m_fe != NULL)
		delete(m_fe);   /* may cause exception */

	m_parts_structure.set_pointcloud(pointcloud);

	classifyPoints(pointcloud);
}

using namespace shark;
using namespace std;
void StructureAnalyser::classifyPoints(PAPointCloud *pointcloud)
{
	QString model_file_name = Utils::getModelName(QString::fromStdString(m_pcModel->getInputFilepath()));
	std::string pcFile = "../data/features_test/" + model_file_name.toStdString() + ".csv";
	if (m_pointcloud != NULL)
		delete(m_pointcloud);
	m_pointcloud = NULL;

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
			double feats[POINT_FEATURES_DIMEN];
			for (int j = 0; j < POINT_FEATURES_DIMEN; j++)
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
	m_label_names = QVector<int>::fromList(distribution[0].keys());
	m_null_label = m_label_names.last();
	m_parts_structure.m_null_label = m_null_label;
	/* Set the classification probability distribution to EnergyFunctions object */
	m_energy_functions->setDistributions(distribution);
	/* Set the classification probability distrubution to the point cloud */
	int point_idx = 0;
	for (QVector<QMap<int, float>>::iterator dis_it = distribution.begin(); dis_it != distribution.end(); ++dis_it)
		m_pointcloud->operator[](point_idx++).setClassConfidences(*dis_it);
	

	/* Create a thread to generate the part candidates */
	m_genCandThread = new GenCandidatesThread(m_pointcloud, m_model_name, distribution, this);
	//m_genCandThread = new GenCandidatesThread(m_model_name, m_pointcloud->size(), 144, this);    /* Directly load candidates from local files */
	connect(m_genCandThread, SIGNAL(addDebugText(QString)), this, SLOT(onDebugTextAdded(QString)));
	connect(m_genCandThread, SIGNAL(genCandidatesDone(int, Part_Candidates)), this, SLOT(onGenCandidatesDone(int, Part_Candidates, QVector<int>)));
	connect(m_genCandThread, SIGNAL(setOBBs(QVector<OBB *>)), this, SLOT(setOBBs(QVector<OBB *>)));
	m_genCandThread->start();
}

void StructureAnalyser::onPointLabelsGot(QVector<int> labels)
{
	m_pcModel->setLabels(labels);
}

void StructureAnalyser::onGenCandidatesDone(int num_of_candidates, Part_Candidates part_candidates, QVector<int> point_cluster_map)
{
	onDebugTextAdded("Genarating parts candidates has finished.");
	qDebug() << "Generating parts candidates has finished.";

	m_parts_candidates = part_candidates;
	m_point_assignments = point_cluster_map;

	onDebugTextAdded("There are " + QString::number(part_candidates.size()) + " part candidates in total.");
	qDebug("Threre are %d part candidates in total.", part_candidates.size());

	/* Release the memory of GenCandidatesThread */
	if (m_genCandThread != NULL)
	{
		if (m_genCandThread->isRunning())
			m_genCandThread->terminate();
		delete(m_genCandThread);    /* may cause exception */
		m_genCandThread = NULL;
	}


	/* Do part labels and orientations prediction */
	onDebugTextAdded("Predict part labels and orientations.");
	qDebug() << "Predict part labels and orientations.";

	bool use_symmetry = m_iteration == 0 ? true : false;
	m_predictionThread = new PredictionThread(m_energy_functions, part_candidates, m_label_names, use_symmetry, this);
	connect(m_predictionThread, SIGNAL(predictionDone(QMap<int, int>, std::vector<int>)), this, SLOT(onPredictionDone(QMap<int, int>, std::vector<int>)));
	//connect(m_predictionThread, SIGNAL(predictionDone()), this, SLOT(onPredictionDone()));
	m_predictionThread->start();
}

void StructureAnalyser::onPredictionDone(QMap<int, int> parts_picked, std::vector<int> candidate_labels)
{
	qDebug() << "Part labels and orientations prediction done.";

	int null_label = m_energy_functions->getNullLabelName();
	int numLabels = m_label_names.size();
	QMap<int, OBB *> obbs;
	QVector<OBB *> obbs_to_show;
	QVector<int> cluster_labels(m_parts_candidates.size() / 24);  /* label of each cluster */
	cluster_labels.fill(-1);

	int i = 0;
	for (QMap<int, int>::iterator it = parts_picked.begin(); it != parts_picked.end(); ++it)
	{
		int label = it.key();
		assert(label == i);
		int candidate_idx = it.value();
		PAPart *part = new PAPart(m_parts_candidates[candidate_idx]);
		part->setLabel(label);
		
		OBB * obb = part->getOBB();
		if (obb == NULL)
			obb = part->generateOBB();

		if (part->num_of_samples < param_min_num_cuboid_sample_points)
			part->samplePoints();

		obb->setColor(QVector3D(COLORS[label][0], COLORS[label][1], COLORS[label][2]));
		obbs.insert(label, obb);
		obbs_to_show[i++] = new OBB(obb);

		m_parts_structure.add_part(label, part);

		/* Fill the label of cluster */
		int cluster_no = candidate_idx / 24;
		if (label != null_label)
			cluster_labels[cluster_no] = label;
	}

	/* release the memories occupied by parts candidates */
	m_parts_candidates.clear();

	/* update the points to parts assignments with the optimization result */
	for (QVector<int>::iterator point_it = m_point_assignments.begin(); point_it != m_point_assignments.end(); ++point_it)
	{
		int point_cluster_no = *point_it;
		int assignment;
		/* If the point doesn't belong to any point cluster, set the assignment to null part */
		if (point_cluster_no < 0)
			assignment = m_energy_functions->getNullLabelName();
		else
			assignment = cluster_labels[point_cluster_no];
		*point_it = assignment;
	}

	/* Update points assignments to the parts structure */
	m_parts_structure.set_points_assignments(m_point_assignments);

	/* Update the OBBs and points assignments to EnergyFunctions */
	m_energy_functions->setOBBs(obbs);
	m_energy_functions->setPointAssignments(m_point_assignments);

	/* Display the OBBs of result parts */
	emit sendOBBs(obbs_to_show);

	/* Release the memory of PredictionThread */
	if (m_predictionThread != NULL)
	{
		if (m_predictionThread->isRunning())
			m_predictionThread->terminate();
		delete(m_predictionThread);    /* may cause exception */
		m_predictionThread = NULL;
	}

	/* Compute symmetry groups */
	m_parts_structure.compute_symmetry_groups();

	/* Do point segmentation */
	m_segmentationThread = new PointSegmentationThread(m_pointcloud, m_label_names, m_energy_functions, this);
	connect(m_segmentationThread, SIGNAL(pointSegmentationDone(QVector<int>)), this, SLOT(pointSegmentationDone(QVector<int>)));
	m_segmentationThread->start();
}

void StructureAnalyser::pointSegmentationDone(QVector<int> new_point_assignments)
{
	cout << "Update the point assignments to point cloud." << endl;
	m_pcModel->setLabels(new_point_assignments);

	/* Reassign points to the parts */
	std::vector<PAPart *> all_parts = m_parts_structure.get_all_parts();
	int num_parts = all_parts.size();
	/* Clear previous points assignments in each part */
	for (std::vector<PAPart *>::iterator it = all_parts.begin(); it != all_parts.end(); ++it)
		(*it)->clearVertices();

	/* Reassign */
	assert(new_point_assignments.size() == m_pointcloud->size());
	for (int point_index = 0; point_index < new_point_assignments.size(); point_index++)
	{
		int point_assignment = new_point_assignments[point_index];
		PAPoint point = m_pointcloud->operator[](point_index);

		if (point_assignment >= 0 && point_assignment != m_null_label)
			all_parts[point_assignment]->addVertex(point_index, point.getPosition(), point.getNormal());
	}

	/* Update the samples of the parts and the samples correspondences */
	for (std::vector<PAPart *>::iterator part_it = all_parts.begin(); part_it != all_parts.end(); ++part_it)
	{
		(*part_it)->samplePoints();
		(*part_it)->update_sample_correspondences();
	}


	/* Release the memory of PointSegmentationThread */
	if (m_segmentationThread != NULL)
	{
		if (m_segmentationThread->isRunning())
			m_segmentationThread->terminate();
		delete(m_segmentationThread);
		m_segmentationThread = NULL;
	}
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

int StructureAnalyser::numOfLabels()
{
	return m_label_names.size();
}