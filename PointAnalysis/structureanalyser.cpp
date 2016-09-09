  #include "structureanalyser.h"

StructureAnalyser::StructureAnalyser(QObject *parent)
	: QThread(parent), m_iteration(0), m_null_label(-1)
{
	qRegisterMetaType<PAPointCloud *>("PAPointCloudPointer");
	qRegisterMetaType<QVector<QMap<int, float>>>("ClassificationDistribution");
	qRegisterMetaType<PAPart>("PAPart");
	qRegisterMetaType<Part_Candidates>("PartCandidates");
	qRegisterMetaType<PartsStructure *>("PartsStructure");

	m_modelClassName = "coseg_chairs_8";
	m_energy_functions.reset(new EnergyFunctions(m_modelClassName));

	m_point_classifier.reset(new PointClassifier(this));
	m_candidates_generator.reset(new CandidatesGenerator(this));
	m_parts_predictor.reset(new PartsPredictor(this));
	m_point_segmentation.reset(new PointSegmentation(this));
	m_parts_solver.reset(new PartsSolver());

	connect(m_candidates_generator.data(), SIGNAL(setOBBs(QVector<OBB *>)), this, SLOT(setOBBs(QVector<OBB *>)));
	
}

StructureAnalyser::StructureAnalyser(PCModel *pcModel, QObject * parent)
	: QThread(parent), m_iteration(0), m_null_label(-1)
{
	qRegisterMetaType<PAPointCloud *>("PAPointCloudPointer");
	qRegisterMetaType<QVector<QMap<int, float>>>("ClassificationDistribution");
	qRegisterMetaType<PAPart>("PAPart");
	qRegisterMetaType<Part_Candidates>("PartCandidates");
	qRegisterMetaType<PartsStructure *>("PartsStructure");
	m_pcModel = pcModel;
	m_modelClassName = "coseg_chairs_8";
	m_energy_functions.reset(new EnergyFunctions(m_modelClassName));
	m_parts_structure.set_model(pcModel);

	m_point_classifier.reset(new PointClassifier(this));
	m_candidates_generator.reset(new CandidatesGenerator(this));
	m_parts_predictor.reset(new PartsPredictor(this));
	m_point_segmentation.reset(new PointSegmentation(this));
	m_parts_solver.reset(new PartsSolver());

	connect(m_candidates_generator.data(), SIGNAL(setOBBs(QVector<OBB *>)), this, SLOT(setOBBs(QVector<OBB *>)));
}

StructureAnalyser::~StructureAnalyser()
{

	//delete(m_joint_normal_predictor);

	for (std::vector<std::list<CuboidFeatures *>>::iterator it = m_feature_list.begin(); it != m_feature_list.end(); ++it)
	{
		for (std::list<CuboidFeatures *>::iterator jt = it->begin(); jt != it->end(); ++jt)
		{
			if (*jt != NULL)
				delete(*jt);
			*jt = NULL;
		}
		it->clear();
	}
	m_feature_list.clear();

	for (std::vector<std::list<CuboidTransformation *>>::iterator it = m_transformation_list.begin();
		it != m_transformation_list.end(); ++it)
	{
		for (std::list<CuboidTransformation *>::iterator jt = it->begin(); jt != it->end(); jt++)
		{
			if (*jt != NULL)
				delete(*jt);
			*jt = NULL;
		}
		it->clear();
	}
	m_transformation_list.clear();

}

void StructureAnalyser::execute()
{
	m_model_name = Utils::getModelName(m_pcModel->getInputFilepath());

	/* Estimate poitn cloud features */
	m_fe.reset(new FeatureEstimator(m_pcModel, PHASE::TESTING, this));
	connect(m_fe.data(), SIGNAL(addDebugText(QString)), this, SLOT(onDebugTextAdded(QString)));
	connect(m_fe.data(), SIGNAL(estimateCompleted(PAPointCloud *)), this, SLOT(initialize(PAPointCloud *)));
	onDebugTextAdded("Estimating points features...");
	m_fe->estimateFeatures();

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

	m_pointcloud.reset(pointcloud);
	m_parts_structure.set_pointcloud(m_pointcloud);

	this->start();
}

using namespace shark;
using namespace std;

void StructureAnalyser::run()
{
	predict();
}

void StructureAnalyser::classifyPoints(QVector<int> & pc_labels, QVector<QMap<int, float>> &distribution)
{

	m_point_classifier->setLabelNames(m_parts_structure.m_label_names);
	m_point_classifier->setModelClassName(m_modelClassName);
	m_point_classifier->testPointCloud(m_model_name, pc_labels, distribution);

	//emit setPointcloudLabels(pc_labels);
	m_pcModel->setLabels(pc_labels);

	//m_label_names = QVector<int>::fromStdVector(m_parts_structure.m_label_names);
	//m_null_label = m_label_names.last();
	//m_parts_structure.m_null_label = m_null_label;

	/* Set the classification probability distribution to EnergyFunctions object */
	m_energy_functions->setDistributions(distribution);

	/* Set the classification probability distrubution to the point cloud */
	int point_idx = 0;
	for (QVector<QMap<int, float>>::iterator dis_it = distribution.begin(); dis_it != distribution.end(); ++dis_it)
	{
		/* Sum scores of all symmetric labels up and set it to all labels in one symmetric group */
		for (std::vector<std::list<LabelIndex>>::iterator it = m_parts_structure.m_label_symmetries.begin();
			it != m_parts_structure.m_label_symmetries.end(); ++it)
		{
			if (it->size() > 1)
			{
				float sum = 0;
				for (std::list<LabelIndex>::iterator label_it = it->begin(); label_it != it->end(); ++label_it)
					sum += dis_it->operator[](*label_it);

				for (std::list<LabelIndex>::iterator label_it = it->begin(); label_it != it->end(); ++label_it)
					dis_it->operator[](*label_it) = sum;
			}
		}

		m_pointcloud->operator[](point_idx++).setClassConfidences(*dis_it);
	}
}

using namespace pcl;

void StructureAnalyser::setPointCloud(PCModel *pcModel)
{
	m_pcModel = pcModel;
	m_parts_structure.set_model(pcModel);
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

void StructureAnalyser::get_joint_normal_relations(std::vector<std::vector<CuboidJointNormalRelations *>> &relations)
{
	const int num_features = CuboidFeatures::k_num_features;

	//int num_labels = m_parts_structure.m_label_names.size();
	int num_labels = m_feature_list.size();
	int num_objects = m_parts_structure.num_of_parts();

	assert(m_parts_structure.m_label_parts.size() == num_labels);

	for (std::vector<std::vector<CuboidJointNormalRelations *>>::iterator it_1 = relations.begin(); it_1 != relations.end(); ++it_1)
		for (std::vector<CuboidJointNormalRelations *>::iterator it_2 = it_1->begin(); it_2 != it_1->end(); ++it_2)
			delete (*it_2);

	relations.clear();
	relations.resize(num_labels);
	for (int part_index = 0; part_index < num_labels; ++part_index)
		relations[part_index].resize(num_labels, NULL);

	for (int label_index_1 = 0; label_index_1 < num_labels; label_index_1++)
	{
		for (int label_index_2 = 0; label_index_2 < num_labels; label_index_2++)
		{
			assert(m_object_list.size() == m_feature_list[label_index_1].size());
			assert(m_object_list.size() == m_feature_list[label_index_2].size());
			assert(m_object_list.size() == m_transformation_list[label_index_1].size());
			assert(m_object_list.size() == m_transformation_list[label_index_2].size());

			std::vector<CuboidFeatures *> feature_1;
			std::vector<CuboidFeatures *> feature_2;
			std::vector<CuboidTransformation *> transformation_1;
			std::vector<CuboidTransformation *> transformation_2;

			feature_1.reserve(m_feature_list[label_index_1].size());
			feature_2.reserve(m_feature_list[label_index_2].size());
			transformation_1.reserve(m_transformation_list[label_index_1].size());
			transformation_2.reserve(m_transformation_list[label_index_2].size());

			std::list<std::string>::const_iterator o_it = m_object_list.begin();
			std::list<CuboidFeatures *>::const_iterator f_it_1 = m_feature_list[label_index_1].begin();
			std::list<CuboidFeatures *>::const_iterator f_it_2 = m_feature_list[label_index_2].begin();
			std::list<CuboidTransformation *>::const_iterator t_it_1 = m_transformation_list[label_index_1].begin();
			std::list<CuboidTransformation *>::const_iterator t_it_2 = m_transformation_list[label_index_2].begin();

			int num_objects = 0;
			
			while (true)
			{
				if (f_it_1 == m_feature_list[label_index_1].end()
					|| f_it_2 == m_feature_list[label_index_2].end()
					|| t_it_1 == m_transformation_list[label_index_1].end()
					|| t_it_2 == m_transformation_list[label_index_2].end())
					break;

				bool has_values = (!(*f_it_1)->has_nan() && !(*f_it_2)->has_nan());

				if (has_values)
				{
					feature_1.push_back(*f_it_1);
					feature_2.push_back(*f_it_2);
					transformation_1.push_back(*t_it_1);
					transformation_2.push_back(*t_it_2);
					++num_objects;
				}

				++o_it;
				++f_it_1;
				++f_it_2;
				++t_it_1;
				++t_it_2;
			}

			if (num_objects == 0)
				continue;

			relations[label_index_1][label_index_2] = new CuboidJointNormalRelations();
			CuboidJointNormalRelations *relation_12 = relations[label_index_1][label_index_2];
			assert(relation_12);

			assert(feature_1.size() == num_objects);
			assert(feature_2.size() == num_objects);
			assert(transformation_1.size() == num_objects);
			assert(transformation_2.size() == num_objects);

			// NOTE:
			// Since the center point is always the origin in the local coordinates,
			// it is not used as the feature values.
			const unsigned int num_cols = CuboidJointNormalRelations::k_mat_size;
			Eigen::MatrixXd X(num_objects, num_cols);

			for (int object_index = 0; object_index < num_objects; ++object_index)
			{
				assert(feature_1[object_index]);
				assert(feature_2[object_index]);
				assert(transformation_1[object_index]);
				assert(transformation_2[object_index]);

				Eigen::VectorXd pairwise_feature_vec;
				CuboidJointNormalRelations::get_pairwise_cuboid_features(
					(*feature_1[object_index]), (*feature_2[object_index]),
					transformation_1[object_index], transformation_2[object_index],
					pairwise_feature_vec);

				X.row(object_index) = pairwise_feature_vec;
			}

			Eigen::RowVectorXd mean = X.colwise().mean();
			Eigen::MatrixXd centered_X = X.rowwise() - mean;

			Eigen::MatrixXd cov = (centered_X.transpose() * centered_X) / static_cast<double>(num_objects);
			Eigen::MatrixXd inv_cov = Utils::regularized_inverse(cov);

			relation_12->set_mean(mean.transpose());
			relation_12->set_inv_cov(inv_cov);

#ifdef DEBUG_TEST
			Eigen::MatrixXd diff = (X.rowwise() - mean).transpose();
			Eigen::VectorXd error = (diff.transpose() * inv_cov * diff).diagonal();
			std::cout << "(" << label_index_1 << ", " << label_index_2 << "): max_error = " << error.maxCoeff() << std::endl;
#endif
		}
	}
}

bool StructureAnalyser::load_object_list(const std::string &_filename)
{
	std::ifstream file(_filename);
	if (!file)
	{
		std::cerr << "Can't load file: \"" << _filename << "\"" << std::endl;
		return false;
	}

	m_object_list.clear();
	std::string buffer;

	while (!file.eof())
	{
		std::getline(file, buffer);
		if (buffer == "") break;
		m_object_list.push_back(buffer);
	}

	return true;
}

bool StructureAnalyser::load_features(const std::string & _filename_prefix)
{
	for (std::vector<std::list<CuboidFeatures *>>::iterator f_it = m_feature_list.begin();
		f_it != m_feature_list.end(); ++f_it)
	{
		for (std::list<CuboidFeatures *>::iterator f_jt = f_it->begin(); f_jt != f_it->end(); ++f_jt)
			delete(*f_jt);
		f_it->clear();
	}
	m_feature_list.clear();

	for (int cuboid_index = 0; true; cuboid_index++)
	{
		std::stringstream sstr;
		sstr << _filename_prefix << cuboid_index << std::string(".csv");
		std::string attributes_filename = sstr.str();

		QFileInfo attributes_file(attributes_filename.c_str());
		if (!attributes_file.exists())
			break;

		std::cout << "Loading '" << attributes_filename << "'..." << std::endl;

		std::list<CuboidFeatures *> stats;
		CuboidFeatures::load_feature_collection(
			attributes_filename.c_str(), stats);

		m_feature_list.push_back(stats);
	}

	return true;
}

bool StructureAnalyser::load_transformations(const std::string &_filename_prefix)
{
	for (std::vector<std::list<CuboidTransformation *>>::iterator t_it = m_transformation_list.begin();
		t_it != m_transformation_list.end(); ++t_it)
	{
		for (std::list<CuboidTransformation *>::iterator t_jt = t_it->begin();
			t_jt != t_it->end(); ++t_jt)
			delete(*t_jt);
		t_it->clear();
	}
	m_transformation_list.clear();

	for (int cuboid_index = 0; true; ++cuboid_index)
	{
		std::stringstream  transformation_filename_sstr;
		transformation_filename_sstr << _filename_prefix << cuboid_index << std::string(".csv");

		QFileInfo transformation_file(transformation_filename_sstr.str().c_str());
		if (!transformation_file.exists())
			break;

		std::cout << "Loading '" << transformation_filename_sstr.str() << "'..." << std::endl;

		std::list<CuboidTransformation *> stats;
		CuboidTransformation::load_transformation_collection(
			transformation_filename_sstr.str().c_str(), stats);

		m_transformation_list.push_back(stats);
	}

	return true;
}

void StructureAnalyser::predict()
{
	/* Load basic information */
	bool ret = true;
	ret = ret & m_parts_structure.load_labels((label_info_path + label_info_filename).c_str());
	ret = ret & m_parts_structure.load_label_symmetries((label_info_path + label_symmetry_info_filename).c_str());
	/* Load symmetry groups */
	ret = ret & m_parts_structure.load_symmetry_groups((label_info_path + symmetry_group_info_filename).c_str());
	/* Load CuboidFeatures and CuboidTransformations */
	load_object_list("../data/coseg_chairs_8_list.txt");
	load_features(training_dir + std::string("/") + feature_filename_prefix);
	load_transformations(training_dir + std::string("/") + transformation_filename_prefix);

	m_parts_structure.clear_parts();

	if (!ret)
	{
		do {
			std::cout << "Error: Cannot open label information files.";
			std::cout << '\n' << "Press the Enter key to continue.";
		} while (std::cin.get() != '\n');
	}

	std::vector<std::vector<CuboidJointNormalRelations *>> joint_normal_relations;
	get_joint_normal_relations(joint_normal_relations);
	CuboidJointNormalRelationPredictor joint_normal_predictor(joint_normal_relations);

	QVector<int> pc_labels;
	QVector<QMap<int, float>> distribution;
	classifyPoints(pc_labels, distribution);

	m_label_names = QVector<int>::fromList(distribution[0].keys());
	m_null_label = m_label_names.last();
	m_parts_structure.m_null_label = m_null_label;

	Part_Candidates part_candidates;
	QVector<int> point_cluster_map;
	generateCandidates(distribution, part_candidates, point_cluster_map);


	while (m_iteration < 30)
	{
		QMap<int, int> parts_picked;
		std::vector<int> candidate_labels;
		predictPartLabelsAndOrientations(part_candidates, m_label_names, parts_picked, candidate_labels);

		/* Compute symmetry groups */
		m_parts_structure.compute_symmetry_groups();

		segmentPoints();

		m_parts_solver->optimizeAttributes(m_parts_structure, joint_normal_predictor, param_opt_single_energy_term_weight,
			param_opt_symmetry_energy_term_weight, param_opt_max_iterations, false);

		const bool use_symmetry = !(disable_symmetry_terms);
		if (use_symmetry)
		{
			m_parts_structure.compute_symmetry_groups();

			m_parts_solver->optimizeAttributes(m_parts_structure, joint_normal_predictor, param_opt_single_energy_term_weight,
				param_opt_symmetry_energy_term_weight, param_opt_max_iterations, use_symmetry);
		}
	}
}

void StructureAnalyser::generateCandidates(const QVector<QMap<int, float>> & distribution, Part_Candidates & part_candidates, QVector<int> & point_cluster_map)
{
	m_candidates_generator->generateCandidates(m_model_name, m_pointcloud, distribution, part_candidates, point_cluster_map);
}

void StructureAnalyser::predictPartLabelsAndOrientations(const Part_Candidates & part_candidates, const QVector<int> & label_names,
	QMap<int, int> &parts_picked, std::vector<int> &candidate_labels)
{
	m_parts_predictor->setModelClassName(m_modelClassName);
	if (m_iteration == 0)
		m_parts_predictor->setUseSymmetry(true);
	else
		m_parts_predictor->setUseSymmetry(false);

	m_parts_predictor->predictLabelsAndOrientations(part_candidates, label_names, parts_picked, candidate_labels, m_energy_functions);

	int numLabels = m_label_names.size();
	QMap<int, OBB *> obbs;
	QVector<OBB *> obbs_to_show;
	obbs_to_show.reserve(parts_picked.size());
	QVector<int> cluster_labels(part_candidates.size() / 24);  /* label of each cluster */
	cluster_labels.fill(-1);

	for (QMap<int, int>::iterator it = parts_picked.begin(); it != parts_picked.end(); ++it)
	{
		cout << "Part label " << it.key() << " correspondes to Candidate-" << it.value() << endl;

		int label = it.key();

		int candidate_idx = it.value();
		PAPart *part = new PAPart(part_candidates[candidate_idx]);
		part->setLabel(label);

		OBB * obb = part->getOBB();
		if (obb == NULL)
			obb = part->generateOBB();

		if (part->numOfSamples() < param_min_num_cuboid_sample_points)
			part->samplePoints();

		obb->setColor(QVector3D(COLORS[label][0], COLORS[label][1], COLORS[label][2]));
		obbs.insert(label, obb);
		obbs_to_show.push_back(new OBB(obb));

		m_parts_structure.add_part(label, part);
	}

	/* update the points to points assignments with the optimization result */
	for (std::vector<int>::iterator ass_it = m_parts_structure.m_points_assignments.begin();
		ass_it != m_parts_structure.m_points_assignments.end(); ++ass_it)
		(*ass_it) = m_parts_structure.m_null_label;
	std::vector<PAPart *> all_parts = m_parts_structure.get_all_parts();
	for (std::vector<PAPart *>::iterator part_it = all_parts.begin(); part_it != all_parts.end(); ++part_it)
	{
		int label = (*part_it)->getLabel();
		std::vector<int> vertices_indices = (*part_it)->getVerticesIndices();
		for (std::vector<int>::iterator index_it = vertices_indices.begin(); index_it != vertices_indices.end(); ++index_it)
		{
			m_parts_structure.m_points_assignments[*index_it] = label;
			m_pointcloud->operator[](*index_it).setLabel(label);
		}
	}

	/* Update the labels of point cloud that is being displayed */
	m_pcModel->setLabels(QVector<int>::fromStdVector(m_parts_structure.m_points_assignments));

	/* Update the OBBs and points assignments to EnergyFunctions */
	m_energy_functions->setOBBs(obbs);
	m_energy_functions->setPointAssignments(QVector<int>::fromStdVector(m_parts_structure.m_points_assignments));

	/* Display the OBBs of result parts */
	emit sendOBBs(obbs_to_show);
}

void StructureAnalyser::segmentPoints()
{
	QVector<int> point_assignments_segmented;

	m_point_segmentation->segmentPoints(m_parts_structure, m_label_names, point_assignments_segmented);

	m_pcModel->setLabels(point_assignments_segmented);

	//std::ofstream out("../data/nps.csv");
	//for (QVector<int>::iterator it = new_point_assignments.begin(); it != new_point_assignments.end(); ++it)
	//	out << *it << std::endl;
	cout << "Updating done." << endl;
	//out.close();
}