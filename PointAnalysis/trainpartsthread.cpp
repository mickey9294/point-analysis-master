#include "trainpartsthread.h"

using namespace std;

TrainPartsThread::TrainPartsThread(QVector<int> label_names, QObject *parent)
	: QThread(parent), currentId(0), m_num_labels(8), m_label_names(label_names)
{
	//qRegisterMetaType<Parts_Vector>("QVector<PAPart>");
	cout << "TrainPartsThread is created." << endl;

	loadThread.setPhase(PHASE::TRAINING);
	pcaThread.setPhase(PHASE::TRAINING);
	
	connect(&loadThread, SIGNAL(loadPointsCompleted(Model *)), this, SLOT(receiveModel(Model *)));
	connect(&pcaThread, SIGNAL(estimatePartsDone(Parts_Vector)), this, SLOT(receiveParts(Parts_Vector)));

	m_feature_list.resize(label_names.size());
	m_transformation_list.resize(label_names.size());

	ifstream list_in("../data/coseg_chairs_8_list.txt");
	if (list_in.is_open())
	{
		const int BUFFER_SIZE = 128;
		char buffer[BUFFER_SIZE];
		while (!list_in.eof())
		{
			list_in.getline(buffer, BUFFER_SIZE);
			if (strlen(buffer) > 0)
			{
				file_list.push_back(string(buffer));
				if (class_name.length() < 1)
				{
					QString qstr(buffer);
					char separate = qstr.contains('\\') ? '\\' : '/';
					QString className = qstr.section(separate, -3, -3);
					class_name = className.toStdString();
				}
			}
		}
	}

	cout << "Load point cloud from" << file_list[currentId] << endl;
	emit addDebugText(QString::fromStdString(file_list[currentId]));
	loadThread.setLoadFileName(file_list[currentId]);
	loadThread.start();
}

TrainPartsThread::~TrainPartsThread()
{
	if (isRunning())
		terminate();
}

void TrainPartsThread::run()
{
	analyseProbPartModel();
	savePartRelationPriors();
}

void TrainPartsThread::receiveModel(Model *model)
{
	qDebug() << "Receive point cloud from" << QString::fromStdString(model->getInputFilepath());

	emit showModel(model);
	pcaThread.setPointCloud(model);
	pcaThread.start();
}

void TrainPartsThread::receiveParts(Parts_Vector parts)
{
	cout << "Receive parts vectors of point cloud " << file_list[currentId] << endl;

	QVector<int> labels(parts.size());    /* the i-th component of the vector represents the label of the i-th part */
	QMap<int, int> labels_indices;    /* the label-index map showing the index of the part with the particular label */
	for (int i = 0; i < parts.size(); i++)
	{
		labels[i] = parts[i]->getLabel();
		labels_indices.insert(labels[i], i);
	}

	/* Create CuboidFeatures and CuboidTransformation for the model and add them to the cuboid relation list */
	std::string model_name = Utils::getModelName(file_list[currentId]);
	for (int label_index = 0; label_index < m_num_labels; label_index++)
	{
		CuboidTransformation *transformation = new CuboidTransformation(model_name);
		CuboidFeatures * features = new CuboidFeatures(model_name);

		PAPart * cuboid = NULL;
		int cuboid_index = labels_indices.value(label_index, -1);
		if (cuboid_index > -1)
			cuboid = parts[cuboid_index];

		assert(cuboid);

		transformation->compute_transformation(cuboid);
		features->compute_features(cuboid);
		

		m_transformation_list[label_index].push_back(transformation);
		m_feature_list[label_index].push_back(features);
	}

	//QVector<QPair<int, int>> part_pairs = Utils::getCombinations(labels);
	//for (int i = 0; i < part_pairs.size(); i++)
	//{
	//	QPair<int, int> part_pair = part_pairs[i];
	//	PAPartRelation relation(parts[labels_indices.value(part_pair.first)], parts[labels_indices.value(part_pair.second)]);
	//	if (!partRelations.contains(part_pair))
	//		partRelations.insert(part_pair, QList<PAPartRelation>());
	//	partRelations[part_pair].push_back(relation);
	//}

	for (QMap<int, int>::iterator label_it_1 = labels_indices.begin(); label_it_1 != labels_indices.end(); label_it_1++)
	{
		int label1 = label_it_1.key();
		int part_index1 = label_it_1.value();
		for (QMap<int, int>::iterator label_it_2 = labels_indices.begin(); label_it_2 != labels_indices.end(); label_it_2++)
		{
			int label2 = label_it_2.key();
			if (label1 != label2)
			{
				int part_index2 = label_it_2.value();
				PAPartRelation relation(*(parts[part_index1]), *(parts[part_index2]));
				QPair<int, int> part_pair(label1, label2);
				if (!partRelations.contains(part_pair))
					partRelations.insert(part_pair, QList<PAPartRelation>());
				partRelations[part_pair].push_back(relation);
			}
		}
	}

	currentId++;

	/* Release the memory of the parts pointers */
	for (QVector<PAPart *>::iterator it = parts.begin(); it != parts.end(); ++it)
	{
		if (*it != NULL)
			delete(*it);
		*it = NULL;
	}
	parts.clear();

	if (currentId < file_list.size())   /* If there are point clouds remain unestimating */
	{
		/* Load the next point cloud */
		if (loadThread.isRunning())
			loadThread.terminate();
		std::cout << "Load point cloud from " <<  file_list[currentId] << std::endl;
		emit addDebugText(QString::fromStdString(file_list[currentId]));
		loadThread.setLoadFileName(file_list[currentId]);
		loadThread.start();
	}
	else
	{
		/* Check the part relations stored in partRelations*/
		QMap<QPair<int, int>, QList<PAPartRelation>>::iterator it;
		for (it = partRelations.begin(); it != partRelations.end(); ++it)
		{
			QPair<int, int> label_pair = it.key();
			QList<PAPartRelation> relations = it.value();
			qDebug("(%d, %d) has %d pairwise part relations.", label_pair.first, label_pair.second, relations.size());
		}

		/* Output the all the CuboidFeatures and CuboidTransformations to files */
		for (int label_index_1 = 0; label_index_1 < m_num_labels; label_index_1++)
		{
			std::stringstream transformation_filename_sstr;
			transformation_filename_sstr << training_dir << std::string("/") << transformation_filename_prefix
				<< label_index_1 << std::string(".csv");
			CuboidTransformation::save_transformation_collection(transformation_filename_sstr.str().c_str(),
				m_transformation_list[label_index_1]);

			std::stringstream feature_filename_sstr;
			feature_filename_sstr << training_dir << std::string("/") << feature_filename_prefix
				<< label_index_1 << std::string(".csv");
			CuboidFeatures::save_feature_collection(feature_filename_sstr.str().c_str(),
				m_feature_list[label_index_1]);
		}

		/* Start to analysis each parts pair */
		this->start();
	}
}

using namespace Eigen;
void TrainPartsThread::analyseProbPartModel()
{
	/* Progressively deal with each parts pair */
	QMap<QPair<int, int>, QList<PAPartRelation>>::iterator it;
	for (it = partRelations.begin(); it != partRelations.end(); ++it)
	{
		QPair<int, int> label_pair = it.key();    /* Part labels of the parts pair */
		std::cout << "Computing mean and covariance of part pair " <<  label_pair.first << "-" << label_pair.second << std::endl;
		QList<PAPartRelation> relations = it.value();    /* The relation vectors of all pairwise parts with the labels above */

		mlpack::distribution::GaussianDistribution normalDistribution(32);
		//std::cout << "partRelations.size() =" << relations.size() << endl;
		arma::mat matrix(32, relations.size());
		int index = 0;
		QList<PAPartRelation>::iterator relation_it;
		for (relation_it = relations.begin(); relation_it != relations.end(); relation_it++)
		{
			std::vector<double> feat_vector = (*relation_it).getFeatureVector();
			QVector<double> feat_qvector = QVector<double>::fromStdVector(feat_vector);
			
			/*for (std::vector<double>::iterator f_it = feat_vector.begin(); f_it != feat_vector.end(); ++f_it)
				cout << *f_it << " ";
			cout << endl;*/

			arma::vec feat(feat_vector);
			arma::rowvec featt((*relation_it).getFeatureVector());

			matrix.col(index++) = feat;
		}

		/* Train a normal distribution of all parts pairs */
		//qDebug("Features matrix is %d x %d.", matrix.n_rows, matrix.n_cols);
		normalDistribution.Train(matrix);
		
		/* Obtain the mean vector and the covariance matrix of the data */
		arma::vec mean_vec = normalDistribution.Mean();
		arma::mat cov_mat = normalDistribution.Covariance();

		/* Print the mean vector and covariance matrix */
		//cout << "Mean vector:" << endl;
		//cout << mean_vec << endl;
		//cout << "Covariance matrix:" << endl;
		//cout << cov_mat << endl;

		/* Create a Eigen vector to represents the mean vector */
		Eigen::VectorXd mean_vector(32);
		double *mean_data = mean_vec.memptr();
		std::memcpy(mean_vector.data(), mean_data, 32 * sizeof(double));
		/* Create a Eigen matrix to represents the covariance matrix */
		Eigen::MatrixXd covariance_matrix(32, 32);
		std::memcpy(covariance_matrix.data(), cov_mat.memptr(), 32 * 32 * sizeof(double));
		
		mean_vecs.insert(label_pair, mean_vector);
		cov_mats.insert(label_pair, covariance_matrix);

		cout << "Computing done." << endl;
	}
}

void TrainPartsThread::savePartRelationPriors()
{
	std::string mean_path = "../data/parts_relations/" + class_name + "_mean_t.txt";
	std::string cov_path = "../data/parts_relations/" + class_name + "_covariance_t.txt";
	ofstream mean_out(mean_path.c_str());
	ofstream cov_out(cov_path.c_str());

	QMap<QPair<int, int>, VectorXd>::iterator mean_it;
	QMap<QPair<int, int>, MatrixXd>::iterator cov_it;
	for (mean_it = mean_vecs.begin(), cov_it = cov_mats.begin(); 
		mean_it != mean_vecs.end() && cov_it != cov_mats.end(); 
		mean_it++, cov_it++)
	{
		QPair<int, int> label_pair = mean_it.key();
		cout << "Write mean and covariance of part pair" << label_pair.first << "-" << label_pair.second << endl;
		/* Output the label pair names*/
		mean_out << label_pair.first << " " << label_pair.second << std::endl;
		cov_out << label_pair.first << " " << label_pair.second << std::endl;

		/* Output the mean vector */
		VectorXd mean_vec = mean_it.value();
		for (int i = 0; i < mean_vec.size() - 1; i++)
			mean_out << mean_vec[i] << " ";
		mean_out << mean_vec[mean_vec.size() - 1] << std::endl;

		/* Output the covariance matrix */
		MatrixXd cov = cov_it.value();
		for (int i = 0; i < cov.rows(); i++)   /* Progressively print the rows of the covariance */
		{
			VectorXd row = cov.row(i);
			for (int j = 0; j < row.size() - 1; j++)
				cov_out << row[j] << " ";
			cov_out << row[row.size() - 1] << std::endl;
		}
	}

	mean_out.close();
	cov_out.close();

	cout << "Writting part relations done." << endl;
}
