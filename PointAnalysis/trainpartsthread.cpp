#include "trainpartsthread.h"

using namespace std;

TrainPartsThread::TrainPartsThread(QObject *parent)
	: QThread(parent), currentId(0)
{
	qRegisterMetaType<QVector<PAPart>>("QVector<PAPart>");
	qDebug("TrainPartsThread is created.");
	emit addDebugText("TrainPartsThread is created.");

	loadThread.setPhase(LoadThread::PHASE::TRAINING);
	
	connect(&loadThread, SIGNAL(loadPointsCompleted(PCModel *)), this, SLOT(receiveModel(PCModel *)));
	connect(&pcaThread, SIGNAL(estimatePartsDone(QVector<PAPart>)), this, SLOT(receiveParts(QVector<PAPart>)));

	ifstream list_in("../data/coseg_chairs_list.txt");
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

	qDebug() << "Load point cloud from" << QString::fromStdString(file_list[currentId]);
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

void TrainPartsThread::receiveModel(PCModel *pc)
{
	qDebug() << "Receive point cloud from" << QString::fromStdString(pc->getInputFilename());

	emit showModel(pc);
	pcaThread.setPointCloud(pc);
	pcaThread.start();
}

void TrainPartsThread::receiveParts(QVector<PAPart> parts)
{
	qDebug() << "Receive parts vectors of point cloud" << QString::fromStdString(file_list[currentId]);

	QVector<int> labels(parts.size());
	QMap<int, int> labels_indices;
	for (int i = 0; i < parts.size(); i++)
	{
		labels[i] = parts[i].getLabel();
		labels_indices.insert(labels[i], i);
	}

	QVector<QPair<int, int>> part_pairs = Utils::getCombinations(labels);
	for (int i = 0; i < part_pairs.size(); i++)
	{
		QPair<int, int> part_pair = part_pairs[i];
		PAPartRelation relation(parts[labels_indices.value(part_pair.first)], parts[labels_indices.value(part_pair.second)]);
		if (!partRelations.contains(part_pair))
			partRelations.insert(part_pair, QList<PAPartRelation>());
		partRelations[part_pair].push_back(relation);
	}

	currentId++;

	if (currentId < file_list.size())   /* If there are point clouds remain unestimating */
	{
		/* Load the next point cloud */
		if (loadThread.isRunning())
			loadThread.terminate();
		qDebug() << "Load point cloud from" << QString::fromStdString(file_list[currentId]);
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
		emit addDebugText("Computing mean and covariance of part pair " + QString::number(label_pair.first) + "-" + QString::number(label_pair.second));
		QList<PAPartRelation> relations = it.value();    /* The relation vectors of all pairwise parts with the labels above */

		mlpack::distribution::GaussianDistribution normalDistribution(32);
		qDebug() << "partRelations.size() =" << relations.size();
		arma::mat matrix(32, relations.size());
		int index = 0;
		QList<PAPartRelation>::iterator relation_it;
		for (relation_it = relations.begin(); relation_it != relations.end(); relation_it++)
		{
			std::vector<double> feat_vector = (*relation_it).getFeatureVector();
			QVector<double> feat_qvector = QVector<double>::fromStdVector(feat_vector);
			qDebug() << feat_qvector;
			arma::vec feat(feat_vector);
			arma::rowvec featt((*relation_it).getFeatureVector());

			matrix.col(index++) = feat;
		}

		/* Train a normal distribution of all parts pairs */
		qDebug("Features matrix is %d x %d.", matrix.n_rows, matrix.n_cols);
		normalDistribution.Train(matrix);
		
		/* Obtain the mean vector and the covariance matrix of the data */
		arma::vec mean_vec = normalDistribution.Mean();
		arma::mat cov_mat = normalDistribution.Covariance();

		/* Create a Eigen vector to represents the mean vector */
		Eigen::VectorXd mean_vector(32);
		double *mean_data = mean_vec.memptr();
		std::memcpy(mean_vector.data(), mean_data, 32 * sizeof(double));
		/* Create a Eigen matrix to represents the covariance matrix */
		Eigen::MatrixXd covariance_matrix(32, 32);
		std::memcpy(covariance_matrix.data(), cov_mat.memptr(), 32 * 32 * sizeof(double));
		
		mean_vecs.insert(label_pair, mean_vector);
		cov_mats.insert(label_pair, covariance_matrix);

		emit addDebugText("Computing done.");
	}
}

void TrainPartsThread::savePartRelationPriors()
{
	std::string mean_path = "../data/parts_relations/" + class_name + "_mean.txt";
	std::string cov_path = "../data/parts_relations/" + class_name + "_covariance.txt";
	ofstream mean_out(mean_path.c_str());
	ofstream cov_out(cov_path.c_str());

	QMap<QPair<int, int>, VectorXd>::iterator mean_it;
	QMap<QPair<int, int>, MatrixXd>::iterator cov_it;
	for (mean_it = mean_vecs.begin(), cov_it = cov_mats.begin(); 
		mean_it != mean_vecs.end() && cov_it != cov_mats.end(); 
		mean_it++, cov_it++)
	{
		QPair<int, int> label_pair = mean_it.key();
		qDebug("Write part pair %d-%d to file.", label_pair.first, label_pair.second);
		emit addDebugText("Write mean and covariance of part pair" + QString::number(label_pair.first) + "-" + QString::number(label_pair.second));
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
}
