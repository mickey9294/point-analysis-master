#include "energyfunctions.h"

using namespace std;
using namespace Eigen;

float EnergyFunctions::w1 = 0.0025;
float EnergyFunctions::w2 = 1.0;
float EnergyFunctions::w3 = 1.0;
float EnergyFunctions::w4 = 1e-4;
float EnergyFunctions::w5 = 100.0;

EnergyFunctions::EnergyFunctions(string modelClassName) : m_modelClassName(modelClassName), m_null_label(10)
{
	cout << "Consruct EnegerFunctions" << endl;
	/* Load the part relations priors from file */
	const int DIMENSION = 32;
	string covariance_path = "../data/parts_relations/" + m_modelClassName + "_covariance.txt";
	string mean_path = "../data/parts_relations/" + m_modelClassName + "_mean.txt";
	ifstream cov_in(covariance_path.c_str());
	ifstream mean_in(mean_path.c_str());

	/* Load the covariance matrices of each parts pair */
	if (cov_in.is_open())
	{
		char buffer[512];
		while (!cov_in.eof())
		{
			/* Read the part labels pair */
			cov_in.getline(buffer, 512);
			int label1, label2;
			if (strlen(buffer) > 0)
			{
				QString labels(buffer);
				label1 = labels.section(' ', 0, 0).toInt();
				label2 = labels.section(' ', 1, 1).toInt();


				/* Read covariance matrix row by row */
				MatrixXf covariance(DIMENSION, DIMENSION);
				for (int i = 0; i < DIMENSION; i++)
				{
					VectorXf row(DIMENSION);
					cov_in.getline(buffer, 512);
					QStringList row_str = QString(buffer).split(' ');
					for (int j = 0; j < DIMENSION; j++)
						row(j) = row_str[j].toFloat();

					covariance.row(i) = row;
				}

				m_covariance_matrices.insert(QPair<int, int>(label1, label2), covariance);
			}
		}

		cov_in.close();
	}

	/* Load the mean vectors of each parts pair */
	if (mean_in.is_open())
	{
		char buffer[512];
		while (!mean_in.eof())
		{
			/* Read the part labels pair */
			mean_in.getline(buffer, 512);
			int label1, label2;
			if (strlen(buffer) > 0)
			{
				QString labels(buffer);
				label1 = labels.section(' ', 0, 0).toInt();
				label2 = labels.section(' ', 1, 1).toInt();


				/* Read the mean vectors */
				VectorXf mean_vec(DIMENSION);
				mean_in.getline(buffer, 512);
				QStringList vec_str = QString(buffer).split(' ');
				for (int j = 0; j < vec_str.size(); j++)
					mean_vec(j) = vec_str[j].toFloat();

				m_mean_vectors.insert(QPair<int, int>(label1, label2), mean_vec);
			}
		}

		mean_in.close();
	}
}

EnergyFunctions::~EnergyFunctions()
{
}

void EnergyFunctions::setPointCloud(PAPointCloud * pointcloud)
{
	m_pointcloud = pointcloud;
}

void EnergyFunctions::setDistributions(QVector<QMap<int, float>> distributions)
{
	m_distributions = QVector<QMap<int, float>>(distributions);
	m_null_label = m_distributions[0].keys().last();
}

double EnergyFunctions::Epnt(PAPart part, int label)
{
	double energy = 0;
	int count = 0;

	vector<int> vertices_indices = part.getVerticesIndices();  /* The indices of the points belonging to the part in m_pointcloud */

	//for (int i = 0; i < m_pointcloud->size(); i++)
	//{
	//	PAPoint point = m_pointcloud->at(i);
	//	QMap<int, float> distribution = m_distributions[i];
	//	if (part.isInside(point.getPosition()))
	//	{
	//		double e = -log(distribution.value(label));
	//		energy += e;
	//		count++;
	//	}
	//}

	for (int i = 0; i < vertices_indices.size(); i++)
	{
		PAPoint point = m_pointcloud->at(vertices_indices[i]);
		QMap<int, float> distribution = m_distributions[vertices_indices[i]];
		float score = distribution.value(label);
		double e;
		if (Utils::float_equal(score, 0.0))
			e = INF;
		else
			e = -log(score);
		energy += e;
		count++;
	}

	energy /= (double)count;
	energy *= w1;
	return energy;
}

double EnergyFunctions::Epair(PAPartRelation relation, int cluster_no_1, int cluster_no_2, int label1, int label2)
{
	//cout << "Compute Epair of Cand_" << part1.getClusterNo() << " - Cand_" << part2.getClusterNo() 
		//<< " of label " << label1 << " and " << label2 << endl;
	/* If two candidate parts belong to the same point cluster */
	if (cluster_no_1 == cluster_no_2)
	{
		if (label1 != m_null_label && label2 != m_null_label)
			return  3.3e33;
		else
			return 0;
	}
	else
	{
		/* If either of the assumed labels is null label, set the energy value to 0 */
		if (label1 == m_null_label || label2 == m_null_label)
			return INF / 2.0;
		/* If two assumed labels are the same, set the energy value to infinity */
		if (label1 == label2 || !m_mean_vectors.contains(QPair<int, int>(label1, label2)))
			return INF;

		/* If the two assumed labels are the labels of real parts */
		//PAPartRelation relation(part1, part2);
		VectorXf relation_vec(relation.getDimension());

		std::vector<float> relation_feature = relation.getFeatureVector_Float();
		for (int i = 0; i < 32; i++)
			relation_vec(i) = relation_feature[i];

		VectorXf mean_vec = m_mean_vectors.value(QPair<int, int>(label1, label2));
		MatrixXf covariance = m_covariance_matrices.value(QPair<int, int>(label1, label2));
		MatrixXf cov_inverse = covariance.inverse();
		VectorXf std_mean = relation_vec - mean_vec;

		float energy_square_without_w = std_mean.transpose() * cov_inverse * std_mean;
		//cout << "energy_square = " << energy_square << endl;
		float energy = w4 * std::sqrt(energy_square_without_w);
		//cout << "Epair = " << energy << endl;

		return energy;
	}
}