#include "energyfunctions.h"

using namespace std;
using namespace Eigen;

float EnergyFunctions::w1 = 25;
float EnergyFunctions::w2 = 1.0;
float EnergyFunctions::w3 = 1.0;
float EnergyFunctions::w4 = 1.0;
float EnergyFunctions::w5 = 100.0;

EnergyFunctions::EnergyFunctions(string modelClassName) : m_modelClassName(modelClassName), m_null_label(10)
{
	cout << "Consruct EnegerFunctions" << endl;
	/* Load the part relations priors from file */
	const int DIMENSION = 32;
	//string covariance_path = "../data/parts_relations/" + m_modelClassName + "_covariance.txt";
	//string mean_path = "../data/parts_relations/" + m_modelClassName + "_mean.txt";
	string covariance_path = "../data/parts_relations/coseg_chairs_covariance_t.txt";
	string mean_path = "../data/parts_relations/coseg_chairs_mean_t.txt";

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

				covariance.setIdentity();
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
	
	/* Load the symmetry groups */
	/* Load symmetry groups from file */
	std::ifstream sym_in("../data/symmetry_groups.txt");
	if (sym_in.is_open())
	{
		char buffer[128];
		while (!sym_in.eof())
		{
			sym_in.getline(buffer, 128);
			if (strlen(buffer) > 0)
			{
				QStringList sym_str_list = QString(buffer).split(' ');
				QVector<int> group(sym_str_list.size());
				for (int i = 0; i < sym_str_list.size(); i++)
				{
					int label = sym_str_list[i].toInt();
					group[i] = label;
					m_symmetry_set.insert(label);
				}
				m_symmetry_groups.push_back(group);
			}
		}
	}
}

EnergyFunctions::~EnergyFunctions()
{
}

void EnergyFunctions::setPointCloud(PAPointCloud * pointcloud)
{
	m_pointcloud = pointcloud;
}

void EnergyFunctions::setOBBs(QMap<int, OBB *> obbs)
{
	using namespace pcl;

	/* Delete the previous used obbs */
	for (QMap<int, OBB *>::iterator obb_it = m_obbs.begin(); obb_it != m_obbs.end(); ++obb_it)
	{
		if (*obb_it != NULL)
			delete(*obb_it);
		*obb_it = NULL;
	}
	m_obbs.clear();

	/* Set OBBs and generate kd-trees for each part */
	m_kdtrees.clear();  /* Clear the previous used kd-trees */
	int tree_idx = 0;
	for (QMap<int, OBB *>::iterator obb_it = obbs.begin(); obb_it != obbs.end(); ++obb_it)
	{
		int label = obb_it.key();
		if ((*obb_it)->sampleCount() < 3)  /* If the OBB has not been sampled */
			(*obb_it)->samplePoints();    /* Sample point on the OBB */

		m_obbs.insert(label, *obb_it);

		/* Generate pointcloud data for the kdtree */
		PointCloud<PointXYZ>::Ptr cloud(new PointCloud<PointXYZ>);
		cloud->width = (*obb_it)->sampleCount();
		cloud->height = 1;
		cloud->points.resize(cloud->width * cloud->height);

		int sample_idx = 0;
		for (QVector<Eigen::Vector3f>::iterator sample_it = (*obb_it)->samples_begin();
			sample_it != (*obb_it)->samples_end(); ++sample_it)
		{
			cloud->points[sample_idx].x = sample_it->x();
			cloud->points[sample_idx].y = sample_it->y();
			cloud->points[sample_idx++].z = sample_it->z();
		}

		KdTreeFLANN<PointXYZ> kdtree;
		kdtree.setInputCloud(cloud);
		m_kdtrees.insert(label, kdtree);
	}
}

void EnergyFunctions::setPointAssignments(QVector<int> point_assignments)
{
	m_point_assignments = point_assignments;
}

QVector<int> EnergyFunctions::getPointAssignments() const
{
	return m_point_assignments;
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

	/* Decide whether the assumed label is contained in a symmetry group and which group it belongs to */
	QVector<int> symmetry_group;
	if (m_symmetry_set.contains(label))    /* The label represents a symmetric part */
	{
		for (QList<QVector<int>>::iterator it = m_symmetry_groups.begin(); it != m_symmetry_groups.end(); ++it)
		{
			QVector<int> group = *it;
			if (group.contains(label))
			{
				symmetry_group.resize(group.size());
				std::memcpy(symmetry_group.data(), group.data(), group.size() * sizeof(int));
			}
		}
	}

	for (int i = 0; i < vertices_indices.size(); i++)
	{
		PAPoint point = m_pointcloud->at(vertices_indices[i]);
		QMap<int, float> distribution = m_distributions[vertices_indices[i]];
		float score = 0;    /* The classification score of the assumed label */

		if (symmetry_group.size() > 0)    /* If the label represents a symmetric part */
		{
			/* The score equals to the sum of scores of all symmetric parts in the group */
			for (QVector<int>::iterator it = symmetry_group.begin(); it != symmetry_group.end(); ++it)
				score += distribution.value(*it);
		}
		else    /* If the label rerpresents a umsymmetric part */
			score = distribution.value(label);    /* The score is just the score of the label */

		double e;
		if (Utils::float_equal(score, 0.0))
			e = 1e8;
		else
			e = -log(score);
		energy += e;
		count++;
	}

	energy /= (double)count;
	energy *= w1;
	return energy;
}

double EnergyFunctions::Epnt_single(int point_index, int label)
{
	float probability = m_distributions[point_index][label];
	double energy = -log(probability); 
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
			return  1e8;
		else
			return 0;
	}
	else
	{
		/* If either of the assumed labels is null label, set the energy value to 0 */
		if (label1 == m_null_label || label2 == m_null_label)
			return 1e4;
		/* If two assumed labels are the same, set the energy value to infinity */
		if (label1 == label2 || !m_mean_vectors.contains(QPair<int, int>(label1, label2)))
			return 1e8;

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

double EnergyFunctions::Ep_q(int point_index, int assigned_label)
{
	/* Find the nearest neighbor of the point */
	pcl::KdTreeFLANN<pcl::PointXYZ> kdtree = m_kdtrees[assigned_label];
	PAPoint point = m_pointcloud->operator[](point_index);
	pcl::PointXYZ searchPoint(point.x(), point.y(), point.z());

	int K = 1;
	std::vector<int> pointIdxNKNSearch(K);
	std::vector<float> pointNKNSquaredDistance(K);

	kdtree.nearestKSearch(searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance);
	return w3 * pointNKNSquaredDistance.front();
}

QVector<Eigen::Triplet<double>> EnergyFunctions::Esmooth(int point_index)
{
	double energy = 0;
	float radius = m_pointcloud->getRadius() * 0.05;
	const int param_num_point_neighbors = 8;
	int num_of_neighbors = min(param_num_point_neighbors, m_pointcloud->size());

	PAPoint point = m_pointcloud->operator[](point_index);
	pcl::PointXYZ searchPoint(point.x(), point.y(), point.z());

	QVector<Triplet<double>> pair_potentials;

	 /* Find the neighbors of the point */
	vector<int> pointIdxRadiusSearch;
	vector<float> pointRadiusSquredDistance;

	if (m_cloud_kdtree.radiusSearch(searchPoint, radius, pointIdxRadiusSearch, pointRadiusSquredDistance) > num_of_neighbors)
	{
		//assert(pointIdxRadiusSearch.size() == pointRadiusSquredDistance.size());
		for (int i = 0; i < pointIdxRadiusSearch.size(); i++)
		{
			int n_point_index = pointIdxRadiusSearch[i];

			/* Avoid symmetric pairs */
			if (n_point_index <= point_index)
				continue;

			double distance = radius - sqrt(pointRadiusSquredDistance[i]);
			double energy = w2 * distance * distance;
			pair_potentials.push_back(Triplet<double>(point_index, n_point_index, energy));
		}
	}
	else
	{
		vector<int> piontIdxNKNSearch(num_of_neighbors);
		vector<float> pointNKNSquaredDistance(num_of_neighbors);

		m_cloud_kdtree.nearestKSearch(searchPoint, num_of_neighbors, pointIdxRadiusSearch, pointNKNSquaredDistance);
		for (int i = 0; i < pointIdxRadiusSearch.size(); i++)
		{
			int n_point_index = pointIdxRadiusSearch[i];

			/* Avoid symmetric pairs  and exclude the situation that 2 points have the same part assignments */
			if (n_point_index <= point_index || m_point_assignments[point_index] == m_point_assignments[n_point_index])
				continue;

			double distance = radius - sqrt(pointNKNSquaredDistance[i]);
			double energy = w2 * distance * distance;
			pair_potentials.push_back(Triplet<double>(point_index, n_point_index, energy));
		}
	}

	return pair_potentials;
}

void EnergyFunctions::generateKdTree()
{
	using namespace pcl;

	PointCloud<PointXYZ>::Ptr cloud(new PointCloud<PointXYZ>);

	cloud->width = m_pointcloud->size();
	cloud->height = 1;
	cloud->points.resize(cloud->width * cloud->height);
	
	int point_idx = 0;
	for (std::vector<PAPoint>::iterator point_it = m_pointcloud->begin(); point_it != m_pointcloud->end(); ++point_it)
	{
		cloud->points[point_idx].x = point_it->x();
		cloud->points[point_idx].y = point_it->y();
		cloud->points[point_idx++].z = point_it->z();
	}

	m_cloud_kdtree.setInputCloud(cloud);
}