#include "obbestimator.h"

OBBEstimator::OBBEstimator()
{

}

using namespace pcl;
OBBEstimator::OBBEstimator(int label, PointCloud<PointXYZ>::Ptr cloud)
{
	m_cloud = cloud;
	m_label = label;
}


OBBEstimator::~OBBEstimator()
{
}

OBB *OBBEstimator::computeOBB()
{
	PCA<PointXYZ> pca_operator;
	pca_operator.setInputCloud(m_cloud);
	/* Compute the eigen values and eigen vectors of the point cloud */
	Eigen::Matrix3f eigen_vectors = pca_operator.getEigenVectors();
	Eigen::Vector3f eigen_values = pca_operator.getEigenValues();

	/* Project all the points into the local system whose axes are the eigen vectors */
	PointCloud<pcl::PointXYZ>::Ptr cloudPCAprojection(new pcl::PointCloud<pcl::PointXYZ>);
	pca_operator.project(*m_cloud, *cloudPCAprojection);

	/* Three eigen vectors, namely the axes of the local system of the OBB */
	Eigen::Vector3f eigen_vector0 = eigen_vectors.col(0);
	Eigen::Vector3f eigen_vector1 = eigen_vectors.col(1);
	Eigen::Vector3f eigen_vector2 = eigen_vectors.col(2);
	/* Three global system axes */
	Eigen::Matrix3f global_axes = Eigen::Matrix3f::Identity();
	Eigen::Matrix3f local_axes;

	/* Specify the eigen vector closest to each global axis as each axis of the local system of OBB */
	Eigen::Matrix<float, 6, 3> eigen_vectors_transpose;
	for (int i = 0; i < 3; i++)
	{
		eigen_vectors_transpose.row(2 * i) = eigen_vectors.col(i).transpose();
		eigen_vectors_transpose.row(2 * i + 1) = (-eigen_vectors.col(i)).transpose();
	}
	Eigen::Matrix<float, 6, 3> angles = eigen_vectors_transpose * global_axes;
	int axes_indices[3] = { 0, 0, 0 };
	for (int i = 0; i < 3; i++)
	{
		Eigen::VectorXf v = angles.col(i);
		float max = -1.0;
		for (int j = 0; j < 6; j++)
		{
			if (v[j] > max)
			{
				max = v[j];
				axes_indices[i] = j / 2;
				local_axes.col(i) = std::pow(-1.0, j % 2) * eigen_vectors.col(j / 2);
			}
		}
	}

	/* Find out the spans on three axes */
	float min[3] = { 100.0, 100.0, 100.0 };
	float max[3] = { -100.0, -100.0, -100.0 };
	for (int i = 0; i < cloudPCAprojection->size(); i++)
	{
		PointXYZ point = cloudPCAprojection->at(i);
		if (point.x < min[0])
			min[0] = point.x;
		if (point.x > max[0])
			max[0] = point.x;
		if (point.y < min[1])
			min[1] = point.y;
		if (point.y > max[1])
			max[1] = point.y;
		if (point.z < min[2])
			min[2] = point.z;
		if (point.z > max[2])
			max[2] = point.z;
	}
	float x_length = max[axes_indices[0]] - min[axes_indices[0]];
	float y_length = max[axes_indices[1]] - min[axes_indices[1]];
	float z_length = max[axes_indices[2]] - min[axes_indices[2]];

	PointXYZ pca_centroid((min[0] + max[0]) / 2.0, (min[1] + max[1]) / 2.0, (min[2] + max[2]) / 2.0);  /* The centroid of the OBB in the local system */
	PointXYZ obb_centroid;    /* The centroid of the OBB in the gloabal system */
	/* Obtain the centroid of OBB in global system from that in the local system */
	pca_operator.reconstruct(pca_centroid, obb_centroid);

	QVector3D x_axis(local_axes.col(0).x(), local_axes.col(0).y(), local_axes.col(0).z());
	QVector3D y_axis(local_axes.col(1).x(), local_axes.col(1).y(), local_axes.col(1).z());
	QVector3D z_axis(local_axes.col(2).x(), local_axes.col(2).y(), local_axes.col(2).z());
	QVector3D centroid(obb_centroid.x, obb_centroid.y, obb_centroid.z);


	return new OBB(x_axis, y_axis, z_axis, centroid, x_length, y_length, z_length, m_label); 
}

QVector<OBB *> OBBEstimator::computeOBBCandidates()
{
	PCA<PointXYZ> pca_operator;
	pca_operator.setInputCloud(m_cloud);
	/* Compute the eigen values and eigen vectors of the point cloud */
	Eigen::Matrix3f eigen_vectors = pca_operator.getEigenVectors();
	Eigen::Vector3f eigen_values = pca_operator.getEigenValues();

	/* Project all the points into the local system whose axes are the eigen vectors */
	PointCloud<pcl::PointXYZ>::Ptr cloudPCAprojection(new pcl::PointCloud<pcl::PointXYZ>);
	pca_operator.project(*m_cloud, *cloudPCAprojection);

	/* Find out the spans on three local axes */
	float min[3] = { 100.0, 100.0, 100.0 };
	float max[3] = { -100.0, -100.0, -100.0 };
	for (int i = 0; i < cloudPCAprojection->size(); i++)
	{
		PointXYZ point = cloudPCAprojection->at(i);
		if (point.x < min[0])
			min[0] = point.x;
		if (point.x > max[0])
			max[0] = point.x;
		if (point.y < min[1])
			min[1] = point.y;
		if (point.y > max[1])
			max[1] = point.y;
		if (point.z < min[2])
			min[2] = point.z;
		if (point.z > max[2])
			max[2] = point.z;
	}

	PointXYZ pca_centroid((min[0] + max[0]) / 2.0, (min[1] + max[1]) / 2.0, (min[2] + max[2]) / 2.0);  /* The centroid of the OBB in the local system */
	PointXYZ obb_centroid;    /* The centroid of the OBB in the gloabal system */
	/* Obtain the centroid of OBB in global system from that in the local system */
	pca_operator.reconstruct(pca_centroid, obb_centroid);

	/* Generate 24 candidate OBB. 
	 * each possible choice of assigning PCA-aces to a right-handed coordinate system provides one hypothesis.
	 */
	QVector<OBB *> obb_candidates(24);
	Eigen::Matrix3f local_axes;
	int axes_indices[3];
	double x_length = 0, y_length = 0, z_length = 0;
	int candidate_count = 0;
	for (int i = 0; i < 6; i++)    /* There are 6 choises for x-axis */
	{
		local_axes.col(0) = std::pow(-1.0, i) * eigen_vectors.col(i / 2);
		axes_indices[0] = i / 2;
		for (int j = 0; j < 6; j++)
		{
			if (j != i / 2 * 2 && j != i / 2 * 2 + 1)    /* There are 4 choises for y_axis */
			{
				local_axes.col(1) = std::pow(-1.0, j) * eigen_vectors.col(j / 2);
				axes_indices[1] = j / 2;
				axes_indices[2] = 3 - i / 2 - j / 2;
				x_length = max[axes_indices[0]] - min[axes_indices[0]];
				y_length = max[axes_indices[1]] - min[axes_indices[1]];
				z_length = max[axes_indices[2]] - min[axes_indices[2]];
				/* Once x-axis and y-axis are determined, the z-axis only has one possible choise */
				if (i % 2 == 0)
					local_axes.col(2) = std::pow(-1.0, j) * eigen_vectors.col(3 - i / 2 - j / 2);
				else
					local_axes.col(2) = std::pow(-1.0, j + 1) * eigen_vectors.col(3 - i / 2 - j / 2);
			
				/* Create a candidate OBB */
				obb_candidates[candidate_count++] = new OBB(QVector3D(local_axes.col(0).x(), local_axes.col(0).y(), local_axes.col(0).z()),  /* The x axis */
					QVector3D(local_axes.col(1).x(), local_axes.col(1).y(), local_axes.col(1).z()),    /* The y axis */
					QVector3D(local_axes.col(2).x(), local_axes.col(2).y(), local_axes.col(2).z()),    /* The z axis */
					QVector3D(obb_centroid.x, obb_centroid.y, obb_centroid.z),     /* the centroid of the OBB */
					x_length, y_length, z_length, m_label);
			}
		}
	}
	return obb_candidates;
}

void OBBEstimator::reset(int label, PointCloud<PointXYZ>::Ptr cloud)
{
	m_label = label;
	m_cloud = cloud;
}