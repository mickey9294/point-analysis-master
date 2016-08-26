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

	/* Normalize the eigen vectors */
	for (int i = 0; i < eigen_vectors.cols(); i++)
		eigen_vectors.col(i).normalize();

	/* Three eigen vectors, namely the axes of the local system of the OBB */
	Eigen::Vector3f eigen_vector0 = eigen_vectors.col(0);
	Eigen::Vector3f eigen_vector1 = eigen_vectors.col(1);
	Eigen::Vector3f eigen_vector2 = eigen_vectors.col(2);
	/* Three global system axes */
	Eigen::Matrix3f global_axes = Eigen::Matrix3f::Identity();
	Eigen::Matrix3f local_axes;

	/* Specify the eigen vectors closests to the global axes as the local system axes.
	 * Particularly, choose the local system with the minimal sum of rotation angles on three system axes
	 */
	float min_angle = 720;
	int axes_indices[3] = { 0, 0, 0 };
	for (int i = 0; i < 6; i++)
	{
		Eigen::Vector3f x_axis = std::pow(-1.0, i) * eigen_vectors.col(i / 2);

		for (int j = 0; j < 6; j++)
		{
			if (j != i / 2 * 2 && j != i / 2 * 2 + 1)    /* There are 4 choises for y_axis */
			{
				Eigen::Vector3f y_axis = std::pow(-1.0, j) * eigen_vectors.col(j / 2);

				Eigen::Vector3f z_axis = x_axis.cross(y_axis);
				z_axis.normalize();

				/* Commpute the sum of rotation angles of 3 local system axes to global system axes */
				float x_angle = std::acos(x_axis.dot(global_axes.col(0)));
				float y_angle = std::acos(y_axis.dot(global_axes.col(1)));
				float z_angle = std::acos(z_axis.dot(global_axes.col(2)));
				float angle_sum = x_angle + y_angle + z_angle;

				if (angle_sum < min_angle)
				{
					min_angle = angle_sum;
					local_axes.col(0) = x_axis;
					local_axes.col(1) = y_axis;
					local_axes.col(2) = z_axis;

					axes_indices[0] = i / 2;
					axes_indices[1] = j / 2;
					axes_indices[2] = 3 - i / 2 - j / 2;
				}
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

	Eigen::Vector3f centroid(obb_centroid.x, obb_centroid.y, obb_centroid.z);

	OBB *obb = new OBB(local_axes.col(0), local_axes.col(1), local_axes.col(2), centroid, x_length, y_length, z_length, m_label);
	obb->triangulate();
	if (m_phase == PHASE::TRAINING)
		obb->normalize(m_cloud);

	return obb; 
}

OBB * OBBEstimator::computeOBB_PCA()
{
	using namespace Eigen;

	int npoints = m_cloud->size();
	
	MatrixXf data(3, npoints);

	for (int i = 0; i < npoints; i++)
	{
		PointXYZ point = m_cloud->at(i);
		Vector3f point_vec(point.x, point.y, point.z);
		data.col(i) = point_vec;
	}

	MatrixXf M = data.transpose();
	MatrixXf centered = M.rowwise() - M.colwise().mean();
	MatrixXf covariance = centered.adjoint() * centered / float(npoints - 1);

	EigenSolver<MatrixXf> es(covariance);
	//VectorXf eigen_values = es.eigenvalues();
	MatrixXcf eigen_vectors_complex = es.eigenvectors();
	MatrixXf eigen_vectors(eigen_vectors_complex.rows(), eigen_vectors_complex.cols());

	/* Normalize the eigen vectors */
	for (int i = 0; i < eigen_vectors_complex.cols(); i++)
	{
		eigen_vectors.col(i) = eigen_vectors_complex.col(i).real();
		eigen_vectors.col(i).normalize();
	}

	Matrix3f global_axes = Matrix3f::Identity();
	Matrix3f local_axes;

	/* Specify the eigen vectors closests to the global axes as the local system axes.
	* Particularly, choose the local system with the minimal sum of rotation angles on three system axes
	*/
	float min_angle = 720;
	int axes_indices[3] = { 0, 0, 0 };
	for (int i = 0; i < 6; i++)
	{
		Eigen::Vector3f x_axis = std::pow(-1.0, i) * eigen_vectors.col(i / 2);

		for (int j = 0; j < 6; j++)
		{
			if (j != i / 2 * 2 && j != i / 2 * 2 + 1)    /* There are 4 choises for y_axis */
			{
				Eigen::Vector3f y_axis = std::pow(-1.0, j) * eigen_vectors.col(j / 2);

				Eigen::Vector3f z_axis = x_axis.cross(y_axis);
				z_axis.normalize();

				/* Commpute the sum of rotation angles of 3 local system axes to global system axes */
				float x_angle = std::acos(x_axis.dot(global_axes.col(0)));
				float y_angle = std::acos(y_axis.dot(global_axes.col(1)));
				float z_angle = std::acos(z_axis.dot(global_axes.col(2)));
				float angle_sum = x_angle + y_angle + z_angle;

				if (angle_sum < min_angle)
				{
					min_angle = angle_sum;
					local_axes.col(0) = x_axis;
					local_axes.col(1) = y_axis;
					local_axes.col(2) = z_axis;

					axes_indices[0] = i / 2;
					axes_indices[1] = j / 2;
					axes_indices[2] = 3 - i / 2 - j / 2;
				}
			}
		}
	}

	/* Find out the spans on three axes */
	float min[3] = { 100.0, 100.0, 100.0 };
	float max[3] = { -100.0, -100.0, -100.0 };
	Matrix3f transform_mat = local_axes.inverse();
	for (int i = 0; i < npoints; i++)
	{
		Vector3f p = data.col(i);
		/* transform the point in the original system to the local system */
		Vector3f point = transform_mat * p;

		if (point.x() < min[0])
			min[0] = point.x();
		if (point.x() > max[0])
			max[0] = point.x();
		if (point.y() < min[1])
			min[1] = point.y();
		if (point.y() > max[1])
			max[1] = point.y();
		if (point.z() < min[2])
			min[2] = point.z();
		if (point.z() > max[2])
			max[2] = point.z();
	}
	float x_length = max[0] - min[0];
	float y_length = max[1] - min[1];
	float z_length = max[2] - min[2];

	/* Compute the centroid of the OBB */
	float x_centroid = (max[0] + min[0]) / 2.0;
	float y_centroid = (max[1] + min[1]) / 2.0;
	float z_centroid = (max[2] + min[2]) / 2.0;
	Vector3f centroid_local(x_centroid, y_centroid, z_centroid);
	Vector3f obb_centroid = local_axes * centroid_local;

	QVector3D xaxis(local_axes.col(0).x(), local_axes.col(0).y(), local_axes.col(0).z());
	QVector3D yaxis(local_axes.col(1).x(), local_axes.col(1).y(), local_axes.col(1).z());
	QVector3D zaxis(local_axes.col(2).x(), local_axes.col(2).y(), local_axes.col(2).z());
	Eigen::Vector3f centroid(obb_centroid.x(), obb_centroid.y(), obb_centroid.z());
	
	return new OBB(local_axes.col(0), local_axes.col(1), local_axes.col(2), centroid, x_length, y_length, z_length, m_label);
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
	Eigen::Matrix3f global_axes = Eigen::Matrix3f::Identity();    /* Three global system axes */

	QVector<OBB *> obb_candidates(24);
	Eigen::Matrix3f local_axes;
	int axes_indices[3];
	double x_length = 0, y_length = 0, z_length = 0;
	int best_idx = 0;   /* The index of the local system which is cloest to the global one */
	float best_angle = 1000;    /* The sum of rotation angles of the best local system to the global system */
	int index = 0;    /* The current index of current candidate local system */
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
				Eigen::Vector3f zaxis = local_axes.col(0).cross(local_axes.col(1));
				zaxis.normalize();
				local_axes.col(2) = zaxis;

				/* Commpute the sum of rotation angles of 3 local system axes to global system axes */
				float x_angle = std::acos(local_axes.col(0).dot(global_axes.col(0)));
				float y_angle = std::acos(local_axes.col(1).dot(global_axes.col(1)));
				float z_angle = std::acos(local_axes.col(2).dot(global_axes.col(2)));
				float angle_sum = x_angle + y_angle + z_angle;
				/*if (m_label == 1)
				{
					std::cout << local_axes.transpose() << std::endl;
					std::cout << "angle_sum = " << angle_sum;
					if (angle_sum < best_angle)
						std::cout << ".  Best angle so far!" << std::endl;
					else
						std::cout << "." << std::endl;
				}*/
				/* If the sum of ratation angles are the smallest, set the best local system to the current one */
				if (angle_sum < best_angle)
				{
					best_angle = angle_sum;
					best_idx = index;
				}
	
				/* Create a candidate OBB */
				OBB * obb_cand = new OBB(local_axes.col(0),  /* The x axis */
					local_axes.col(1),    /* The y axis */
					local_axes.col(2),    /* The z axis */
					Eigen::Vector3f(obb_centroid.x, obb_centroid.y, obb_centroid.z),     /* the centroid of the OBB */
					x_length, y_length, z_length, m_label);
				//ICP_pcl(obb_cand);
				obb_candidates[index++] = obb_cand;
			}
		}
	}

	/* Exchange the positions of the first candidate OBB and the candidate with the local system closest to the global system */
	if (best_idx != 0)
	{
		OBB * temp = obb_candidates[0];
		obb_candidates[0] = obb_candidates[best_idx];
		obb_candidates[best_idx] = temp;
	}

	return obb_candidates;
}

void OBBEstimator::reset(int label, PointCloud<PointXYZ>::Ptr cloud)
{
	m_label = label;
	m_cloud = cloud;
}

using namespace Eigen;

void OBBEstimator::ICP_procedure(OBB *obb)
{
	int sample_size = 0;
	if (obb->sampleCount() < 3)
		sample_size = obb->createGridSamples();
	else
		sample_size = obb->sampleCount();

	MatrixXd sample_points(3, sample_size);
	MatrixXd part_points(3, m_cloud->size());

	/* Form a data matrix for sample points on OBB */
	QVector<Vector3f> samples = obb->getSamplePoints();
	int idx = 0;
	for (QVector<Vector3f>::iterator sample_it = samples.begin(); sample_it != samples.end(); ++sample_it)
	{
		Vector3f p = *sample_it;
		Vector3d point(p.x(), p.y(), p.z());
		sample_points.col(idx++) = point;
	}

	/* Form a data matrix for the points of the part(model) */
	idx = 0;
	for (pcl::PointCloud<pcl::PointXYZ>::iterator part_it = m_cloud->begin(); part_it != m_cloud->end(); ++part_it)
	{
		pcl::PointXYZ p = *part_it;
		Vector3d point(p.x, p.y, p.z);
		part_points.col(idx++) = point;
	}

	Matrix3d rotation_mat;
	Vector3d translation_vec;

	/* Execute ICP procedure */
	double error = ICP::run_iterative_closest_points(sample_points, part_points, rotation_mat, translation_vec);

	std::cout << "For part_" << m_label << "'s OBB, rotation matrix:" << std::endl;
	std::cout << rotation_mat << std::endl;
	std::cout << "translation vector:" << std::endl; 
	std::cout << translation_vec.transpose() << std::endl;

	//obb->setSamplePoints(sample_points);
	//obb->rotate(rotation_mat, translation_vec, m_cloud);
}

void OBBEstimator::ICP_pcl(OBB* obb)
{
	std::cout << "Use ICP to adjust OBB_" << m_label << std::endl;
	using namespace pcl;
	PointCloud<PointXYZ>::Ptr cloud_in(new PointCloud<PointXYZ>);
	PointCloud<PointXYZ>::Ptr cloud_out(new PointCloud<PointXYZ>);

	/* Set the samples on the OBB as the input point cloud */
	int sample_size = 0;
	if (obb->sampleCount() < 3)
		sample_size = obb->createGridSamples();
	else
		sample_size = obb->sampleCount();

	cloud_in->width = sample_size;
	cloud_in->height = 1;
	cloud_in->is_dense = false;
	cloud_in->points.resize(cloud_in->width * cloud_in->height);

	int sample_idx = 0;
	for (QVector<SamplePoint>::iterator sample_it = obb->samples_begin(); sample_it != obb->samples_end(); ++sample_it)
	{
		cloud_in->points[sample_idx].x = sample_it->x();
		cloud_in->points[sample_idx].y = sample_it->y();
		cloud_in->points[sample_idx++].z = sample_it->z();
	}

	/* Set the points on the model as the target point cloud */
	cloud_out->width = m_cloud->size();
	cloud_out->height = 1;
	cloud_out->is_dense = false;
	cloud_out->points.resize(cloud_out->width * cloud_out->height);

	int point_idx = 0;
	for (PointCloud<PointXYZ>::iterator point_it = m_cloud->begin(); point_it != m_cloud->end(); ++point_it)
	{
		cloud_out->points[point_idx].x = point_it->x;
		cloud_out->points[point_idx].y = point_it->y;
		cloud_out->points[point_idx++].z = point_it->z;
	}
	
	/* Execute ICP algorithm */
	IterativeClosestPoint<PointXYZ, PointXYZ> icp;
	icp.setInputCloud(cloud_in);
	icp.setInputTarget(cloud_out);
	PointCloud<PointXYZ> Final;
	icp.align(Final);
	std::cout << "has converged: " << icp.hasConverged() << "; score: " << icp.getFitnessScore() << std::endl;
	Matrix4f transform_mat = icp.getFinalTransformation();

	//obb->setSamplePoints(Final);
	obb->transform(transform_mat, m_cloud);
}

void OBBEstimator::setPhase(PHASE phase)
{
	m_phase = phase;
}