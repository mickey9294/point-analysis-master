#include "pointfeaturethread.h"

PointFeatureThread::PointFeatureThread(int super, int idno, pcl::PointCloud<pcl::PointXYZ>::Ptr c, pcl::PointCloud<pcl::Normal>::Ptr n,
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree, float co, double rad, int start, int e, QObject *parent)
	: QThread(parent)
{
	qDebug("PointFeatureThread-%d-%d is created.", super, idno);
	QString dtext = "PointFeatureThread-" + QString::number(super) + "-" + QString::number(idno) + " is created";
	emit addDebugText(dtext);

	cloud = c;
	normals = n;
	kdtree = tree;
	begin = start;
	end = e;
	id = idno;
	superid = super;
	coef = co;
	radius = rad;
}

PointFeatureThread::~PointFeatureThread()
{
	if (isRunning())
		terminate();
}

void PointFeatureThread::run()
{
	QList<QVector<double>> points_feats;
	estimate(points_feats);
	emit estimateCompleted(id, points_feats);
}

using namespace pcl;
using namespace Eigen;

void PointFeatureThread::estimate(QList<QVector<double>> &points_feats)
{
	/* Compute the geometry features of each point in the sub point cloud */
	qDebug("PointFeatureThread-%d-%d: Computing geometry features for each point...", superid, id);
	QString dtext = "PointFeatureThread-" + QString::number(superid) + "-" + QString::number(id) + ": Computing geometry features for each point...";
	emit addDebugText(dtext);

	std::vector<int> pointIdxRadiusSearch;
	std::vector<float> pointRadiusSquaredDistance;

	for (int i = begin; i <= end; i++)
	{
		/* Declear the feature variables of the point */
		double evqu0 = 0, evqu1 = 0, grav0 = 0, grav1 = 0, curvature = 0;

		/* Extract the search point from the point cloud */
		pcl::PointXYZ searchPoint = cloud->at(i);

		/* Find the neighbors of the point */
		if (kdtree->radiusSearch(searchPoint, radius * coef, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0)
		{
			//qDebug("The number of neighbors of (%f, %f, %f) is %d\n", searchPoint.x, searchPoint.y, searchPoint.z, pointIdxRadiusSearch.size());
			PointCloud<PointXYZ> neighborhood;
			for (int j = 0; j < pointIdxRadiusSearch.size(); j++)
				neighborhood.push_back(cloud->at(pointIdxRadiusSearch.at(j)));

			/* Compute the covariance matrix on the point over its neighborhood */
			Eigen::Matrix3d cov_matrix;
			computeCovarianceMatrix(neighborhood, cov_matrix);

			/* Compute the eigen values and eigen vectors of the covariance matrix */
			Eigen::EigenSolver<Eigen::Matrix3d> solver(cov_matrix, true);
			VectorXcd evalues = solver.eigenvalues().transpose();
			//qDebug() << "Eigenvalues of point" << i <<":" << eva[0].real() << "," << eva[1].real() << "," << eva[2].real();
			Eigen::MatrixXcd eigenvectors = solver.eigenvectors();
			Vector3cd evector0 = eigenvectors.col(0);
			Vector3cd evector1 = eigenvectors.col(1);
			Vector3cd evector2 = eigenvectors.col(2);

			/* Specify the order of eigen values */
			int firstno, secno, thirdno;
			double max = -255.0;
			for (int i = 0; i < 3; i++)
			{
				if (evalues[i].real() > max)
				{
					firstno = i;
					max = evalues[i].real();
				}
			}
			max = -255.0;
			for (int i = 0; i < 3; i++)
			{
				if (i != firstno)
				{
					if (evalues[i].real() > max)
					{
						secno = i;
						max = evalues[i].real();
					}
				}
			}
			thirdno = 3 - firstno - secno;

			/* Calculate part of the features */
			evqu0 = evalues[secno].real() / evalues[firstno].real();
			evqu1 = evalues[thirdno].real() / evalues[firstno].real();
			Vector3cd g(0, -1.0, 0);
			MatrixXcd gm0 = eigenvectors.col(firstno).transpose() * g;
			grav0 = gm0.data()[0].real();
			MatrixXcd gm2 = eigenvectors.col(thirdno).transpose() * g;
			grav1 = gm2.data()[0].real();
		}
		curvature = normals->at(i).curvature;
		//sdf = Utils::sdf(cloud, normals, i);

		/* Add the point feature vector to list */
		QVector<double> feat;
		feat.push_back(evqu0);
		feat.push_back(evqu1);
		feat.push_back(grav0);
		feat.push_back(grav1);
		feat.push_back(curvature);
		points_feats.push_back(feat);
	}
}
