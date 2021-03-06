#ifndef FEATUREESTIMATOR_H
#define FEATUREESTIMATOR_H

#include <QObject>
#include <QVector>
#include <fstream>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/common/impl/centroid.hpp>
#include <pcl/registration/distances.h>
#include <pcl/filters/voxel_grid.h>
#include <Eigen/src/Core/MatrixBase.h>
#include <Eigen\src\Eigenvalues\EigenSolver.h>
#include "pcmodel.h"
#include <CGAL/Min_sphere_annulus_d_traits_d.h>
#include <CGAL/Min_sphere_d.h>
#include <CGAL/Cartesian_d.h>
#include "featurethread.h"
#include "PAPointCloud.h"

#define NUM_OF_THREADS 6

typedef CGAL::Cartesian_d<double>              K;
typedef CGAL::Min_sphere_annulus_d_traits_d<K> Traits;
typedef CGAL::Min_sphere_d<Traits>             Min_sphere;
typedef K::Point_d                             Point;

class FeatureEstimator : public QObject
{
	Q_OBJECT

public:
	enum PHASE{
		TRAINING,
		TESTING
	};
	FeatureEstimator(QObject *parent = 0);
	FeatureEstimator(PCModel *pcModel, PHASE phase, QObject *parent = 0);
	~FeatureEstimator();

	void estimateFeatures();
	void reset(PCModel * pcModel);
	void setPhase(PHASE phase);

	public slots:
	void receiveFeatures(int id, QVector<QVector<double>> feats);
	void onDebugTextAdded(QString text);

signals:
	void estimateCompleted(PAPointCloud *cloud);
	void addDebugText(QString text);

private:
	pcl::PointCloud<pcl::PointXYZ>::Ptr m_cloud;
	pcl::PointCloud<pcl::Normal>::Ptr m_normals;
	double m_radius;
	int finish_count;
	QVector<FeatureThread *> m_subthreads;
	PAPointCloud *m_pointcloud;
	QString m_pointcloudFile;
	QVector<int> m_points_labels;
	PHASE m_phase;
	QVector<double> m_sdf;

	//QVector<int> getLabels(QString segfile);
};

#endif // FEATUREESTIMATOR_H
