#ifndef ENERGYFUNCTIONS_H
#define ENERGYFUNCTIONS_H

#include <qvector.h>
#include <qmap.h>
#include <QSet>
#include <QList>
#include <iostream>
#include <string>
#include <QSharedPointer>
#include <cmath>
#include <vector>
#include <fstream>
#include "papart.h"
#include "PAPointCloud.h"
#include "papartrelation.h"
#include "PAPoint.h"
#include "definitions.h"
#include "utils.h"
#include "constants.h"

class EnergyFunctions
{
public:
	EnergyFunctions(std::string modelClassName);
	~EnergyFunctions();

	void setPointCloud(QSharedPointer<PAPointCloud> pointcloud);
	void setDistributions(const QVector<QMap<int, float>> &distributions);
	void setOBBs(QMap<int, OBB*> obbs);
	void setPointAssignments(QVector<int> point_assignbments);
	int getNullLabelName() { return m_null_label; }
	QVector<int> getPointAssignments() const;

	/* 
	 Epnt
	 The function computing the point classification energy.
	 Parameters: 
	 part - the candidate part.
	 distributions - points classifier probability distribution.
	 label - the assumed label.
	 use_symmetry - whether to use symmetry information, only be true when first iteration of the algorithm.
	 Return value: the Epnt energy value of the candidate with certain assumed label.
	 */
	double Epnt(PAPart *part, int label);
	double Epnt(PAPart *part, int label, bool use_symmetry);
	double Epnt(const PAPart *part, int label, bool use_symmetry);

	/* Epnt_single 
	   The function computing the point classification energy for a single point 
	   Parameters:
	   point_index - the index of the point in the point cloud.
	   label - the assumed label.
	   return value: the Epnt energyu for this point */
	double Epnt_single(int point_index, int label);
	/* 
	 Epair
	 The function computing the part relations energy.
	 Parameters:
	 part1 - the first candida te part.
	 part2 - the second candidate part.
	 label1 - the assumed label of the first part.
	 label2 - the assumed label of the second pard.
	 Return value: the Epair of the two parts with certain assumed labels.
	 */
	double Epair(PAPartRelation relation, int cluster_no_1, int cluster_no_2, int label1, int label2);

	/* Ep_q
	   One of the function in energy parts-points distances, which computes distanc from points to sample points 
	   Parameters:
	   point_index - the index of the point in the point cloud.
	   assignment - the assumed assignment of the piont to a certain part.
	   return value: the Ep_q of the point */
	double Ep_q(int point_index, int assignment);

	/* Esmooth
	   The function computing the segmentation smoothness.
	   Parameters:
	   point_index - the index of the point in the pointc cloud.
	   return value:
	   a list of triplets, each triplets is a energy item of the input point and one of its neighbor point;
	   the triplet is of form <index_0, index_1, energy>
	   */
	QVector<Eigen::Triplet<double>> Esmooth(int point_index);

private:
	std::string m_modelClassName;
	QMap<QPair<int, int>, Eigen::MatrixXf> m_covariance_matrices;
	QMap<QPair<int, int>, Eigen::VectorXf> m_mean_vectors;
	QVector<QMap<int, float>> m_distributions;
	QSharedPointer<PAPointCloud> m_pointcloud;
	pcl::KdTreeFLANN<pcl::PointXYZ> m_cloud_kdtree;  /* The kd-tree for searching nearest neighbor in the whole point cloud */
	int m_null_label;
	QList<QVector<int>> m_symmetry_groups;  /* Symmetry groups, all parts contained in a group are symmetry to each other.
											   If a symmetry group vector only has one element, that means the part is self-symmetric */
	QSet<int> m_symmetry_set;    /* The sets of parts which are symmetry to other parts */
	QMap<int, OBB *> m_obbs; /* The container storing the OBBs of all the winner candidate parts */
	QMap<int, pcl::KdTreeFLANN<pcl::PointXYZ>> m_kdtrees;   /* The kd-trees of each part for searching the nearest neighbor in samples of a particular part */
	QVector<int> m_point_assignments;    /* The assignments of point to particular part */

	/* Generate a search kd-tree for the point cloud */
	void generateKdTree();

	static float w1, w2, w3, w4, w5;
};

#endif
