#ifndef ENERGYFUNCTIONS_H
#define ENERGYFUNCTIONS_H

#include <qvector.h>
#include <qmap.h>
#include <iostream>
#include <string>
#include <cmath>
#include <fstream>
#include "papart.h"
#include "PAPointCloud.h"
#include "papartrelation.h"
#include "PAPoint.h"
#include "gencandidatesthread.h"
#include "utils.h"

#define INF 1E9 /* The infinite value */

class EnergyFunctions
{
public:
	EnergyFunctions(std::string modelClassName);
	~EnergyFunctions();

	void setPointCloud(PAPointCloud *pointcloud);
	void setDistributions(QVector<QMap<int, float>> distributions);
	int getNullLabelName() { return m_null_label; }
	/* 
	 Epnt
	 The function computing the point classification energy.
	 Parameters: 
	 part - the candidate part.
	 distributions - points classifier probability distribution.
	 label - the assumed label.
	 Return value: the Epnt energy value of the candidate with certain assumed label.
	 */
	double Epnt(PAPart part, int label);
	/* 
	 Epair
	 The function computing the part relations energy.
	 Parameters:
	 part1 - the first candidate part.
	 part2 - the second candidate part.
	 label1 - the assumed label of the first part.
	 label2 - the assumed label of the second pard.
	 Return value: the Epair of the two parts with certain assumed labels.
	 */
	double Epair(PAPartRelation relation, int cluster_no_1, int cluster_no_2, int label1, int label2);

private:
	std::string m_modelClassName;
	QMap<QPair<int, int>, Eigen::MatrixXf> m_covariance_matrices;
	QMap<QPair<int, int>, Eigen::VectorXf> m_mean_vectors;
	QVector<QMap<int, float>> m_distributions;
	PAPointCloud *m_pointcloud;
	int m_null_label;

	static float w1, w2, w3, w4, w5;
};

#endif
