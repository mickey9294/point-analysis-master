#include "papartrelation.h"

using namespace Eigen;

PAPartRelation::PAPartRelation()
{

}

PAPartRelation::PAPartRelation(PAPart part1, PAPart part2)
{
	m_part_label_1 = part1.getLabel();
	m_part_label_2 = part2.getLabel();

	Matrix<float, 3, 4> left12;
	left12.block<3, 3>(0, 0) = part2.getRotMat().transpose();
	left12.block<3, 1>(0, 3) = -part2.getRotMat().transpose() * part2.getTransVec();
	Matrix4f med12;
	med12.block<3, 3>(0, 0) = part1.getRotMat();
	med12.block<3, 1>(0, 3) = part1.getTransVec();
	med12.block<1, 3>(3, 0) = Vector3f::Zero().transpose();
	med12(3, 3) = 1;

	Matrix4f right12;
	Vector3f scale1 = part1.getScale();
	right12 << scale1[0], 0.0f, 0.0f, 0.0f,
		0.0f, scale1[1], 0.0f, 0.0f,
		0.0f, 0.0f, scale1[2], 0.0f,
		0.0f, 0.0f, 0.0f, 1.0f;

	Matrix<float, 3, 4> T12 = left12 * med12 * right12;

	Matrix<float, 3, 4> left21;
	left21.block<3, 3>(0, 0) = part1.getRotMat().transpose();
	left21.block<3, 1>(0, 3) = -part1.getRotMat().transpose() * part1.getTransVec();
	Matrix4f med21;
	med21.block<3, 3>(0, 0) = part2.getRotMat();
	med21.block<3, 1>(0, 3) = part2.getTransVec();
	med21.block<1, 3>(3, 0) = Vector3f::Zero().transpose();
	med21(3, 3) = 1;
	Matrix4f right21;
	Vector3f scale2 = part2.getScale();
	right21 << scale2[0], 0.0f, 0.0f, 0.0f,
		0.0f, scale2[1], 0.0f, 0.0f,
		0.0f, 0.0f, scale2[2], 0.0f,
		0.0f, 0.0f, 0.0f, 1.0f;

	Matrix<float, 3, 4> T21 = left21 * med21 * right21;

	std::memcpy(m_feature, T12.data(), 12 * sizeof(float));
	std::memcpy(m_feature + 12, part1.getHeight().data(), 4 * sizeof(float));
	std::memcpy(m_feature + 16, T21.data(), 12 * sizeof(float));
	std::memcpy(m_feature + 28, part2.getHeight().data(), 4 * sizeof(float));

	//VectorXf feat(m_feature);
	qDebug() << "done.";
}


PAPartRelation::~PAPartRelation()
{
}

QPair<int, int> PAPartRelation::getLabelPair()
{
	QPair<int, int> pair(m_part_label_1, m_part_label_2);
	return pair;
}

std::vector<double> PAPartRelation::getFeatureVector()
{
	std::vector<double> feat(32);
	for (int i = 0; i < 32; i++)
		feat[i] = (double)m_feature[i];
	//feat[32] = 0;
	return feat;
}

float * PAPartRelation::getFeatureArray()
{
	return m_feature;
}