#include "PAPoint.h"


PAPoint::PAPoint() : part_label(NULL_LABEL)
{
}

PAPoint::PAPoint(double feats[POINT_FEATURES_DIMEN]) : part_label(NULL_LABEL)
{
	memcpy(features, feats, POINT_FEATURES_DIMEN * sizeof(double));
}

PAPoint::PAPoint(QVector<double> feats) : part_label(NULL_LABEL)
{
	memcpy(features, feats.data(), POINT_FEATURES_DIMEN * sizeof(double));
}

PAPoint::PAPoint(const PAPoint &point)
{
	for (int i = 0; i < POINT_FEATURES_DIMEN; i++)
		features[i] = point[i];
	part_label = point.getLabel();
	m_position[0] = point.x();
	m_position[1] = point.y();
	m_position[2] = point.z();
	m_normal = point.getNormal();
}

PAPoint::~PAPoint()
{

}

double PAPoint::operator[](int f) const
{
	return features[f];
}

double *PAPoint::getFeatures()
{
	return features;
}

void PAPoint::setFeatures(int part, double feats[POINT_FEATURE_PARTS])
{
	for (int i = 0; i < POINT_FEATURE_PARTS; i++)
		features[part * POINT_FEATURE_PARTS + i] = feats[i];
}

void PAPoint::setFeatures(int part, QVector<double> feats)
{
	//qDebug() << "Set features:" << feats;
	for (int i = 0; i < POINT_FEATURE_PARTS; i++)
		features[part * POINT_FEATURE_PARTS + i] = feats[i];
}

void PAPoint::setClassConfidences(QMap<int, float> confidences)
{
	m_class_confidences.clear();
	for (QMap<int, float>::iterator it = confidences.begin(); it != confidences.end(); ++it)
	{
		int label = it.key();
		m_class_confidences.insert(label, *it);
	}
}

void PAPoint::setSdf(double sdfvalue)
{
	features[POINT_FEATURES_DIMEN - 1] = sdfvalue;
}

void PAPoint::setHeight(double heightvalue)
{
	features[POINT_FEATURES_DIMEN - 2] = heightvalue;
}

using namespace std;
string PAPoint::toString()
{
	string featStr;
	for (int i = 0; i < POINT_FEATURES_DIMEN - 1; i++)
		featStr.append(to_string(features[i]) + ",");
	featStr.append(to_string(features[POINT_FEATURES_DIMEN - 1]));

	if (part_label >= 0 && part_label < 10)
		featStr.append("," + to_string(part_label));

	return featStr;
}

void PAPoint::setLabel(int l)
{
	part_label = l;
}

int PAPoint::getLabel() const
{
	return part_label;
}

void PAPoint::setPosition(float x, float y, float z)
{
	m_position[0] = x;
	m_position[1] = y;
	m_position[2] = z;
}

void PAPoint::setNormal(float nx, float ny, float nz)
{
	m_normal[0] = nx;
	m_normal[1] = ny;
	m_normal[2] = nz;
}

float PAPoint::x() const
{
	return m_position.x();
}

float PAPoint::y() const
{
	return m_position.y();
}

float PAPoint::z() const
{
	return m_position.z();
}

Eigen::Vector3f PAPoint::getPosition() const
{
	return m_position;
}

Eigen::Vector3f PAPoint::getNormal() const
{
	return m_normal;
}

float PAPoint::getClassConfidence(int label)
{
	return m_class_confidences.value(label);
}

