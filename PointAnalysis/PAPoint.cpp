#include "PAPoint.h"


PAPoint::PAPoint() : part_label(NULL_LABEL)
{
}

PAPoint::PAPoint(double feats[DIMEN]) : part_label(NULL_LABEL)
{
	memcpy(features, feats, DIMEN * sizeof(double));
}

PAPoint::PAPoint(QVector<double> feats) : part_label(NULL_LABEL)
{
	memcpy(features, feats.data(), DIMEN * sizeof(double));
}

PAPoint::PAPoint(const PAPoint &point)
{
	for (int i = 0; i < DIMEN; i++)
		features[i] = point[i];
	part_label = point.getLabel();
	m_position[0] = point.x();
	m_position[1] = point.y();
	m_position[2] = point.z();
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

void PAPoint::setFeatures(int part, double feats[PART])
{
	for (int i = 0; i < PART; i++)
		features[part * PART + i] = feats[i];
}

void PAPoint::setFeatures(int part, QVector<double> feats)
{
	//qDebug() << "Set features:" << feats;
	for (int i = 0; i < PART; i++)
		features[part * PART + i] = feats[i];
}

void PAPoint::setSdf(double sdfvalue)
{
	features[DIMEN - 1] = sdfvalue;
}

void PAPoint::setHeight(double heightvalue)
{
	features[DIMEN - 2] = heightvalue;
}

using namespace std;
string PAPoint::toString()
{
	string featStr;
	for (int i = 0; i < DIMEN - 1; i++)
		featStr.append(to_string(features[i]) + ",");
	featStr.append(to_string(features[DIMEN - 1]));

	if (part_label >= 0 && part_label <= 10)
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
	return Eigen::Vector3f(m_position);
}