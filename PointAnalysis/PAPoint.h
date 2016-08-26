#pragma once
#include <QVector>
#include <QMap>
#include <qdebug.h>
#include <cstdlib>
#include <string>
#include <Eigen\Core>
#include "constants.h"

class PAPoint
{
public:
	PAPoint();
	PAPoint(double feats[POINT_FEATURES_DIMEN]);
	PAPoint(QVector<double> feats);
	PAPoint(const PAPoint &point);
	~PAPoint();
	 
	enum FEATURE{
		evqu00 = 0, evqu01, grav00, grav01, curvature0,
		evqu10, evqu11, grav10, grav11, curvature1,
		evqu20, evqu21, grav20, grav21, curvature2,
		evqu30, evqu31, grav30, grav31, curvature3,
		evqu40, evqu41, grav40, grav41, curvature4, 
		height, sdf
	};

	double operator[](int f) const;
	double * getFeatures();
	void setFeatures(int part, double feats[POINT_FEATURE_PARTS]);
	void setFeatures(int part, QVector<double> feats);
	void setClassConfidences(QMap<int, float> confidences);
	std::string toString();
	void setSdf(double sdfvalue); 
	void setHeight(double heightvalue);
	void setLabel(int l);
	int getLabel() const;
	void setPosition(float x, float y, float z);
	void setNormal(float nx, float ny, float nz);
	float x() const;
	float y() const;
	float z() const;
	Eigen::Vector3f getPosition() const;
	Eigen::Vector3f getNormal() const;
	float getClassConfidence(int label);

private:
	double features[POINT_FEATURES_DIMEN];
	int part_label;
	Eigen::Vector3f m_position;
	Eigen::Vector3f m_normal;
	QMap<int, float> m_class_confidences;
};

