#ifndef PAPARTRELATION_H
#define PAPARTRELATION_H

#include "papart.h"
#include <Eigen\Core>
#include <cstdlib>
#include <qdebug.h>
#include <QPair>
#include <vector>
#include "utils.h"

class PAPartRelation
{
public:
	PAPartRelation();
	PAPartRelation(const PAPartRelation & relation);
	PAPartRelation(const PAPart &part1, const PAPart &part2);
	PAPartRelation(PAPart * part1, PAPart *part2);
	~PAPartRelation();

	QPair<int, int> getLabelPair();
	int getFirstLabel() const { return m_part_label_1; }
	int getSecondLabel() const { return m_part_label_2; }
	std::vector<double> getFeatureVector() const;
	std::vector<float> getFeatureVector_Float() const;
	float * getFeatureArray() const;
	int getDimension() const { return 32; }

private:
	float m_feature[32];
	int m_part_label_1;
	int m_part_label_2;
};

#endif