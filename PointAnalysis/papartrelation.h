#ifndef PAPARTRELATION_H
#define PAPARTRELATION_H

#include "papart.h"
#include <Eigen\Core>
#include <cstdlib>
#include <qdebug.h>
#include <QPair>
#include <vector>

class PAPartRelation
{
public:
	PAPartRelation();
	PAPartRelation(PAPart part1, PAPart part2);
	~PAPartRelation();

	QPair<int, int> getLabelPair();
	int getFirstLabel() { return m_part_label_1; }
	int getSecondLabel(){ return m_part_label_2; }
	std::vector<double> getFeatureVector();
	float * getFeatureArray();
	int getDimension() { return 32; }

private:
	float m_feature[32];
	int m_part_label_1;
	int m_part_label_2;
};

#endif