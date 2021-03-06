#pragma once
#include <vector>
#include <fstream>
#include <iostream>
#include "PAPoint.h"

class PAPointCloud
{
public:
	PAPointCloud();
	PAPointCloud(int size);
	PAPointCloud(const PAPointCloud &cloud);
	~PAPointCloud();

	void addPoint(PAPoint point);
	void addPoint(PAPoint point, int index);
	PAPoint & at(int index);
	PAPoint & operator[](int index);
	void resize(int newsize);
	int size();
	void writeToFile(const char *file);
	void setRadius(float radius);
	float getRadius() const;
	std::vector<PAPoint> getPoints() const;

private:
	std::vector<PAPoint> pointList;
	float m_radius;
};

