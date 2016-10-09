#include "PAPointCloud.h"


PAPointCloud::PAPointCloud()
{
	m_radius = 0;
}

PAPointCloud::PAPointCloud(int size)
{
	m_radius = 0;
	pointList.resize(size);
	//for (int i = 0; i < size; i++)
	//{
	//	PAPoint nullPoint;
	//	pointList.push_back(nullPoint);
	//}
}

PAPointCloud::PAPointCloud(const PAPointCloud &cloud)
{
	pointList = std::vector<PAPoint>(cloud.getPoints());
	m_radius = cloud.getRadius();
}

PAPointCloud::~PAPointCloud()
{
	//pointList.clear();
}

void PAPointCloud::addPoint(PAPoint point)
{
	pointList.push_back(point);
}

PAPoint & PAPointCloud::at(int index)
{
	return pointList[index];
}

PAPoint & PAPointCloud::operator[](int index)
{
	return pointList[index];
}

int PAPointCloud::size() const
{
	return pointList.size();
}

void PAPointCloud::resize(int newsize)
{
	if (newsize > size())
	{
		int num_of_added = newsize - size();
		for (int i = 0; i < num_of_added; i++)
		{
			if (i == 8778)
				std::cout << "";
			PAPoint nullPoint;
			pointList.push_back(nullPoint);
		}
	}
	else
	{
		int num_of_removed = size() - newsize;
		for (int i = 0; i < num_of_removed; i++)
			pointList.pop_back();
	}
}

using namespace std;
void PAPointCloud::writeToFile(const char *filename)
{
	ofstream out(filename);
	if (out.is_open())
	{
		//out << size() << endl;
		for (int i = 0; i < size(); i++){
			int label = pointList.at(i).getLabel();
			if (label == 9294 || label >= 0 && label <= 10)
				out << pointList.at(i).toString() << endl;
		}

		out.close();
	}
}

void PAPointCloud::setRadius(float radius)
{
	m_radius = radius;
}

float PAPointCloud::getRadius() const
{
	return m_radius;
}

std::vector<PAPoint> PAPointCloud::getPoints() const
{
	return pointList;
}

std::vector<PAPoint>::iterator PAPointCloud::begin()
{
	return pointList.begin();
}

std::vector<PAPoint>::iterator PAPointCloud::end()
{
	return pointList.end();
}

void PAPointCloud::outputConfidences(const char *file)
{
	ofstream out(file);

	if (out.is_open())
	{
		for (std::vector<PAPoint>::iterator point_it = pointList.begin(); point_it != pointList.end(); ++point_it)
		{
			const QMap<int, float> distribution = point_it->getClassConfidences();
			for (QMap<int, float>::const_iterator dis_it = distribution.begin(); dis_it != distribution.end(); ++dis_it)
				out << *dis_it << ",";

			out << endl;
		}

		out.close();
	}
}