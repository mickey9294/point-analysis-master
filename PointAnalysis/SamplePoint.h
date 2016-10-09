#ifndef SAMPLEPOINT_H
#define SAMPLEPOINT_H

#include <Eigen/Core>
#include <array>
#include <string>
#include <qstring.h>
#include <qstringlist.h>

typedef double Real;

class SamplePoint
{
public:
	SamplePoint();
	~SamplePoint();
	SamplePoint(const SamplePoint &sample);
	SamplePoint(Eigen::Vector3f vertex, Eigen::Vector3f normal, Eigen::Vector3f bary = Eigen::Vector3f::Zero());
	SamplePoint(Eigen::Vector3f vertex, Eigen::Vector3f normal, int face_index, std::array<Real, 8> corner_weights);
	SamplePoint(Eigen::Vector3f vertex);
	SamplePoint(float x, float y, float z, float nx, float ny, float nz);
	SamplePoint(float x, float y, float z);
	SamplePoint(std::string string);

	float x() const;
	float y() const;
	float z() const;
	float nx() const;
	float ny() const;
	float nz() const;
	Eigen::Vector3f getVertex() const;
	Eigen::Vector3f getNormal() const;
	Eigen::Vector3f getBary() const;
	void setVertex(Eigen::Vector3f vertex);
	void setNormal(Eigen::Vector3f normal);
	void setBary(float bx, float by, float bz);
	double visibility() const;
	void setVisibility(double visibility);
	std::array<Real, 8> getCornerWeights() const;
	void setCornerWeights(std::array<Real, 8> corner_weights);
	int getFaceIndex() const;
	void setFaceIndex(int face_index);

	float & operator[](int index);
	std::string toString();

private:
	Eigen::Vector3f m_vertex;
	Eigen::Vector3f m_normal;
	Eigen::Vector3f m_bary;
	double m_visibility;
	int m_face_index;
	std::array<Real, 8> m_corner_weights;
	
};

#endif