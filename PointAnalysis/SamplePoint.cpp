#include "SamplePoint.h"

using namespace Eigen;

SamplePoint::~SamplePoint()
{
}

SamplePoint::SamplePoint()
{
	m_vertex.setZero();
	m_normal.setZero();
	m_visibility = 1.0;
	m_face_index = -1;
}

SamplePoint::SamplePoint(const SamplePoint &sample)
{
	m_vertex = sample.getVertex();
	m_normal = sample.getNormal();
	m_visibility = sample.visibility();
	m_corner_weights = sample.getCornerWeights();
	m_face_index = sample.getFaceIndex();
}

SamplePoint::SamplePoint(Vector3f vertex, Vector3f normal)
{
	m_vertex = vertex;
	m_normal = normal;
	m_visibility = 1.0;
	m_face_index = -1;
}

SamplePoint::SamplePoint(Eigen::Vector3f vertex, Eigen::Vector3f normal, int face_index, std::array<Real, 8> corner_weights)
{
	m_vertex = vertex;
	m_normal = normal;
	m_face_index = face_index;
	m_corner_weights = corner_weights;
}

SamplePoint::SamplePoint(Vector3f vertex)
{
	m_vertex = vertex;
	m_normal.setZero();
	m_visibility = 1.0;
	m_face_index = -1;
}

SamplePoint::SamplePoint(float x, float y, float z)
{
	m_vertex[0] = x;
	m_vertex[1] = y;
	m_vertex[2] = z;
	m_normal.setZero();
	m_visibility = 1.0;
	m_face_index = -1;
}

SamplePoint::SamplePoint(float x, float y, float z, float nx, float ny, float nz)
{
	m_vertex[0] = x;
	m_vertex[1] = y;
	m_vertex[2] = z;
	m_normal[0] = nx;
	m_normal[1] = ny;
	m_normal[2] = nz;
	m_face_index = -1;
	m_visibility = 1.0;
}

float SamplePoint::x() const
{
	return m_vertex.x();
}

float SamplePoint::y() const
{
	return m_vertex.y();
}

float SamplePoint::z() const
{
	return m_vertex.z();
}

float SamplePoint::nx() const
{
	return m_normal.x();
}

float SamplePoint::ny() const
{
	return m_normal.y();
}

float SamplePoint::nz() const
{
	return m_normal.z();
}

Vector3f SamplePoint::getVertex() const
{
	return m_vertex;
}

Vector3f SamplePoint::getNormal() const
{
	return m_normal;
}

void SamplePoint::setVertex(Vector3f vertex)
{
	m_vertex[0] = vertex[0];
	m_vertex[1] = vertex[1];
	m_vertex[2] = vertex[2];
}

void SamplePoint::setNormal(Vector3f normal)
{
	m_normal[0] = normal[0];
	m_normal[1] = normal[1];
	m_normal[2] = normal[2];
}

double SamplePoint::visibility() const
{
	return m_visibility;
}

void SamplePoint::setVisibility(double visibility)
{
	m_visibility = visibility;
}

float & SamplePoint::operator[](int index)
{
	assert(index > 5);

	switch (index)
	{
	case 0:
		return m_vertex[0];
	case 1:
		return m_vertex[1];
	case 2:
		return m_vertex[2];
	case 3:
		return m_normal[0];
	case 4:
		return m_normal[1];
	case 5:
		return m_normal[2];
	}
}

std::array<Real, 8> SamplePoint::getCornerWeights() const
{
	return m_corner_weights;
}

void SamplePoint::setCornerWeights(std::array<Real, 8> corner_weights)
{
	m_corner_weights = corner_weights;
}

int SamplePoint::getFaceIndex() const
{
	return m_face_index;
}