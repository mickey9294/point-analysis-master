#include "papart.h"

using namespace Eigen;
PAPart::PAPart(OBB * obb)
{
	m_label = obb->getLabel();
	QVector3D centroid = obb->getCentroid();
	m_translate = Vector3f(centroid.x(), centroid.y(), centroid.z());
	QVector3D scale = obb->getScale();
	m_scale = Vector3f(scale.x(), scale.y(), scale.z());
	Vector3f x_axis(obb->getXAxis().x(), obb->getXAxis().y(), obb->getXAxis().z());
	Vector3f y_axis(obb->getYAxis().x(), obb->getYAxis().y(), obb->getYAxis().z());
	Vector3f z_axis(obb->getZAxis().x(), obb->getZAxis().y(), obb->getZAxis().z());
	
	Matrix3f base_axes = Matrix3f::Identity();
	m_rotate.col(0) = x_axis;
	m_rotate.col(1) = y_axis;
	m_rotate.col(2) = z_axis;

	const Vector3f gravity(0.0, -1.0, 0.0);
	Matrix<float, 3, 4> Rt;
	Rt.block<3, 3>(0, 0) = m_rotate;
	Rt.block<3, 1>(0, 3) = m_translate;
	Matrix4f Sb;
	Sb << m_scale[0] , 0.0f , 0.0f , 0.0f, 
		0.0f , m_scale[1] , 0.0f , 0.0f,
		0.0f , 0.0f , m_scale[2] , 0.0f,
		0.0f , 0.0f , 0.0f , 1.0f;
	
	m_height = (gravity.transpose() * Rt * Sb).transpose();
}

PAPart::PAPart(const PAPart &part)
{
	m_rotate = Matrix3f(part.getRotMat());
	m_translate = Vector3f(part.getTransVec());
	m_height = Vector4f(part.getHeight());
	m_scale = Vector3f(part.getScale());
	m_label = part.getLabel();
}

PAPart::PAPart()
{
	m_rotate = Matrix3f::Identity();
	m_translate = Vector3f::Zero();
	m_scale = Vector3f::Zero();
}


PAPart::~PAPart()
{
}

Matrix3f PAPart::getRotMat() const
{
	return m_rotate;
}

QMatrix4x4 PAPart::getQRotMat()
{
	QVector4D col0(m_rotate.col(0).x(), m_rotate.col(0).y(), m_rotate.col(0).z(), 0);
	QVector4D col1(m_rotate.col(1).x(), m_rotate.col(1).y(), m_rotate.col(1).z(), 0);
	QVector4D col2(m_rotate.col(2).x(), m_rotate.col(2).y(), m_rotate.col(2).z(), 0);
	QVector4D col3(0, 0, 0, 1.0);
	QMatrix4x4 qRotMat;
	qRotMat.setColumn(0, col0);
	qRotMat.setColumn(1, col1);
	qRotMat.setColumn(2, col2);
	qRotMat.setColumn(3, col3);
	qDebug() << qRotMat;

	return qRotMat;
}

Vector3f PAPart::getTransVec() const
{
	return m_translate;
}

Vector3f PAPart::getScale() const
{
	return m_scale;
}

QVector3D PAPart::getQTransVec()
{
	return QVector3D(m_translate.x(), m_translate.y(), m_translate.z());
}

QVector3D PAPart::getQScale()
{
	return QVector3D(m_scale.x(), m_scale.y(), m_scale.z());
}

Vector4f PAPart::getHeight() const
{
	return m_height;
}

QVector4D PAPart::getQHeight()
{
	return QVector4D(m_height[0], m_height[1], m_height[2], m_height[3]);
}