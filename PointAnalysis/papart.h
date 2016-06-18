#ifndef PAPART_H
#define PAPART_H

#include <Eigen\Core>
#include "obb.h"
#include <cmath>

class PAPart
{
public:
	PAPart();
	PAPart(OBB *obb);
	PAPart(const PAPart &part);
	~PAPart();

	Eigen::Matrix3f getRotMat() const;
	Eigen::Vector3f getTransVec() const;
	Eigen::Vector3f getScale() const;
	Eigen::Vector4f getHeight() const;
	QMatrix4x4 getQRotMat();
	QVector3D getQTransVec();
	QVector3D getQScale();
	QVector4D getQHeight();
	int getLabel() const{ return m_label; }

private:
	Eigen::Matrix3f m_rotate;    /* Rotation matrix of 3x3 */
	Eigen::Vector3f m_translate;
	Eigen::Vector3f m_scale;
	Eigen::Vector4f m_height;
	int m_label;
};

Q_DECLARE_METATYPE(PAPart)

#endif