#ifndef PAPART_H
#define PAPART_H

#include <Eigen\Core>
#include <qvector.h>
#include <qvector3d.h>
#include <QList>
#include <vector>
#include <string>
#include <fstream>
#include "obb.h"
#include <cmath>

class PAPart
{
public:
	PAPart();
	PAPart(OBB *obb);
	PAPart(std::string path);
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
	Eigen::Matrix3f getAxes() const;
	bool isInside(Eigen::Vector3f point);
	std::vector<int> getVerticesIndices() const;
	void setVerticesIndices(QList<int> indices);
	void setClusterNo(int cluster_no);
	int getClusterNo() const { return m_cluster_no; }
	void saveToFile(std::string name);
	OBB * generateOBB();

private:
	Eigen::Matrix3f m_rotate;    /* Rotation matrix of 3x3 */
	Eigen::Vector3f m_translate;
	Eigen::Vector3f m_scale;
	Eigen::Vector4f m_height;
	Eigen::Matrix3f m_axes;    /* Each column of the matrix represents an axis of the part local coordinate system */
	std::vector<int> m_vertices_indices;
	int m_label;
	int m_cluster_no;
};

Q_DECLARE_METATYPE(PAPart)

#endif