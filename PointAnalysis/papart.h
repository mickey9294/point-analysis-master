#ifndef PAPART_H
#define PAPART_H

#include <Eigen\Core>
#include <qvector.h>
#include <qvector3d.h>
#include <qvector4d.h>
#include <qmatrix4x4.h>
#include <QList>
#include <vector>
#include <string>
#include <fstream>
#include <cmath>
#include "obb.h"

class PAPart
{
public:
	PAPart();
	PAPart(std::string path);
	PAPart(OBB *obb);
	PAPart(const PAPart &part);
	~PAPart();

	Eigen::Matrix3f getRotMat() const;
	Eigen::Vector3f getTransVec() const;
	Eigen::Vector3f getScale() const;
	Eigen::Vector4f getHeight() const;
	OBB * getOBB() const;
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
	void saveToFile(std::string name);
	int getClusterNo() const { return m_cluster_no; }
	OBB * generateOBB();
	std::vector<int>::iterator vertices_begin();
	std::vector<int>::iterator vertices_end();

private:
	Eigen::Matrix3f m_rotate;    /* Rotation matrix of 3x3 */
	Eigen::Vector3f m_translate;
	Eigen::Vector3f m_scale;
	Eigen::Vector4f m_height;
	Eigen::Matrix3f m_axes;    /* Each column of the matrix represents an axis of the part local coordinate system */
	std::vector<int> m_vertices_indices;
	int m_label;
	int m_cluster_no;
	OBB *m_obb;
};

Q_DECLARE_METATYPE(PAPart)

#endif