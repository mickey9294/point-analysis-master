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
#include "utils.h"
#include "ICP.h"
#include "SamplePoint.h"
#include "constants.h"

class PAPart
{
public:
	PAPart();
	PAPart(std::string path);
	PAPart(OBB *obb);
	PAPart(const PAPart &part);
	~PAPart();

	int num_of_samples;

	Eigen::Matrix3f getRotMat() const;
	Eigen::Vector3f getTransVec() const;
	Eigen::Vector3f getScale() const;
	Eigen::Vector4f getHeight() const;
	void setOBB(OBB *obb);
	OBB * getOBB() const;
	QMatrix4x4 getQRotMat();
	QVector3D getQTransVec();
	QVector3D getQScale();
	QVector4D getQHeight();
	int getLabel() const{ return m_label; }
	Eigen::Matrix3f getAxes() const;
	bool isInside(Eigen::Vector3f point);
	std::vector<int> getVerticesIndices() const;
	std::vector<Eigen::Vector3f> getVertices() const;
	std::vector<Eigen::Vector3f> getVerticesNormals() const;
	std::vector<SamplePoint> getSamples() const;
	void getSamplePoints(std::vector<Eigen::Vector3f> &sample_points) const;
	SamplePoint getSample(int index);
	int numOfSamples() const;
	void setVerticesIndices(QList<int> indices);
	void setClusterNo(int cluster_no);
	void saveToFile(std::string name);
	int getClusterNo() const { return m_cluster_no; }
	OBB * generateOBB();
	std::vector<int>::iterator vertices_begin();
	std::vector<int>::iterator vertices_end();
	void setLabel(int label);
	void setVertices(QVector<Eigen::Vector3f> vertices);
	void setVertices(std::vector<Eigen::Vector3f> vertices);
	void setVerticesNormals(QVector<Eigen::Vector3f> vertices_normals);
	void setVerticesNormals(std::vector<Eigen::Vector3f> vertices_normals);
	void setTranslation(Eigen::Vector3f centroid);
	void samplePoints();
	
	int get_sample_to_cuboid_surface_correspondences(const int point_index);
	const std::vector<int> & get_sample_to_cuboid_surface_correspondences() const;
	int get_cuboid_surface_to_sample_correspondences(const int point_index);
	const std::vector<int> & get_cuboid_surface_to_sample_correspondeces() const;
	void create_random_points_on_obb_surface();
	void create_grid_points_on_obb_surface();
	void update_sample_correspondences();

	void clearVertices();
	void clearSamples();

	void addVertex(int point_index, Eigen::Vector3f vertex, Eigen::Vector3f vertex_normal);

	std::vector<SamplePoint>::iterator samples_begin();
	std::vector<SamplePoint>::iterator samples_end();

	void updateFromOBB();
	void update_axes_center_size_corner_points();
	void update_center_size_corner_points();

	void ICP_adjust_OBB();

	int num_cuboid_surface_points();

private:
	Eigen::Matrix3f m_rotate;    /* Rotation matrix of 3x3 */
	Eigen::Vector3f m_translate;
	Eigen::Vector3f m_scale;
	Eigen::Vector4f m_height;
	Eigen::Matrix3f m_axes;    /* Each column of the matrix represents an axis of the part local coordinate system */
	std::vector<int> m_vertices_indices;
	std::vector<Eigen::Vector3f> m_vertices;
	std::vector<Eigen::Vector3f> m_vertices_normals;
	std::vector<SamplePoint> m_samples;
	int m_label;
	int m_cluster_no;
	OBB *m_obb;
	std::vector<int> m_sample_to_cuboid_surface_correspondence;
	std::vector<int> m_cuboid_surface_to_sample_correspondence;
};

Q_DECLARE_METATYPE(PAPart)

#endif