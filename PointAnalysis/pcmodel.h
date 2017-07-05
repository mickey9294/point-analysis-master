#ifndef PCMODEL_H
#define PCMODEL_H

#include <QObject>
#include <QtOpenGL>
#include <QVector>
#include <cstdlib>
#include <qset.h>
#include <assert.h>
#include <qalgorithms.h>
#include <cmath>
#include <fstream>
#include <iostream>
#include <vector>
#include <string>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include "model.h"
#include "utils.h"
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/jet_estimate_normals.h>
#include <CGAL/mst_orient_normals.h>
#include <CGAL/Simple_cartesian.h>
#include <CGAL/wlop_simplify_and_regularize_point_set.h>
#include "CuboidSymmetryGroup.h"
//#include <GL/glut.h>

// kernel
//typedef CGAL::Exact_predicates_inexact_constructions_kernel Kernel;
//// Simple geometric types
//typedef Kernel::FT FT;
//typedef Kernel::Point_3 Point3;
//typedef Kernel::Vector_3 Vector;
//// Point with normal vector stored in a std::pair.
//typedef std::pair<Point3, Vector> PointVectorPair;
//typedef std::vector<PointVectorPair> PointList;
// types
typedef CGAL::Simple_cartesian<double> SC_Kernel;
typedef SC_Kernel::Point_3 SC_Point;
typedef SC_Kernel::Vector_3 SC_Vector;
// Point with normal vector stored in a std::pair.
typedef std::pair<SC_Point, SC_Vector> SC_PointVectorPair;

class PCModel : public QObject, public Model
{
	Q_OBJECT

public:
	PCModel();
	//PCModel(pcl::PointCloud<pcl::PointXYZ>::ConstPtr pcl_cloud);
	PCModel(const PCModel &pc);
	PCModel(const char *file_path, int normals_estimation_method);
	PCModel(std::string file_path, int normals_estimation_method);
	~PCModel();

	int vertexCount() const { return m_vertices_list.size(); }
	Eigen::Vector3f getCentroid() const;
	void output(const char *filename);
	double getRadius() const{ return m_radius; }
	void setInputFilename(const char *name);
	void setInputFilename(std::string name);
	std::string getInputFilepath() const;
	QVector<int> getVerticesLabels() const;
	Eigen::Vector3f getVertexNormal(int index);
	QVector<double> getSdf() const;
	void setSdf(QVector<double> sdf);
	QVector<int> getLabelNames() const;
	int numOfClasses();
	void rotate(float angle, float x, float y, float z);
	void transform(Eigen::Matrix4f trans_mat);

	void draw(float scale);
	void drawSymmetry(float scale);
	void drawBBox(float scale);
	void drawLine(const Eigen::Vector3f & point_1, const Eigen::Vector3f & point_2, float scale);

	QVector<Eigen::Vector3f>::iterator vertices_begin(){ return m_vertices_list.begin(); }
	QVector<Eigen::Vector3f>::iterator vertices_end() { return m_vertices_list.end(); }
	QVector<Eigen::Vector3f>::iterator normals_begin() { return m_normals_list.begin(); }
	QVector<Eigen::Vector3f>::iterator normals_end() { return m_normals_list.end(); }

	Eigen::Vector3f getBBoxCenter() const;
	Eigen::Vector3f getBBoxSize() const;

	QVector<Eigen::Vector3f> getVertices() const;
	QVector<Eigen::Vector3f> getNormals() const;
	Eigen::Vector3f getVertex(int index);
	Eigen::Vector3f getNormal(int index);

	Eigen::Vector3f & operator[](int index);
	Eigen::Vector3f at(int index);

	void normalize();
	void computeBoundingBox();

	void outputVerticesLabels(const char *file_path);
	void splitOutput(const std::string output_dir);
	void downSample();

	bool isDrawSymmetryPlanes() const { return m_draw_sym_planes; }
	bool isDrawSymmetryAxes() const { return m_draw_sym_axes; }

	public slots:
	void setLabels(QVector<int> labels);
	void receiveSignalTest();
	void setDrawSymmetryPlanes(int state);
	void setDrawSymmetryAxes(int state);

signals:
	void outputProgressReport(int progress);
	void onLabelsChanged();
	void addDebugText(QString text);

private:
	//QVector<GLfloat> m_data;
	QVector<int> m_labels;
	//int m_count;
	Eigen::Vector3f m_centroid;  /* the center of the minimal bounding sphere */
	double m_radius;  /* the radius of the minimal bounding sphere */
	std::string m_input_filepath;
	QVector<double> m_sdf;
	QVector<int> m_label_names;
	QVector<Eigen::Vector3f> m_vertices_list;
	QVector<Eigen::Vector3f> m_normals_list;

	QVector<QVector<float>> m_points_label_confidences;

	QList<CuboidReflectionSymmetryGroup> m_reflection_symmetry;
	QList<CuboidRotationSymmetryGroup> m_rotation_symmetry;

	Eigen::Vector3f m_bbox_center;
	Eigen::Vector3f m_bbox_size;
	Eigen::Vector3f m_bbox_min;
	Eigen::Vector3f m_bbox_max;

	bool m_draw_sym_planes;
	bool m_draw_sym_axes;

	//void add(const QVector3D &v, const QVector3D &n, const QVector3D &c);
	void load_from_file(const char *file_path, int normals_estimation_normals);
	//void saveNormals();
};

#endif // PCMODEL_H
