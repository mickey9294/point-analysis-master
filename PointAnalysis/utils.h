#ifndef UTILS_H
#define UTILS_H

#include <QObject>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <fstream>
#include <assert.h>
#include <string>
#include <cstdlib>
#include <QDebug>
#include <QVector>
#include <QVector3D>
#include <QPair>
#include <CGAL/Min_sphere_annulus_d_traits_d.h>
#include <CGAL/Min_sphere_d.h>
#include <CGAL/Cartesian_d.h>
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/jet_estimate_normals.h>
#include <CGAL/mst_orient_normals.h>
#include <CGAL/property_map.h>
#include <CGAL/IO/read_off_points.h>
#include <CGAL/boost/graph/graph_traits_Polyhedron_3.h>
#include <CGAL/IO/Polyhedron_iostream.h>
#include <CGAL/mesh_segmentation.h>
#include <CGAL/Simple_cartesian.h>
#include <CGAL/wlop_simplify_and_regularize_point_set.h>
#include <CGAL/edge_aware_upsample_point_set.h>
#include <utility> // defines std::pair
#include <list>
#include <Eigen/Core>
#include <boost/accumulators/accumulators.hpp>
#include <boost/accumulators/statistics.hpp>
#include <boost/date_time/posix_time/posix_time_types.hpp>
#include <cmath>
#include "constants.h"

typedef CGAL::Cartesian_d<double>              K;
typedef CGAL::Min_sphere_annulus_d_traits_d<K> Traits;
typedef CGAL::Min_sphere_d<Traits>             Min_sphere;
typedef K::Point_d                             Point;

// kernel
typedef CGAL::Exact_predicates_inexact_constructions_kernel Kernel;
// Simple geometric types
typedef Kernel::FT FT;
typedef Kernel::Point_3 Point3;
typedef Kernel::Vector_3 Vector;
// Point with normal vector stored in a std::pair.
typedef std::pair<Point3, Vector> PointVectorPair;
typedef std::vector<PointVectorPair> PointList;
// Concurrency
#ifdef CGAL_LINKED_WITH_TBB
typedef CGAL::Parallel_tag Concurrency_tag;
#else
typedef CGAL::Sequential_tag Concurrency_tag;
#endif

typedef CGAL::Polyhedron_3<Kernel> Polyhedron;
typedef Polyhedron::Halfedge_around_facet_const_circulator Halfedge_facet_circulator;

// types used for downsampling and upsampling
typedef CGAL::Simple_cartesian<double> Simple_Cartesian_Kernel;
typedef Simple_Cartesian_Kernel::Point_3 SC_Point;
typedef Simple_Cartesian_Kernel::Vector_3 SC_Vector;
// Point with normal vector stored in a std::pair.
typedef std::pair<SC_Point, SC_Vector> SC_PointVectorPair;

enum PHASE{
	TRAINING,
	TESTING
};

class Utils : public QObject
{
	Q_OBJECT

public:
	Utils(QObject *parent = 0);
	~Utils();

	static double sdf(pcl::PointCloud<pcl::PointXYZ>::Ptr points, pcl::PointCloud<pcl::Normal>::Ptr normals, int searchPointIdx);
	static QVector<double> sdf_mesh(QString off_mesh_filename);
	static bool double_equal(double a, double b);
	static bool float_equal(double a, double b);
	static QString getSegFilename(QString model_file_name);
	static std::string getSegFilename(std::string model_file_name);
	static QString getModelName(QString filepath);
	static std::string getSegFilename(const char *model_file_name);
	static int comb(int n, int i);
	static QVector<QPair<int, int>> getCombinations(QVector<int> nums);
	static std::string vectorToString(Eigen::VectorXf vec);
	static std::string matrixToString(Eigen::MatrixXf mat);
	static void saveMatrixToFile(Eigen::MatrixXf mat, Eigen::VectorXf relation, Eigen::VectorXf mean, Eigen::VectorXf vec);
	//static void savePartsPairToFile(PAPart part1, PAPart part2);
	static void saveRelationToFile(Eigen::Matrix<float, 3, 4> T12, Eigen::Vector4f h1, Eigen::Matrix<float, 3, 4> T21, Eigen::Vector4f h2);
	static void saveFeatureToFile(float feature[32]);
	static void saveFeatureToFile(std::vector<float> featrue);
	static long getCurrentTime();
	static QVector3D eigen_vector3f_to_qvector3d(Eigen::Vector3f vec);
	static std::string getFileFormat(std::string file_path);
	static std::string getFileFormat(const char *file_path);
	static Eigen::Matrix3f rotation_matrix(Eigen::Vector3f v0, Eigen::Vector3f v1);
	static Eigen::Matrix3f skew_symmetric_cross(Eigen::Vector3f v);
	static void CHECK_NUMERICAL_ERROR(const std::string& _desc, const double& _error);
	static void CHECK_NUMERICAL_ERROR(const std::string & _desc, const double & _value1, const double & _value_2);
	static int downSample(std::vector<Eigen::Vector3f> input, std::vector<Eigen::Vector3f> & output, int num_of_samples);
	static int upSample(std::vector<Eigen::Vector3f> input, std::vector<Eigen::Vector3f> input_normals, 
		std::vector<Eigen::Vector3f> & output, std::vector<Eigen::Vector3f> & output_normals, int num_of_samples);

private:
	static void sort(QList<double> &lens, QList<int> &indices, int low, int high);
	static int partition(QList<double> &subs, QList<int> &indices, int low, int high);
	static int searchPoint(Point3 searchPoint, PointList points);
	static QVector<int> searchPoints(Point3 sps[3], PointList points);
	static int mat_no;
};

#endif // UTILS_H
