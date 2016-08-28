#ifndef CUBOIDRELATION_H
#define CUBOIDRELATION_H

#include <vector>
#include <Eigen/Core>
#include "papart.h"
#include "utils.h"

class CuboidAttributes{
public:
	CuboidAttributes();
	CuboidAttributes(const std::string object_name);
	CuboidAttributes(const CuboidAttributes &other);
	virtual ~CuboidAttributes();

	// [0 - 23] : 8 corner points.
	static const unsigned int k_num_attributes = 24;
	//static const unsigned int k_center_index = 0;
	static const unsigned int k_corner_index = 0;
	// Assume that y = 0 plane is the ground plane, and +y-axis is the normal direction.
	static const Eigen::Vector3f k_up_direction;

	void init();
	void compute_attributes(const OBB *obb);
	void compute_attributes(const PAPart *part);
	const Eigen::VectorXd get_attributes() const { return m_attributes; }
	bool has_nan() const { return m_attributes.hasNaN(); }
	static void get_attribute_collection_matrix(const std::list<CuboidAttributes *>& _stats, Eigen::MatrixXd & _values);
	static bool save_attribute_collection(const std::list<CuboidAttributes *> & _stats, const char *_filename);

private:
	std::string m_object_name;
	Eigen::VectorXd m_attributes;
};

class CuboidFeatures{
public:
	CuboidFeatures();
	CuboidFeatures(const std::string object_name);
	CuboidFeatures(const CuboidFeatures &other);
	virtual ~CuboidFeatures();

	// [0 - 2] : center (local coordinate point).
	// [3 - 26] : 8 corner points  (local coordinate point).
	// [27] : center height.
	// [28 - 35] : corner heights.
	static const int k_num_features = 36;
	static const int k_corner_index = 3;
	// NOTE:
	// First 'k_num_local_coord_values' values are local coordinate values.
	static const int k_num_local_points = 9;
	static const int k_num_global_feature_values = k_num_features - 3 * k_num_local_points;

	void init();
	void compute_features(const OBB *obb, Eigen::MatrixXd *attributes_to_features_map = NULL);
	void compute_features(const PAPart *part, Eigen::MatrixXd *attributes_to_features_map = NULL);
	const Eigen::VectorXd get_features() const { return m_features; }
	bool has_nan() const { return m_features.hasNaN(); }
	static void get_feature_collection_matrix(const std::list<CuboidFeatures *>& _stats, Eigen::MatrixXd &_values);
	static bool load_feature_collection(const char *_filename, std::list<CuboidFeatures *> & _stats);
	static bool save_feature_collection(const char *_filename, const std::list<CuboidFeatures *> & _stats);

private:
	std::string m_object_name;
	Eigen::VectorXd m_features;
};

class CuboidTransformation{
public:
	CuboidTransformation();
	CuboidTransformation(const std::string object_name);
	virtual ~CuboidTransformation();

	void init();
	void compute_transformation(const OBB *obb);
	void compute_transformation(const PAPart *part);
	Eigen::VectorXd get_transformed_features(const CuboidFeatures & _other_features) const;
	Eigen::VectorXd get_transformed_features(const OBB * other_obb) const;
	Eigen::VectorXd get_inverse_transformed_features(const CuboidFeatures & other_features) const;
	Eigen::VectorXd get_inverse_transformed_features(const OBB * other_obb) const;

	void get_transformation(Eigen::Matrix3d & rotation, Eigen::Vector3d & translation) const;
	void get_inverse_transformation(Eigen::Matrix3d & rotation, Eigen::Vector3d & translation) const;
	void get_linear_map_transformation(Eigen::MatrixXd & rotation, Eigen::MatrixXd & translation) const;
	void get_linear_map_inverse_transformation(Eigen::MatrixXd & rotation, Eigen::MatrixXd & translation) const;

	static bool load_transformation_collection(const char * _filename, std::list<CuboidTransformation *> & _stats);
	static bool save_transformation_collection(const char * _filename, const std::list<CuboidTransformation *> & _stats);

private:
	std::string m_object_name;
	Eigen::Vector3d m_first_translation;
	Eigen::Matrix3d m_second_rotation;
};

class CuboidJointNormalRelations{
public:
	CuboidJointNormalRelations();
	~CuboidJointNormalRelations();

	static const int k_mat_size =
		2 * (2 * CuboidFeatures::k_num_features - CuboidFeatures::k_corner_index);

	static void get_pairwise_cuboid_features(const OBB *_cuboid_1, const OBB *_cuboid_2,
		const CuboidTransformation *_transformation1, const CuboidTransformation *_transformation_2,
		Eigen::VectorXd &_pairwise_features_vec);

	static void get_pairwise_cuboid_features(
		const CuboidFeatures &_features_1, const CuboidFeatures &_features_2,
		const CuboidTransformation *_transformation_1, const CuboidTransformation *_transformation_2,
		Eigen::VectorXd &_pairwise_features_vec);

	double compute_error(const OBB *_cuboid_1, const OBB *_cuboid_2,
		const CuboidTransformation *_transformation_1, const CuboidTransformation *_transformation_2)const;

	// 1 is fixed and 2 is unknown.
	double compute_conditional_error(const OBB *_cuboid_1, const OBB *_cuboid_2,
		const CuboidTransformation *_transformation_1)const;

	const Eigen::VectorXd &get_mean()const { return mean_; }
	const Eigen::MatrixXd &get_inv_cov()const { return inv_cov_; }

	void set_mean(const Eigen::VectorXd &_mean) { mean_ = _mean; }
	void set_inv_cov(const Eigen::MatrixXd &_inv_cov_) { inv_cov_ = _inv_cov_; }

private:
	Eigen::VectorXd mean_;
	Eigen::MatrixXd inv_cov_;
};

class CuboidCondNormalRelations {
public:
	CuboidCondNormalRelations();
	~CuboidCondNormalRelations();

	static void get_pairwise_cuboid_features(
		const OBB *_cuboid_1, const OBB *_cuboid_2,
		const CuboidTransformation *_transformation_1, const CuboidTransformation *_transformation_2,
		Eigen::VectorXd &_global_features_vec_1, Eigen::VectorXd &_transformed_features_vec_12);

	static void get_pairwise_cuboid_features(
		const CuboidFeatures &_features_1, const CuboidFeatures &_features_2,
		const CuboidTransformation *_transformation_1, const CuboidTransformation *_transformation_2,
		Eigen::VectorXd &_global_features_vec_1, Eigen::VectorXd &_transformed_features_vec_12);

	double compute_error(const OBB *_cuboid_1, const OBB *_cuboid_2,
		const CuboidTransformation *_transformation_1, const CuboidTransformation *_transformation_2)const;

	const Eigen::MatrixXd &get_mean_A()const { return mean_A_; }
	const Eigen::VectorXd &get_mean_b()const { return mean_b_; }
	const Eigen::MatrixXd &get_inv_cov()const { return inv_cov_; }

	void set_mean_A(const Eigen::MatrixXd &_mean_A) { mean_A_ = _mean_A; }
	void set_mean_b(const Eigen::VectorXd &_mean_b) { mean_b_ = _mean_b; }
	void set_inv_cov(const Eigen::MatrixXd &_inv_cov_) { inv_cov_ = _inv_cov_; }

private:
	Eigen::MatrixXd mean_A_;
	Eigen::VectorXd mean_b_;
	Eigen::MatrixXd inv_cov_;
};

#endif