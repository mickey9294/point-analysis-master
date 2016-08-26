#ifndef CUBOIDSYMMETRYGROUP_H
#define CUBOIDSYMMETRYGROUP_H

#include "papart.h"
#include <ANN/ANN.h>
#include <Eigen/Core>
#include <array>
#include <vector>
#include "definitions.h"
#include "constants.h"
#include "utils.h"

typedef enum
{
	ReflectionSymmetryType,
	RotationSymmetryType
}
CuboidSymmetryGroupType;

struct CuboidSymmetryGroupInfo
{
	CuboidSymmetryGroupInfo();
	CuboidSymmetryGroupInfo(CuboidSymmetryGroupType _symmetry_type,
		unsigned int _aligned_global_axis_index);
	CuboidSymmetryGroupInfo(const CuboidSymmetryGroupInfo& _other);

	CuboidSymmetryGroupType symmetry_type_;
	unsigned int aligned_global_axis_index_;
	std::vector<LabelIndex> single_label_indices_;
	std::vector< std::pair<LabelIndex, LabelIndex> > pair_label_indices_;
};

class CuboidSymmetryGroup
{
public:
	CuboidSymmetryGroup();
	CuboidSymmetryGroup(const CuboidSymmetryGroupInfo &_info);
	CuboidSymmetryGroup(const CuboidSymmetryGroup &_other);
	~CuboidSymmetryGroup();

	struct WeightedPointPair
	{
		WeightedPointPair(const Eigen::Vector3f _p1, const Eigen::Vector3f _p2,
			const Real _weight = 1.0, Real _angle = 0.0)
			: p1_(_p1), p2_(_p2), weight_(_weight), angle_(_angle) {}
		Eigen::Vector3f p1_;
		Eigen::Vector3f p2_;
		Real weight_;
		Real angle_;
	};


	// Virtual functions.
	virtual bool compute_symmetry_axis(const std::vector<PAPart *>& _cuboids) = 0;

	virtual Eigen::Vector3f get_symmetric_point(const Eigen::Vector3f& _point, unsigned int _symmetry_order) const = 0;
	virtual Eigen::Vector3f get_symmetric_normal(const Eigen::Vector3f& _normal, unsigned int _symmetry_order) const = 0;

	virtual CuboidSymmetryGroupType get_symmetry_type() const = 0;
	virtual unsigned int num_symmetry_orders() const;

	Real get_rotation_angle() const;
	unsigned int get_aligned_global_axis_index() const;
	void get_single_cuboid_indices(const std::vector<PAPart *>& _cuboids,
		std::vector<unsigned int> &_single_cuboid_indices) const;
	void get_pair_cuboid_indices(const std::vector<PAPart *>& _cuboids,
		std::vector< std::pair<unsigned int, unsigned int> > &_pair_cuboid_indices) const;

	void get_symmetric_sample_point_pairs(
		const std::vector<PAPart *> &_cuboids,
		const std::vector<ANNpointArray> &_cuboid_ann_points,
		const std::vector<ANNkd_tree *> &_cuboid_ann_kd_tree,
		const Real _squared_neighbor_distance,
		std::list<WeightedPointPair> &_sample_point_pairs) const;

	void get_symmetric_sample_point_pairs(
		const PAPart *_cuboid_1,
		const ANNpointArray &_cuboid_ann_points_2,
		ANNkd_tree *_cuboid_ann_kd_tree_2,
		const Real _squared_neighbor_distance,
		std::list<WeightedPointPair> &_sample_point_pairs) const;

	void get_symmetric_sample_point_pairs(
		const std::vector<Eigen::Vector3f> &_cuboid_1_sample_points,
		const ANNpointArray &_cuboid_ann_points_2,
		ANNkd_tree *_cuboid_ann_kd_tree_2,
		const Real _squared_neighbor_distance,
		std::list<WeightedPointPair> &_sample_point_pairs) const;

	CuboidSymmetryGroupInfo get_symmetry_group_info()const { return info_; }

protected:
	const CuboidSymmetryGroupInfo info_;
	unsigned int num_symmetry_orders_;  /* 一般为2 */
};

class CuboidReflectionSymmetryGroup : public CuboidSymmetryGroup
{
public:
	static CuboidReflectionSymmetryGroup* constructor(
		const CuboidSymmetryGroupInfo &_info,
		const std::vector<PAPart *>& _cuboids);
	virtual ~CuboidReflectionSymmetryGroup();

	CuboidReflectionSymmetryGroup(const Eigen::Vector3f _n, const double _t);
	CuboidReflectionSymmetryGroup(const CuboidSymmetryGroupInfo &_info);
	CuboidReflectionSymmetryGroup(const CuboidReflectionSymmetryGroup &_other);


	// Virtual functions.
	virtual bool compute_symmetry_axis(const std::vector<PAPart *>& _cuboids);

	virtual Eigen::Vector3f get_symmetric_point(const Eigen::Vector3f& _point, unsigned int _symmetry_order) const;
	virtual Eigen::Vector3f get_symmetric_normal(const Eigen::Vector3f& _normal, unsigned int _symmetry_order) const;
	virtual Eigen::Vector3f get_symmetric_point(const Eigen::Vector3f& _point) const;
	virtual Eigen::Vector3f get_symmetric_normal(const Eigen::Vector3f& _normal) const;

	virtual CuboidSymmetryGroupType get_symmetry_type() const;
	virtual unsigned int num_symmetry_orders() const;
	virtual void set_num_symmetry_order(unsigned int _symmetry_order);

	void get_reflection_plane(Eigen::Vector3f &_n, double &_t) const;
	void set_reflection_plane(const Eigen::Vector3f &_n, const double &_t);
	void get_reflection_plane_corners(const Eigen::Vector3f &_point, const Real _size,
		std::array<Eigen::Vector3f, 4>& _corners) const;


	// static functions.
	static unsigned int num_axis_parameters();

	static void add_symmety_cuboid_corner_points(
		const PAPart *cuboid_1, const PAPart *cuboid_2,
		const std::vector<PAPart *>& _cuboids,
		const unsigned int _reflection_axis_index,
		std::list< std::pair < Eigen::Vector3f, Eigen::Vector3f > > &_point_pairs);

	static void compute_reflection_plane(
		const std::list< std::pair < Eigen::Vector3f, Eigen::Vector3f > > &_point_pairs,
		Eigen::Vector3f &_n, double &_t);

private:
	// For any point p on the reflection plane, dot(n, p) = t.
	Eigen::Vector3f n_;  /* 对称平面法向量 */
	double t_;  /* 对称平面的偏移量 */
};

class CuboidRotationSymmetryGroup : public CuboidSymmetryGroup
{
public:
	static CuboidRotationSymmetryGroup* constructor(
		const CuboidSymmetryGroupInfo &_info,
		const std::vector<PAPart *>& _cuboids);
	virtual ~CuboidRotationSymmetryGroup();

	CuboidRotationSymmetryGroup(const CuboidSymmetryGroupInfo &_info);
	CuboidRotationSymmetryGroup(const CuboidRotationSymmetryGroup &_other);


	// Virtual functions.
	virtual bool compute_symmetry_axis(const std::vector<PAPart *>& _cuboids);

	virtual Eigen::Vector3f get_symmetric_point(const Eigen::Vector3f& _point, unsigned int _symmetry_order) const;
	virtual Eigen::Vector3f get_symmetric_normal(const Eigen::Vector3f& _normal, unsigned int _symmetry_order) const;

	virtual CuboidSymmetryGroupType get_symmetry_type() const;


	bool compute_rotation_angle(const std::vector<PAPart *> &_cuboids);

	void get_rotation_axis(Eigen::Vector3f &_n, Eigen::Vector3f &_t) const;
	void set_rotation_axis(const Eigen::Vector3f &_n, const Eigen::Vector3f &_t);
	void get_rotation_axis_corners(const Eigen::Vector3f &_point, const Real _size,
		std::array<Eigen::Vector3f, 2>& _corners) const;


	// static functions.
	static unsigned int num_axis_parameters();

	static void add_symmety_cuboid_corner_points(
		const PAPart *cuboid_1, const PAPart *cuboid_2,
		const std::vector<PAPart *>& _cuboids,
		const unsigned int _reflection_axis_index,
		std::list< std::pair < Eigen::Vector3f, Eigen::Vector3f > > &_point_pairs);

	static void compute_rotation_plane(
		const std::list< std::pair < Eigen::Vector3f, Eigen::Vector3f > > &_point_pairs,
		Eigen::Vector3f &_n, double &_t);

private:
	// For any point p on the rotation axis, p = nx + t (x is scalar parameter).
	Eigen::Vector3f n_;
	Eigen::Vector3f t_;
};

#endif