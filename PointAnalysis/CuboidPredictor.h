#ifndef CUBOIDPREDICTOR_H
#define CUBOIDPREDICTOR_H

#include "obb.h"
#include "papart.h"
#include "PAPointCloud.h"
#include "CuboidRelation.h"
#include "definitions.h"
#include <vector>

class CuboidPredictor
{
public:
	CuboidPredictor(unsigned int _num_labels);
	~CuboidPredictor();

	virtual void get_missing_label_indices(
		const std::list<LabelIndex> &_given_label_indices,
		std::list<LabelIndex> &_missing_label_indices)const;

	virtual Real get_single_potential(const PAPart *_cuboid,
		const CuboidAttributes *_attributes,
		const CuboidTransformation *_transformation,
		const LabelIndex _label_index)const;

	virtual Real get_pair_potential(const PAPart *_cuboid_1, const PAPart *_cuboid_2,
		const CuboidAttributes *_attributes_1, const CuboidAttributes *_attributes_2,
		const CuboidTransformation *_transformation_1, const CuboidTransformation *_transformation_2,
		const LabelIndex _label_index_1, const LabelIndex _label_index_2)const;

	virtual void get_single_quadratic_form(PAPart *_cuboid, const unsigned int _cuboid_index,
		Eigen::MatrixXd &_quadratic_term, Eigen::VectorXd &_linear_term, double& _constant_term)const;

	virtual Real get_pair_quadratic_form(const PAPart *_cuboid_1, const PAPart *_cuboid_2,
		const unsigned int _cuboid_index_1, const unsigned int _cuboid_index_2,
		const LabelIndex _label_index_1, const LabelIndex _label_index_2,
		Eigen::MatrixXd &_quadratic_term, Eigen::VectorXd &_linear_term, double& _constant_term)const;

	// 1 is fixed and 2 is unknown.
	virtual Real get_pair_conditional_quadratic_form(const OBB *_cuboid_1, const OBB *_cuboid_2,
		const unsigned int _cuboid_index_1, const unsigned int _cuboid_index_2,
		const LabelIndex _label_index_1, const LabelIndex _label_index_2,
		Eigen::MatrixXd &_quadratic_term, Eigen::VectorXd &_linear_term, double& _constant_term)const;

protected:
	const int num_labels_;
};

// Use joint normal relations for binary terms.
class CuboidJointNormalRelationPredictor : public CuboidPredictor{
public:
	CuboidJointNormalRelationPredictor(
		const std::vector< std::vector<CuboidJointNormalRelations *> > &_relations);

	virtual void get_missing_label_indices(
		const std::list<LabelIndex> &_given_label_indices,
		std::list<LabelIndex> &_missing_label_indices)const;

	virtual Real get_pair_potential(const PAPart *_cuboid_1, const PAPart *_cuboid_2,
		const CuboidAttributes *_attributes_1, const CuboidAttributes *_attributes_2,
		const CuboidTransformation *_transformation_1, const CuboidTransformation *_transformation_2,
		const LabelIndex _label_index_1, const LabelIndex _label_index_2)const;

	virtual Real get_pair_quadratic_form(const PAPart *_cuboid_1, const PAPart *_cuboid_2,
		const LabelIndex _label_index_1, const LabelIndex _label_index_2,
		const unsigned int _cuboid_index_1, const unsigned int _cuboid_index_2,
		Eigen::MatrixXd &_quadratic_term, Eigen::VectorXd &_linear_term, double& _constant_term)const;

	// 1 is fixed and 2 is unknown.
	virtual Real get_pair_conditional_quadratic_form(const PAPart *_cuboid_1, const PAPart *_cuboid_2,
		const LabelIndex _label_index_1, const LabelIndex _label_index_2,
		const unsigned int _cuboid_index_1, const unsigned int _cuboid_index_2,
		Eigen::MatrixXd &_quadratic_term, Eigen::VectorXd &_linear_term, double& _constant_term)const;

private:
	const std::vector< std::vector<CuboidJointNormalRelations *> > &relations_;
};

// Use conditional normal relations for binary terms.
class CuboidCondNormalRelationPredictor : public CuboidPredictor{
public:
	CuboidCondNormalRelationPredictor(
		const std::vector< std::vector<CuboidCondNormalRelations *> > &_relations);

	virtual void get_missing_label_indices(
		const std::list<LabelIndex> &_given_label_indices,
		std::list<LabelIndex> &_missing_label_indices)const;

	virtual Real get_pair_potential(const PAPart *_cuboid_1, const PAPart *_cuboid_2,
		const CuboidAttributes *_attributes_1, const CuboidAttributes *_attributes_2,
		const CuboidTransformation *_transformation_1, const CuboidTransformation *_transformation_2,
		const LabelIndex _label_index_1, const LabelIndex _label_index_2)const;

	virtual Real get_pair_quadratic_form(const PAPart *_cuboid_1, const PAPart *_cuboid_2,
		const LabelIndex _label_index_1, const LabelIndex _label_index_2,
		const unsigned int _cuboid_index_1, const unsigned int _cuboid_index_2,
		Eigen::MatrixXd &_quadratic_term, Eigen::VectorXd &_linear_term, double& _constant_term)const;

private:
	const std::vector< std::vector<CuboidCondNormalRelations *> > &relations_;
};

#endif