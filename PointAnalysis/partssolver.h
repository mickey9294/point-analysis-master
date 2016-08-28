#ifndef PARTSSOLVER_H
#define PARTSSOLVER_H

#include "energyfunctions.h"
#include "definitions.h"
#include "constants.h"
#include "cuboidrelation.h"
#include "CuboidPredictor.h"
#include "CuboidSymmetryGroup.h"
#include "CuboidNonLinearSolver.h"
#include "PartsStructure.h"
#include <Eigen/Core>

class PartsSolver
{
public:
	PartsSolver();
	~PartsSolver();

	void optimizeAttributes(PartsStructure & parts_structure,
		const CuboidPredictor &_predictor,
		const double _single_energy_term_weight,
		const double _symmetry_energy_term_weight,
		const int _max_num_iterations,
		bool _use_symmetry);
	
	void get_optimization_error(
		const std::vector<PAPart *> & parts,
		const CuboidPredictor &_predictor,
		double &_single_totoal_energy, double &_pair_total_energy);

	void get_optimization_formulation(
		const std::vector<PAPart *> &parts,
		const CuboidPredictor &_predictor,
		Eigen::VectorXd &_init_values,
		Eigen::MatrixXd &_single_quadratic_term, Eigen::MatrixXd &_pair_quadratic_term,
		Eigen::VectorXd &_single_linear_term, Eigen::VectorXd &_pair_linear_term,
		double &_single_constant_term, double &_pair_constant_term,
		double &_single_total_energy, double &_pair_total_energy);

	void optimize_attributes_once(
		PartsStructure & parts_structure,
		const CuboidPredictor& _predictor,
		const double _single_energy_term_weight,
		const double _symmetry_energy_term_weight,
		bool _use_symmetry);

private:
	void update_cuboid_surface_points(PartsStructure & parts_structure);
};

#endif