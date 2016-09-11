#include "partssolver.h"


PartsSolver::PartsSolver()
{
}


PartsSolver::~PartsSolver()
{
}

void PartsSolver::optimizeAttributes(PartsStructure & parts_structure,
	const CuboidPredictor &_predictor,
	const double _single_energy_term_weight,
	const double _symmetry_energy_term_weight,
	const int _max_num_iterations,
	bool _use_symmetry)
{
	std::stringstream sstr;

	int num_labels = parts_structure.num_of_labels();

	std::vector<PAPart *> all_parts = parts_structure.get_all_parts();
	int num_parts = all_parts.size();

	double single_total_energy, pair_total_energy, total_energy;


	// NOTE: Keep the lowest energy cuboids.
	double final_total_energy = std::numeric_limits<double>::max();
	unsigned int final_cuboid_iteration = 0;
	PartsStructure final_parts_structure = parts_structure;

	update_cuboid_surface_points(parts_structure);

	get_optimization_error(all_parts, _predictor, single_total_energy, pair_total_energy);
	total_energy = pair_total_energy + _single_energy_term_weight * single_total_energy;
	std::cout << "Energy: (pair = " << pair_total_energy << ", single = " << _single_energy_term_weight * single_total_energy
		<< ", total = " << total_energy << ")" << std::endl << std::endl;

	int iteration = 1;
	for (; iteration <= _max_num_iterations; iteration++)
	{
		sstr.str(std::string());
		sstr << "iteration [" << iteration << "]" << std::endl;
		std::cout << sstr.str();

		// NOTE: When jointly optimizing all reflection symmetry groups which have
		// orthogonal relations each other, the result might go wrong due to the
		// numerical issue in the solver. We therefore optimize for each reflection
		// symmetry group separately.
		if (optimize_individual_reflection_symmetry_group)
		{
			const std::vector<CuboidReflectionSymmetryGroup *> all_reflection_symmetry_groups
				= parts_structure.m_reflection_symmetry_groups;

			for (std::vector<CuboidReflectionSymmetryGroup *>::const_iterator it =
				all_reflection_symmetry_groups.begin(); it != all_reflection_symmetry_groups.end(); ++it)
			{
				CuboidReflectionSymmetryGroup *reflection_symmetry_groups = (*it);

				std::vector<CuboidReflectionSymmetryGroup *> each_reflection_symmetry_groups;
				each_reflection_symmetry_groups.push_back(reflection_symmetry_groups);
				parts_structure.m_reflection_symmetry_groups = each_reflection_symmetry_groups;

				optimize_attributes_once(
					parts_structure, _predictor,
					_single_energy_term_weight, _symmetry_energy_term_weight,
					_use_symmetry);
			}

			parts_structure.m_reflection_symmetry_groups = all_reflection_symmetry_groups;
		}
		else
		{
			optimize_attributes_once(
				parts_structure, _predictor,
				_single_energy_term_weight, _symmetry_energy_term_weight,
				_use_symmetry);
		}

		get_optimization_error(all_parts, _predictor, single_total_energy, pair_total_energy);
		total_energy = pair_total_energy + _single_energy_term_weight * single_total_energy;
		sstr.str(std::string());
		sstr << " - After optimization" << std::endl;
		sstr << "Energy: (pair = " << pair_total_energy
			<< ", single = " << _single_energy_term_weight * single_total_energy
			<< ", total = " << total_energy << ")" << std::endl;
		std::cout << sstr.str();

		/* Cuboidize */
		if (_use_symmetry)
		{
			/* Cuboid axes are estimated in the optimization. */
			for (std::vector<PAPart *>::const_iterator it = all_parts.begin(); it != all_parts.end(); ++it)
				//(*it)->update_center_size_corner_points();
				(*it)->updateOBB();
		}
		else
		{
			for (std::vector<PAPart *>::const_iterator it = all_parts.begin(); it != all_parts.end(); ++it)
				(*it)->update_axes_center_size_corner_points();
		}

		update_cuboid_surface_points(parts_structure);

		get_optimization_error(all_parts, _predictor,
			single_total_energy, pair_total_energy);
		total_energy = pair_total_energy + _single_energy_term_weight * single_total_energy;
		sstr.str(std::string());
		sstr << " - After cuboidization" << std::endl;
		sstr << "Energy: (pair = " << pair_total_energy
			<< ", single = " << _single_energy_term_weight * single_total_energy
			<< ", total = " << total_energy << ")";
		if (total_energy < final_total_energy)
			sstr << " - Minimum.";
		sstr << std::endl;

		std::cout << sstr.str();

		if (total_energy < final_total_energy)
		{
			final_total_energy = total_energy;
			final_cuboid_iteration = iteration;
			final_parts_structure = parts_structure;
		}
		else if (total_energy > 3.0 * final_total_energy)
		{
			sstr.str(std::string());
			sstr << "Last iteration is worse than the previous one..." << std::endl;
			sstr << "Recover to the previous iteration... Stop." << std::endl;
			std::cout << sstr.str();

			break;
		}
	}

	if (iteration >= _max_num_iterations)
	{
		sstr.str(std::string());
		sstr << "# of iteration exceeds maximum number of iterations ("
			<< _max_num_iterations << ") ... Stop." << std::endl;
		std::cout << sstr.str();
	}
	
	/* Copy final result */
	parts_structure = final_parts_structure;
	all_parts = parts_structure.get_all_parts();
	num_parts = all_parts.size();

	update_cuboid_surface_points(parts_structure);

	sstr.str(std::string());
	sstr << "Final cuboid iteration: " << final_cuboid_iteration << std::endl;
	std::cout << sstr.str();

	get_optimization_error(all_parts, _predictor,
		single_total_energy, pair_total_energy);
	total_energy = pair_total_energy + _single_energy_term_weight * single_total_energy;
	sstr.str(std::string());
	sstr << "Energy: (pair = " << pair_total_energy
		<< ", single = " << _single_energy_term_weight * single_total_energy
		<< ", total = " << total_energy << ")" << std::endl;
	std::cout << sstr.str();
}

void PartsSolver::get_optimization_error(
	const std::vector<PAPart *> & parts,
	const CuboidPredictor &_predictor,
	double &_single_total_energy, double &_pair_total_energy)
{
	Eigen::VectorXd init_values;
	Eigen::MatrixXd single_quadratic_term, pair_quadratic_term;
	Eigen::VectorXd single_linear_term, pair_linear_term;
	double single_constant_term, pair_constant_term;

	get_optimization_formulation(parts, _predictor, init_values,
		single_quadratic_term, pair_quadratic_term,
		single_linear_term, pair_linear_term,
		single_constant_term, pair_constant_term,
		_single_total_energy, _pair_total_energy);
}

void PartsSolver::get_optimization_formulation(
	const std::vector<PAPart *> & parts,
	const CuboidPredictor &_predictor,
	Eigen::VectorXd &_init_values,
	Eigen::MatrixXd &_single_quadratic_term, Eigen::MatrixXd &_pair_quadratic_term,
	Eigen::VectorXd &_single_linear_term, Eigen::VectorXd &_pair_linear_term,
	double &_single_constant_term, double &_pair_constant_term,
	double &_single_total_energy, double &_pair_total_energy)
{
	const int num_attributes = CuboidAttributes::k_num_attributes;
	int num_cuboids = parts.size();
	int mat_size = num_cuboids * num_attributes;

	_single_quadratic_term = Eigen::MatrixXd(mat_size, mat_size);
	_pair_quadratic_term = Eigen::MatrixXd(mat_size, mat_size);

	_single_linear_term = Eigen::VectorXd(mat_size);
	_pair_linear_term = Eigen::VectorXd(mat_size);

	_single_quadratic_term.setZero(); _pair_quadratic_term.setZero();
	_single_linear_term.setZero(); _pair_linear_term.setZero();
	_single_constant_term = 0; _pair_constant_term = 0;

	_init_values = Eigen::VectorXd(mat_size);

	for (unsigned int cuboid_index = 0; cuboid_index < num_cuboids; ++cuboid_index)  /* 对每个包围盒 */
	{
		OBB *cuboid = parts[cuboid_index]->getOBB();
		CuboidAttributes attributes;
		attributes.compute_attributes(cuboid);  /* 计算属性值，其实为包围盒8个顶点的坐标值 */
		_init_values.segment<num_attributes>(num_attributes * cuboid_index)
			= attributes.get_attributes();
	}

	// Single energy (ICP prior energy).
	for (unsigned int cuboid_index = 0; cuboid_index < num_cuboids; ++cuboid_index)
	{
		PAPart *cuboid = parts[cuboid_index];
		//LabelIndex label_index = cuboid->get_label_index();

		Eigen::MatrixXd each_single_quadratic_term(mat_size, mat_size);
		Eigen::VectorXd each_single_linear_term(mat_size);
		double each_single_constant_term;

		_predictor.get_single_quadratic_form(cuboid, cuboid_index,
			each_single_quadratic_term, each_single_linear_term, each_single_constant_term);

		_single_quadratic_term = _single_quadratic_term + each_single_quadratic_term;
		_single_linear_term = _single_linear_term + each_single_linear_term;
		_single_constant_term = _single_constant_term + each_single_constant_term;
	}

	_single_total_energy = 0;
	_single_total_energy += _init_values.transpose() * _single_quadratic_term * _init_values;
	_single_total_energy += 2 * _single_linear_term.transpose() * _init_values;
	_single_total_energy += _single_constant_term;

	/* Pairwise energy */
	double same_pair_total_energy = 0;

	/* for every cuboid */
	for (int cuboid_index_1 = 0; cuboid_index_1 < num_cuboids; ++cuboid_index_1)
	{
		PAPart * cuboid_1 = parts[cuboid_index_1];
		LabelIndex label_index_1 = cuboid_1->getLabel();

		//for (unsigned int cuboid_index_2 = 0; cuboid_index_2 < num_cuboids; ++cuboid_index_2)
		//{
		//	if (cuboid_index_1 == cuboid_index_2)
		//		continue;

		// Using bilateral relations.
		// (A, B) and (B, A) pairs are the same.
		//for (unsigned int cuboid_index_2 = cuboid_index_1 + 1; cuboid_index_2 < num_cuboids; ++cuboid_index_2)
		for (unsigned int cuboid_index_2 = cuboid_index_1; cuboid_index_2 < num_cuboids; ++cuboid_index_2)
		{
			PAPart *cuboid_2 = parts[cuboid_index_2];
			LabelIndex label_index_2 = cuboid_2->getLabel();

			Eigen::MatrixXd each_pair_quadratic_term(mat_size, mat_size);
			Eigen::VectorXd each_pair_linear_term(mat_size);
			double each_pair_constant_term;

			Real energy = _predictor.get_pair_quadratic_form(cuboid_1, cuboid_2,
				cuboid_index_1, cuboid_index_2,
				label_index_1, label_index_2,
				each_pair_quadratic_term, each_pair_linear_term, each_pair_constant_term);  /* 得到的二元二次项是零矩阵，二元一次项是零向量，二元常数项是0 */

			_pair_quadratic_term = _pair_quadratic_term + each_pair_quadratic_term;
			_pair_linear_term = _pair_linear_term + each_pair_linear_term;
			_pair_constant_term = _pair_constant_term + each_pair_constant_term;

			same_pair_total_energy += energy;
		}
	}

	_pair_total_energy = 0;
	_pair_total_energy += _init_values.transpose() * _pair_quadratic_term * _init_values;
	_pair_total_energy += 2 * _pair_linear_term.transpose() * _init_values;
	_pair_total_energy += _pair_constant_term;

#ifdef DEBUG_TEST
	double err = std::abs(_pair_total_energy - same_pair_total_energy);
	Utils::CHECK_NUMERICAL_ERROR(__FUNCTION__, _pair_total_energy, same_pair_total_energy);
#endif
}

void PartsSolver::optimize_attributes_once(
	PartsStructure & parts_structure,
	const CuboidPredictor& _predictor,
	const double _single_energy_term_weight,
	const double _symmetry_energy_term_weight,
	bool _use_symmetry)
{
	const Real squared_neighbor_distance = SPARSE_NEIGHBOR_DISTANCE * SPARSE_NEIGHBOR_DISTANCE
		* parts_structure.m_radius;
	
	std::vector<PAPart *> all_parts = parts_structure.get_all_parts();
	std::vector<CuboidReflectionSymmetryGroup *> all_reflection_symmetry_groups =
		parts_structure.m_reflection_symmetry_groups;
	std::vector<CuboidRotationSymmetryGroup *> all_rotation_symmetry_groups =
		parts_structure.m_rotation_symmetry_groups;

	const int num_attributes = CuboidAttributes::k_num_attributes;
	int num_cuboids = all_parts.size();
	int mat_size = num_cuboids * num_attributes;

	Eigen::VectorXd init_values;
	Eigen::MatrixXd single_quadratic_term, pair_quadratic_term;  /* 一元二次项，二元二次项 */
	Eigen::VectorXd single_linear_term, pair_linear_term;  /* 一元一次项，二元一次项 */
	double single_constant_term, pair_constant_term;  /* 一元常数项，二元常数项 */
	double single_total_energy, pair_total_energy;  /* 一元总能量， 二元总能量 */

	get_optimization_formulation(all_parts, _predictor, init_values,
		single_quadratic_term, pair_quadratic_term,
		single_linear_term, pair_linear_term,
		single_constant_term, pair_constant_term,
		single_total_energy, pair_total_energy);

	Eigen::MatrixXd quadratic_term = pair_quadratic_term + _single_energy_term_weight * single_quadratic_term;  /* 二次项 = 二元二次项 + 一元能量权值 * 一元二次项 */
	Eigen::VectorXd linear_term = pair_linear_term + _single_energy_term_weight * single_linear_term;  /* 一次项 = 二元一次项 + 一元能量权值 * 一元一次项 */
	double constant_term = pair_constant_term + _single_energy_term_weight * single_constant_term;  /* 常数项 = 二元常数项 + 一元能量权值 * 一元常数项 */

	if (!_use_symmetry)
	{
		all_reflection_symmetry_groups.clear();
		all_rotation_symmetry_groups.clear();
	}

	CuboidNonLinearSolver non_linear_solver(
		all_parts,
		all_reflection_symmetry_groups,
		all_rotation_symmetry_groups,
		squared_neighbor_distance,
		min_num_symmetric_point_pairs,
		_symmetry_energy_term_weight);

	non_linear_solver.optimize(quadratic_term, linear_term, constant_term, &init_values);
	cout << endl;
}

void PartsSolver::update_cuboid_surface_points(PartsStructure & parts_structure)
{
	const Real radius = param_occlusion_test_neighbor_distance * parts_structure.m_radius;
	
	std::vector<PAPart *> all_parts = parts_structure.get_all_parts();
	for (std::vector<PAPart *>::iterator it = all_parts.begin(); it != all_parts.end(); ++it)
	{
		PAPart *cuboid = *it;
		cuboid->create_grid_points_on_obb_surface();
	}
}