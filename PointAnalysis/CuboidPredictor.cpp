#include "CuboidPredictor.h"

CuboidPredictor::~CuboidPredictor()
{
}

CuboidPredictor::CuboidPredictor(unsigned int _num_labels)
	: num_labels_(_num_labels)
{

}

void CuboidPredictor::get_missing_label_indices(
	const std::list<LabelIndex> &_given_label_indices,
	std::list<LabelIndex> &_missing_label_indices) const
{
	_missing_label_indices.clear();
}

Real CuboidPredictor::get_single_potential(
	const PAPart *_cuboid,
	const CuboidAttributes *_attributes,
	const CuboidTransformation *_transformation,
	const LabelIndex _label_index)const
{
	assert(_cuboid);
	assert(_attributes);
	assert(_transformation);

	Real potential = 0.0;
	unsigned int num_sample_points = 0;

	// FIXME:
	// This should be done in pre-processing.
	//unsigned int num_confidence_tol_sample_point = 0;
	//for (std::vector<MeshSamplePoint *>::const_iterator sample_it = _cuboid->get_sample_points().begin();
	//	sample_it != _cuboid->get_sample_points().end(); ++sample_it)
	//{
	//	assert(_label_index < (*sample_it)->label_index_confidence_.size());
	//	if ((*sample_it)->label_index_confidence_[_label_index] >= FLAGS_param_min_sample_point_confidence)
	//		++num_confidence_tol_sample_point;
	//}

	//if (num_confidence_tol_sample_point <
	//	FLAGS_param_min_num_confidence_tol_sample_points * _cuboid->get_sample_points().size())
	//{
	//	potential = FLAGS_param_max_potential;
	//}

	return potential;
}

Real CuboidPredictor::get_pair_potential(
	const PAPart *_cuboid_1, const PAPart *_cuboid_2,
	const CuboidAttributes *_attributes_1, const CuboidAttributes *_attributes_2,
	const CuboidTransformation *_transformation_1, const CuboidTransformation *_transformation_2,
	const LabelIndex _label_index_1, const LabelIndex _label_index_2)const
{
	assert(_cuboid_1); assert(_cuboid_2);
	assert(_attributes_1); assert(_attributes_2);
	assert(_transformation_1); assert(_transformation_2);

	// Not implemented.
	Real potential = 0.0;
	return potential;
}

void CuboidPredictor::get_single_quadratic_form(
	PAPart *_cuboid, const unsigned int _cuboid_index,
	Eigen::MatrixXd &_quadratic_term, Eigen::VectorXd &_linear_term, double& _constant_term) const
{
	assert(_cuboid);

	_quadratic_term.setZero();
	_linear_term.setZero();
	_constant_term = 0;

	OBB * obb = _cuboid->getOBB();

	unsigned int num_sample_points = _cuboid->num_of_samples;
	unsigned int num_cuboid_surface_points = obb->sampleCount();
	const unsigned int mat_size = _quadratic_term.cols();
	const unsigned int num_corners = OBB::k_num_corners;
	const unsigned int num_attributes = CuboidAttributes::k_num_attributes;

	// X: sample points, Y: cuboid surface points.
	unsigned int num_X_points = num_sample_points;
	unsigned int num_Y_points = num_cuboid_surface_points;

	if (num_X_points == 0 || num_Y_points == 0)
		return;

	double sum_visibility = 0.0;  /* 包围盒上所有采样点的可见度之和 */
	for (int cuboid_surface_point_index = 0;
		cuboid_surface_point_index < num_cuboid_surface_points;
		++cuboid_surface_point_index)
	{
		sum_visibility += obb->getSample(cuboid_surface_point_index).visibility();
	}

	if (sum_visibility == 0)
		return;

	Eigen::MatrixXd all_X_points(3, num_X_points + num_Y_points);  /* 所有模型上（采样）点 */
	Eigen::MatrixXd all_Y_points(3, num_X_points + num_Y_points);  /* 所有包围盒上采样点 */
	Eigen::MatrixXd all_Y_coeffs(num_corners, num_X_points + num_Y_points);  /* 8行X+Y列，每一列有8个元素，即每个点对应的与8个顶点有关的某特征？ */
	Eigen::VectorXd all_weights(num_X_points + num_Y_points);

	unsigned int pair_index = 0;

	// Sample point -> Cuboid surface point.
	for (int sample_point_index = 0; sample_point_index < num_sample_points; ++sample_point_index)  /* 对每个模型上点 */
	{
		int cuboid_surface_point_index =
			_cuboid->get_sample_to_cuboid_surface_correspondences(sample_point_index);  /* 求模型上点对应包围盒表面的采样点的序号 */
		assert(cuboid_surface_point_index >= 0);

		//
		Eigen::Vector3d X_point;
		Eigen::Vector3d Y_point;
		Eigen::VectorXd Y_coeff(OBB::k_num_corners);  /* 8维列向量，与包围盒八个顶点有关的特征 */

		for (unsigned int i = 0; i < 3; ++i)  /* 获取这一组的模型上点和包围盒采样点 */
		{
			X_point(i) = _cuboid->getSample(sample_point_index)[i];
			Y_point(i) = obb->getSample(cuboid_surface_point_index)[i];
		}

		for (unsigned int corner_index = 0; corner_index < OBB::k_num_corners; ++corner_index)  /* 对包围盒的8个顶点的每个顶点 */
			Y_coeff(corner_index) = obb->getSample(
			cuboid_surface_point_index).getCornerWeights()[corner_index];  /* Y_coeff的每个元素是包围盒采样点的coner_weights的相应一维 */

		all_X_points.col(pair_index) = X_point;
		all_Y_coeffs.col(pair_index) = Y_coeff;
		all_Y_points.col(pair_index) = Y_point;
		all_weights(pair_index) = 1.0 / num_sample_points;  /* 对于模型上点，相应all_weights值是 点数量分之一 */

		++pair_index;
		//
	}

	// Cuboid surface point -> Sample point.
	for (int cuboid_surface_point_index = 0;
		cuboid_surface_point_index < num_cuboid_surface_points;
		++cuboid_surface_point_index)  /* 对包围盒上的每个采样点 */
	{
		int sample_point_index =
			_cuboid->get_cuboid_surface_to_sample_correspondences(cuboid_surface_point_index);  /* 求包围盒上的一采样点对应于模型上点的索引号 */
		assert(sample_point_index >= 0);

		//
		Eigen::Vector3d X_point;
		Eigen::Vector3d Y_point;
		Eigen::VectorXd Y_coeff(OBB::k_num_corners);  /* 8维向量 */

		for (unsigned int i = 0; i < 3; ++i)
		{
			X_point(i) = _cuboid->getSample(sample_point_index)[i];
			Y_point(i) = obb->getSample(cuboid_surface_point_index)[i];
		}

		for (unsigned int corner_index = 0; corner_index < OBB::k_num_corners; ++corner_index)
			Y_coeff(corner_index) = obb->getSample(
			cuboid_surface_point_index).getCornerWeights()[corner_index];

		double visibility = obb->getSample(cuboid_surface_point_index).visibility();

		all_X_points.col(pair_index) = X_point;
		all_Y_coeffs.col(pair_index) = Y_coeff;
		all_Y_points.col(pair_index) = Y_point;
		all_weights(pair_index) = visibility / sum_visibility;   /* 对于包围盒采样点，all_weights的相应值是 可见度除以总可见度 */

		++pair_index;
		//
	}

	assert(pair_index == num_X_points + num_Y_points);

	assert(_quadratic_term.rows() == mat_size);
	assert(_linear_term.rows() == mat_size);
	assert(_cuboid_index * CuboidAttributes::k_num_attributes <= mat_size);


	CuboidAttributes attribute;
	attribute.compute_attributes(obb);
	Eigen::VectorXd x = attribute.get_attributes();  /* OBB的属性即为立方体8个顶点的坐标值 */

	for (unsigned int point_index = 0; point_index < num_X_points + num_Y_points; ++point_index)  /* 对于所有点（包括模型上点和包围盒采样点）*/
	{
		Eigen::Vector3d X_point = all_X_points.col(point_index);
		Eigen::VectorXd Y_coeff = all_Y_coeffs.col(point_index);  /* 8维向量 */
		Eigen::VectorXd Y_point = all_Y_points.col(point_index);
		assert(Y_coeff.size() == OBB::k_num_corners);
		double weight = all_weights(point_index);

		Eigen::MatrixXd A0(3, num_attributes);  /*3行24列，共有24个列向量，即每个包围盒顶点都对应了一个3*3的小矩阵 */
		A0.setZero();

		for (unsigned int corner_index = 0; corner_index < OBB::k_num_corners; ++corner_index)  /* 对于8个顶点中的每个顶点 */
		{
			for (unsigned int i = 0; i < 3; ++i)
				A0.col(CuboidAttributes::k_corner_index + 3 * corner_index + i)(i) =
				Y_coeff(corner_index);  /* 对于与A0中与每个顶点的3个坐标相对应的小矩阵，将对角线设为该顶点对应的coeff值 */
		}

#ifdef DEBUG_TEST
		Real error = std::abs(((A0 * x) - Y_point).norm());
		CHECK_NUMERICAL_ERROR(__FUNCTION__, error);
#endif

		Eigen::Vector3d b = -X_point;  /* 与模型上的点关于原点对称的点 */

		Eigen::MatrixXd A = Eigen::MatrixXd::Zero(3, mat_size);  /* mat_size = 包围盒数* 24 */
		unsigned int start_index = (_cuboid_index * CuboidAttributes::k_num_attributes);  /* 开始序号为当前包围盒编号乘24(包围盒8个顶点所含坐标数) */
		A.block<3, CuboidAttributes::k_num_attributes>(0, start_index) = A0;

		_quadratic_term = _quadratic_term + weight * (A.transpose() * A);
		_linear_term = _linear_term + weight * (A.transpose() * b);
		_constant_term = _constant_term + weight * (b.transpose() * b)(0);
	}
}

Real CuboidPredictor::get_pair_quadratic_form(
	const PAPart *_cuboid_1, const PAPart *_cuboid_2,
	const unsigned int _cuboid_index_1, const unsigned int _cuboid_index_2,
	const LabelIndex _label_index_1, const LabelIndex _label_index_2,
	Eigen::MatrixXd &_quadratic_term, Eigen::VectorXd &_linear_term, double& _constant_term)const
{
	assert(_cuboid_1); assert(_cuboid_2);
	assert(_label_index_1 < num_labels_);
	assert(_label_index_2 < num_labels_);
	//assert(_label_index_1 != _label_index_2);

	const unsigned int num_attributes = CuboidAttributes::k_num_attributes;
	const unsigned int num_features = CuboidFeatures::k_num_features;
	const unsigned int mat_size = _quadratic_term.cols();

	assert(_quadratic_term.rows() == mat_size);
	assert(_linear_term.rows() == mat_size);
	assert(_cuboid_index_1 * num_attributes <= mat_size);
	assert(_cuboid_index_2 * num_attributes <= mat_size);

	_quadratic_term.setZero();
	_linear_term.setZero();
	_constant_term = 0;

	return 0;
}

Real CuboidPredictor::get_pair_conditional_quadratic_form(
	const OBB *_cuboid_1, const OBB *_cuboid_2,
	const unsigned int _cuboid_index_1, const unsigned int _cuboid_index_2,
	const LabelIndex _label_index_1, const LabelIndex _label_index_2,
	Eigen::MatrixXd &_quadratic_term, Eigen::VectorXd &_linear_term, double& _constant_term)const
{
	assert(_cuboid_1); assert(_cuboid_2);
	assert(_label_index_1 < num_labels_);
	assert(_label_index_2 < num_labels_);
	//assert(_label_index_1 != _label_index_2);

	const unsigned int num_attributes = CuboidAttributes::k_num_attributes;
	const unsigned int num_features = CuboidFeatures::k_num_features;
	const unsigned int mat_size = _quadratic_term.cols();

	assert(_quadratic_term.rows() == mat_size);
	assert(_linear_term.rows() == mat_size);
	assert(_cuboid_index_1 * num_attributes <= mat_size);
	assert(_cuboid_index_2 * num_attributes <= mat_size);

	_quadratic_term.setZero();
	_linear_term.setZero();
	_constant_term = 0;

	return 0;
}

CuboidJointNormalRelationPredictor::CuboidJointNormalRelationPredictor(
	const std::vector< std::vector<CuboidJointNormalRelations *> > &_relations)
	: CuboidPredictor(_relations.size())
	, relations_(_relations)
{
	for (unsigned int label_index = 0; label_index < num_labels_; ++label_index)
		assert(relations_[label_index].size() == num_labels_);
}

void CuboidJointNormalRelationPredictor::get_missing_label_indices(
	const std::list<LabelIndex> &_given_label_indices,
	std::list<LabelIndex> &_missing_label_indices)const
{
	_missing_label_indices.clear();
	unsigned int num_labels = relations_.size();

	bool *is_missing_label = new bool[num_labels];
	memset(is_missing_label, true, num_labels * sizeof(bool));

	for (std::list<LabelIndex>::const_iterator it = _given_label_indices.begin();
		it != _given_label_indices.end(); ++it)
	{
		LabelIndex label_index = (*it);
		assert(label_index < num_labels);
		is_missing_label[label_index] = false;
	}

	for (LabelIndex label_index_1 = 0; label_index_1 < num_labels; ++label_index_1)
	{
		if (is_missing_label[label_index_1])
		{
			bool is_label_confliced = false;

			for (LabelIndex label_index_2 = 0; label_index_2 < num_labels; ++label_index_2)
			{
				if (!is_missing_label[label_index_2] && !relations_[label_index_1][label_index_2])
				{
					is_label_confliced = true;
					break;
				}
			}

			if (!is_label_confliced)
			{
				_missing_label_indices.push_back(label_index_1);
			}
		}
	}

	delete[] is_missing_label;
}

Real CuboidJointNormalRelationPredictor::get_pair_potential(
	const PAPart *_cuboid_1, const PAPart *_cuboid_2,
	const CuboidAttributes *_attributes_1, const CuboidAttributes *_attributes_2,
	const CuboidTransformation *_transformation_1, const CuboidTransformation *_transformation_2,
	const LabelIndex _label_index_1, const LabelIndex _label_index_2) const
{
	assert(_cuboid_1); assert(_cuboid_2);
	assert(_attributes_1); assert(_attributes_2);
	assert(_transformation_1); assert(_transformation_2);

	assert(_label_index_1 < num_labels_);
	assert(_label_index_2 < num_labels_);

	// NOTE:
	// Now considering only different label pairs.
	//assert(_label_index_1 != _label_index_2);

	Real potential = INF_POTENTIAL;

	//const CuboidJointNormalRelations *relation_12 = relations_[_label_index_1][_label_index_2];
	//const CuboidJointNormalRelations *relation_21 = relations_[_label_index_2][_label_index_1];

	//if (relation_12 && relation_21)
	//{
	//	Real potential_12 = relation_12->compute_error(_cuboid_1, _cuboid_2, _transformation_1, _transformation_2);
	//	Real potential_21 = relation_21->compute_error(_cuboid_2, _cuboid_1, _transformation_2, _transformation_1);
	//	potential = potential_12 + potential_21;
	//}

	const CuboidJointNormalRelations *relation_12 = relations_[_label_index_1][_label_index_2];
	if (relation_12)
	{
		Real potential_12 = relation_12->compute_error(_cuboid_1->getOBB(), _cuboid_2->getOBB(), _transformation_1, _transformation_2);
		potential = potential_12;

		// Using bilateral relations.
		// (A, B) and (B, A) pairs are the same.
#ifdef DEBUG_TEST
		const CuboidJointNormalRelations *relation_21 = relations_[_label_index_2][_label_index_1];
		if (relation_21)
		{
			Real potential_21 = relation_12->compute_error(_cuboid_1, _cuboid_2, _transformation_1, _transformation_2);
			CHECK_NUMERICAL_ERROR(__FUNCTION__, potential_12, potential_21);
		}
#endif
	}

	return potential;
}

Real CuboidJointNormalRelationPredictor::get_pair_quadratic_form(
	const PAPart *_cuboid_1, const PAPart *_cuboid_2,
	const LabelIndex _label_index_1, const LabelIndex _label_index_2,
	const unsigned int _cuboid_index_1, const unsigned int _cuboid_index_2,
	Eigen::MatrixXd &_quadratic_term, Eigen::VectorXd &_linear_term, double& _constant_term) const
{
	assert(_cuboid_1); assert(_cuboid_2);
	assert(_label_index_1 < num_labels_);
	assert(_label_index_2 < num_labels_);

	// NOTE:
	// Now considering only different label pairs.
	//assert(_label_index_1 != _label_index_2);

	const unsigned int num_attributes = CuboidAttributes::k_num_attributes;
	const unsigned int num_features = CuboidFeatures::k_num_features;
	const unsigned int mat_size = _quadratic_term.cols();

	assert(_quadratic_term.rows() == mat_size);
	assert(_linear_term.rows() == mat_size);
	assert(_cuboid_index_1 * num_attributes <= mat_size);
	assert(_cuboid_index_2 * num_attributes <= mat_size);

	_quadratic_term.setZero();
	_linear_term.setZero();
	_constant_term = 0;

	const CuboidJointNormalRelations *relation_12 = relations_[_label_index_1][_label_index_2];
	if (!relation_12) return 0.0;


	CuboidFeatures features_1, features_2;
	Eigen::MatrixXd attributes_to_features_map_1, attributes_to_features_map_2;

	features_1.compute_features(_cuboid_1->getOBB(), &attributes_to_features_map_1);
	features_2.compute_features(_cuboid_2->getOBB(), &attributes_to_features_map_2);

	assert(attributes_to_features_map_1.rows() == num_features);
	assert(attributes_to_features_map_2.rows() == num_features);

	assert(attributes_to_features_map_1.cols() == num_attributes);
	assert(attributes_to_features_map_2.cols() == num_attributes);

	CuboidTransformation transformation_1;
	transformation_1.compute_transformation(_cuboid_1->getOBB());
	Eigen::MatrixXd rotation_1;
	Eigen::MatrixXd translation_1;
	transformation_1.get_linear_map_transformation(rotation_1, translation_1);

	CuboidTransformation transformation_2;
	transformation_2.compute_transformation(_cuboid_2->getOBB());
	Eigen::MatrixXd rotation_2;
	Eigen::MatrixXd translation_2;
	transformation_2.get_linear_map_transformation(rotation_2, translation_2);


	// (Ax + b)'C(Ax + b) = x'(A'CA)x + 2*(b'CA)x.
	unsigned int start_index_1 = (_cuboid_index_1 * num_attributes);
	unsigned int start_index_2 = (_cuboid_index_2 * num_attributes);

	Eigen::MatrixXd A1_orig = Eigen::MatrixXd::Zero(2 * num_features, mat_size);
	Eigen::MatrixXd A2_orig = Eigen::MatrixXd::Zero(2 * num_features, mat_size);


	// The rotation is fixed, and attribute-to-feature map is a function of 'cuboid_1' attributes.
	A1_orig.block<num_features, num_attributes>(0, start_index_1)
		= A1_orig.block<num_features, num_attributes>(0, start_index_1)
		+ rotation_1 * attributes_to_features_map_1;

	// The translation is a function of 'cuboid_1' attributes.
	A1_orig.block<num_features, num_attributes>(0, start_index_1)
		= A1_orig.block<num_features, num_attributes>(0, start_index_1)
		+ translation_1;

	// The rotation is fixed, and attribute-to-feature map is a function of 'cuboid_2' attributes.
	A1_orig.block<num_features, num_attributes>(num_features, start_index_2)
		= A1_orig.block<num_features, num_attributes>(num_features, start_index_2)
		+ rotation_1 * attributes_to_features_map_2;

	// The translation is a function of 'cuboid_1' attributes.
	A1_orig.block<num_features, num_attributes>(num_features, start_index_1)
		= A1_orig.block<num_features, num_attributes>(num_features, start_index_1)
		+ translation_1;


	// The rotation is fixed, and attribute-to-feature map is a function of 'cuboid_2' attributes.
	A2_orig.block<num_features, num_attributes>(0, start_index_2)
		= A2_orig.block<num_features, num_attributes>(0, start_index_2)
		+ rotation_2 * attributes_to_features_map_2;

	// The translation is a function of 'cuboid_2' attributes.
	A2_orig.block<num_features, num_attributes>(0, start_index_2)
		= A2_orig.block<num_features, num_attributes>(0, start_index_2)
		+ translation_2;

	// The rotation is fixed, and attribute-to-feature map is a function of 'cuboid_1' attributes.
	A2_orig.block<num_features, num_attributes>(num_features, start_index_1)
		= A2_orig.block<num_features, num_attributes>(num_features, start_index_1)
		+ rotation_2 * attributes_to_features_map_1;

	// The translation is a function of 'cuboid_2' attributes.
	A2_orig.block<num_features, num_attributes>(num_features, start_index_2)
		= A2_orig.block<num_features, num_attributes>(num_features, start_index_2)
		+ translation_2;


	// NOTE:
	// Since the center point is always the origin in the local coordinates,
	// it is not used as the feature values.
	const unsigned int num_rows = CuboidJointNormalRelations::k_mat_size;
	Eigen::MatrixXd A(num_rows, mat_size);
	A.topRows(2 * num_features - CuboidFeatures::k_corner_index)
		= A1_orig.bottomRows(2 * num_features - CuboidFeatures::k_corner_index);
	A.bottomRows(2 * num_features - CuboidFeatures::k_corner_index)
		= A2_orig.bottomRows(2 * num_features - CuboidFeatures::k_corner_index);

	Eigen::VectorXd b = -relation_12->get_mean();
	assert(b.rows() == CuboidJointNormalRelations::k_mat_size);

	Eigen::MatrixXd C = relation_12->get_inv_cov();
	assert(C.rows() == CuboidJointNormalRelations::k_mat_size);
	assert(C.cols() == CuboidJointNormalRelations::k_mat_size);


	_quadratic_term = A.transpose() * C * A;
	_linear_term = (b.transpose() * C * A).transpose();
	_constant_term = (b.transpose() * C * b);


	CuboidAttributes attributes_1, attributes_2;
	attributes_1.compute_attributes(_cuboid_1->getOBB());
	attributes_2.compute_attributes(_cuboid_2->getOBB());

	Eigen::VectorXd x1 = attributes_1.get_attributes();
	Eigen::VectorXd x2 = attributes_2.get_attributes();
	Eigen::VectorXd x = Eigen::VectorXd::Zero(mat_size);
	x.block<num_attributes, 1>(start_index_1, 0) = x1;
	x.block<num_attributes, 1>(start_index_2, 0) = x2;

	Real potential = 0;
	potential += (x.transpose() * _quadratic_term * x);
	potential += (2 * _linear_term.transpose() * x);
	potential += _constant_term;


#ifdef DEBUG_TEST
	Real same_potential = relation_12->compute_error(_cuboid_1, _cuboid_2, &transformation_1, &transformation_2);
	CHECK_NUMERICAL_ERROR(__FUNCTION__, potential, same_potential);
#endif

	//Eigen::IOFormat csv_format(Eigen::StreamPrecision, 0, ",");
	//std::stringstream filename_sstr;
	//filename_sstr << std::string("quadratic_mat_")
	//	<< _cuboid_index_1 << std::string("_")
	//	<< _cuboid_index_2 << std::string(".csv");

	//std::ofstream csv_file(filename_sstr.str());
	//Eigen::IOFormat csv_format(Eigen::StreamPrecision, 0, ",");
	//csv_file << _quadratic_term.format(csv_format) << std::endl;
	//csv_file.close();

	return potential;
}

Real CuboidJointNormalRelationPredictor::get_pair_conditional_quadratic_form(
	const PAPart *_cuboid_1, const PAPart *_cuboid_2,
	const unsigned int _cuboid_index_1, const unsigned int _cuboid_index_2,
	const LabelIndex _label_index_1, const LabelIndex _label_index_2,
	Eigen::MatrixXd &_quadratic_term, Eigen::VectorXd &_linear_term, Real& _constant_term) const
{
	assert(_cuboid_1); assert(_cuboid_2);
	assert(_label_index_1 < num_labels_);
	assert(_label_index_2 < num_labels_);

	// NOTE:
	// Now considering only different label pairs.
	//assert(_label_index_1 != _label_index_2);

	const unsigned int num_attributes = CuboidAttributes::k_num_attributes;
	const unsigned int num_features = CuboidFeatures::k_num_features;
	const unsigned int mat_size = _quadratic_term.cols();

	assert(_quadratic_term.rows() == mat_size);
	assert(_linear_term.rows() == mat_size);
	assert(_cuboid_index_1 * num_attributes <= mat_size);
	assert(_cuboid_index_2 * num_attributes <= mat_size);

	_quadratic_term.setZero();
	_linear_term.setZero();
	_constant_term = 0;

	const CuboidJointNormalRelations *relation_12 = relations_[_label_index_1][_label_index_2];
	if (!relation_12) return 0.0;


	CuboidFeatures features_1, features_2;
	Eigen::MatrixXd attributes_to_features_map_1, attributes_to_features_map_2;

	features_1.compute_features(_cuboid_1->getOBB(), &attributes_to_features_map_1);
	features_2.compute_features(_cuboid_2->getOBB(), &attributes_to_features_map_2);

	assert(attributes_to_features_map_1.rows() == num_features);
	assert(attributes_to_features_map_2.rows() == num_features);

	assert(attributes_to_features_map_1.cols() == num_attributes);
	assert(attributes_to_features_map_2.cols() == num_attributes);

	CuboidTransformation transformation_1;
	transformation_1.compute_transformation(_cuboid_1->getOBB());
	Eigen::MatrixXd rotation_1;
	Eigen::MatrixXd translation_1;
	transformation_1.get_linear_map_transformation(rotation_1, translation_1);


	// (Ax + b)'C(Ax + b) = x'(A'CA)x + 2*(b'CA)x.
	unsigned int start_index_1 = (_cuboid_index_1 * num_attributes);
	unsigned int start_index_2 = (_cuboid_index_2 * num_attributes);

	Eigen::MatrixXd A1_orig = Eigen::MatrixXd::Zero(2 * num_features, mat_size);


	// The rotation is fixed, and attribute-to-feature map is a function of 'cuboid_1' attributes.
	A1_orig.block<num_features, num_attributes>(0, start_index_1)
		= A1_orig.block<num_features, num_attributes>(0, start_index_1)
		+ rotation_1 * attributes_to_features_map_1;

	// The translation is a function of 'cuboid_1' attributes.
	A1_orig.block<num_features, num_attributes>(0, start_index_1)
		= A1_orig.block<num_features, num_attributes>(0, start_index_1)
		+ translation_1;

	// The rotation is fixed, and attribute-to-feature map is a function of 'cuboid_2' attributes.
	A1_orig.block<num_features, num_attributes>(num_features, start_index_2)
		= A1_orig.block<num_features, num_attributes>(num_features, start_index_2)
		+ rotation_1 * attributes_to_features_map_2;

	// The translation is a function of 'cuboid_1' attributes.
	A1_orig.block<num_features, num_attributes>(num_features, start_index_1)
		= A1_orig.block<num_features, num_attributes>(num_features, start_index_1)
		+ translation_1;

	// NOTE:
	// 'A2_orig' is not computed since here it is assumed that
	// the local coordinates of 'cuboid_2' is unknown.
	// See function 'CuboidJointNormalRelationPredictor::get_pair_quadratic_form()'


	// NOTE:
	// Since the center point is always the origin in the local coordinates,
	// it is not used as the feature values.
	const unsigned int num_rows =
		2 * CuboidFeatures::k_num_features - CuboidFeatures::k_corner_index;
	Eigen::MatrixXd A = A1_orig.bottomRows(num_rows);

	// NOTE:
	// Since the local coordinates of 'cuboid_2' is unknown,
	// Features for only 1 -> 1 and 1 -> 2 are used.
	Eigen::VectorXd b = -relation_12->get_mean().segment(0, num_rows);

	// NOTE:
	// Since the local coordinates of 'cuboid_2' is unknown,
	// Features for only 1 -> 1 and 1 -> 2 are used.
	Eigen::MatrixXd C = relation_12->get_inv_cov().block(0, 0, num_rows, num_rows);

	_quadratic_term = A.transpose() * C * A;
	_linear_term = (b.transpose() * C * A).transpose();
	_constant_term = (b.transpose() * C * b);


	CuboidAttributes attributes_1, attributes_2;
	attributes_1.compute_attributes(_cuboid_1->getOBB());
	attributes_2.compute_attributes(_cuboid_2->getOBB());

	Eigen::VectorXd x1 = attributes_1.get_attributes();
	Eigen::VectorXd x2 = attributes_2.get_attributes();
	Eigen::VectorXd x = Eigen::VectorXd::Zero(mat_size);
	x.block<num_attributes, 1>(start_index_1, 0) = x1;
	x.block<num_attributes, 1>(start_index_2, 0) = x2;

	Real potential = 0;
	potential += (x.transpose() * _quadratic_term * x);
	potential += (2 * _linear_term.transpose() * x);
	potential += _constant_term;


#ifdef DEBUG_TEST
	Real same_potential = relation_12->compute_conditional_error(_cuboid_1, _cuboid_2, &transformation_1);
	CHECK_NUMERICAL_ERROR(__FUNCTION__, potential, same_potential);
#endif

	//Eigen::IOFormat csv_format(Eigen::StreamPrecision, 0, ",");
	//std::stringstream filename_sstr;
	//filename_sstr << std::string("conditional_quadratic_mat_")
	//	<< _cuboid_index_1 << std::string("_")
	//	<< _cuboid_index_2 << std::string(".csv");

	//std::ofstream csv_file(filename_sstr.str());
	//Eigen::IOFormat csv_format(Eigen::StreamPrecision, 0, ",");
	//csv_file << _quadratic_term.format(csv_format) << std::endl;
	//csv_file.close();

	return potential;
}

CuboidCondNormalRelationPredictor::CuboidCondNormalRelationPredictor(
	const std::vector< std::vector<CuboidCondNormalRelations *> > &_relations)
	: CuboidPredictor(_relations.size())
	, relations_(_relations)
{
	for (unsigned int label_index = 0; label_index < num_labels_; ++label_index)
		assert(relations_[label_index].size() == num_labels_);
}

void CuboidCondNormalRelationPredictor::get_missing_label_indices(
	const std::list<LabelIndex> &_given_label_indices,
	std::list<LabelIndex> &_missing_label_indices)const
{
	_missing_label_indices.clear();
	unsigned int num_labels = relations_.size();

	bool *is_missing_label = new bool[num_labels];
	memset(is_missing_label, true, num_labels * sizeof(bool));

	for (std::list<LabelIndex>::const_iterator it = _given_label_indices.begin();
		it != _given_label_indices.end(); ++it)
	{
		LabelIndex label_index = (*it);
		assert(label_index < num_labels);
		is_missing_label[label_index] = false;
	}

	for (LabelIndex label_index_1 = 0; label_index_1 < num_labels; ++label_index_1)
	{
		if (is_missing_label[label_index_1])
		{
			bool is_label_confliced = false;

			for (LabelIndex label_index_2 = 0; label_index_2 < num_labels; ++label_index_2)
			{
				if (!is_missing_label[label_index_2] && !relations_[label_index_1][label_index_2])
				{
					is_label_confliced = true;
					break;
				}
			}

			if (!is_label_confliced)
			{
				_missing_label_indices.push_back(label_index_1);
			}
		}
	}

	delete[] is_missing_label;
}

Real CuboidCondNormalRelationPredictor::get_pair_potential(
	const PAPart *_cuboid_1, const PAPart *_cuboid_2,
	const CuboidAttributes *_attributes_1, const CuboidAttributes *_attributes_2,
	const CuboidTransformation *_transformation_1, const CuboidTransformation *_transformation_2,
	const LabelIndex _label_index_1, const LabelIndex _label_index_2) const
{
	assert(_cuboid_1); assert(_cuboid_2);
	assert(_attributes_1); assert(_attributes_2);
	assert(_transformation_1); assert(_transformation_2);

	assert(_label_index_1 < num_labels_);
	assert(_label_index_2 < num_labels_);

	// NOTE:
	// Now considering only different label pairs.
	//assert(_label_index_1 != _label_index_2);

	Real potential = INF_POTENTIAL;

	const CuboidCondNormalRelations *relation_12 = relations_[_label_index_1][_label_index_2];
	const CuboidCondNormalRelations *relation_21 = relations_[_label_index_2][_label_index_1];

	if (relation_12 && relation_21)
	{
		Real potential_12 = relation_12->compute_error(_cuboid_1->getOBB(), _cuboid_2->getOBB(), _transformation_1, _transformation_2);
		Real potential_21 = relation_21->compute_error(_cuboid_2->getOBB(), _cuboid_1->getOBB(), _transformation_2, _transformation_1);
		potential = potential_12 + potential_21;
	}

	return potential;
}

Real CuboidCondNormalRelationPredictor::get_pair_quadratic_form(
	const PAPart *_cuboid_1, const PAPart *_cuboid_2,
	const unsigned int _cuboid_index_1, const unsigned int _cuboid_index_2,
	const LabelIndex _label_index_1, const LabelIndex _label_index_2,
	Eigen::MatrixXd &_quadratic_term, Eigen::VectorXd &_linear_term, Real& _constant_term) const
{
	assert(_cuboid_1); assert(_cuboid_2);
	assert(_label_index_1 < num_labels_);
	assert(_label_index_2 < num_labels_);

	// NOTE:
	// Now considering only different label pairs.
	//assert(_label_index_1 != _label_index_2);

	const unsigned int num_attributes = CuboidAttributes::k_num_attributes;
	const unsigned int num_features = CuboidFeatures::k_num_features;
	const unsigned int num_global_coord_points = CuboidFeatures::k_num_local_points;
	const unsigned int num_non_global_coord_features = num_features - 3 * num_global_coord_points;
	const unsigned int mat_size = _quadratic_term.cols();

	assert(_quadratic_term.rows() == mat_size);
	assert(_linear_term.rows() == mat_size);
	assert(_cuboid_index_1 * num_attributes <= mat_size);
	assert(_cuboid_index_2 * num_attributes <= mat_size);

	_quadratic_term.setZero();
	_linear_term.setZero();
	_constant_term = 0;


	CuboidFeatures features_1, features_2;
	Eigen::MatrixXd attributes_to_features_map_1, attributes_to_features_map_2;

	features_1.compute_features(_cuboid_1->getOBB(), &attributes_to_features_map_1);
	features_2.compute_features(_cuboid_2->getOBB(), &attributes_to_features_map_2);

	assert(attributes_to_features_map_1.rows() == num_features);
	assert(attributes_to_features_map_2.rows() == num_features);

	assert(attributes_to_features_map_1.cols() == num_attributes);
	assert(attributes_to_features_map_2.cols() == num_attributes);

	CuboidTransformation transformation_1;
	transformation_1.compute_transformation(_cuboid_1->getOBB());
	Eigen::MatrixXd rotation_1;
	Eigen::MatrixXd translation_1;
	transformation_1.get_linear_map_transformation(rotation_1, translation_1);

	CuboidTransformation transformation_2;
	transformation_2.compute_transformation(_cuboid_2->getOBB());


	const CuboidCondNormalRelations *relation_12 = relations_[_label_index_1][_label_index_2];
	if (!relation_12) return 0.0;


	// (Ax + b)'C(Ax + b) = x'(A'CA)x + 2*(b'CA)x.
	Eigen::MatrixXd A = Eigen::MatrixXd::Zero(num_features, mat_size);
	Eigen::VectorXd b = Eigen::VectorXd::Zero(num_features);
	Eigen::MatrixXd C = Eigen::MatrixXd::Zero(num_features, num_features);

	unsigned int start_index_1 = (_cuboid_index_1 * num_attributes);
	unsigned int start_index_2 = (_cuboid_index_2 * num_attributes);

	Eigen::MatrixXd mean_A = Eigen::MatrixXd::Zero(num_features, num_features);
	mean_A.rightCols(num_non_global_coord_features) = relation_12->get_mean_A();

	A.block<num_features, num_attributes>(0, start_index_1)
		= A.block<num_features, num_attributes>(0, start_index_1)
		- mean_A * attributes_to_features_map_1;

	A.block<num_features, num_attributes>(0, start_index_2)
		= A.block<num_features, num_attributes>(0, start_index_2)
		+ rotation_1 * attributes_to_features_map_2;

	A.block<num_features, num_attributes>(0, start_index_1)
		= A.block<num_features, num_attributes>(0, start_index_1)
		+ translation_1;

	b = -relation_12->get_mean_b();

	C = relation_12->get_inv_cov();

	_quadratic_term = A.transpose() * C * A;
	_linear_term = (b.transpose() * C * A).transpose();
	_constant_term = (b.transpose() * C * b);


	CuboidAttributes attributes_1, attributes_2;
	attributes_1.compute_attributes(_cuboid_1->getOBB());
	attributes_2.compute_attributes(_cuboid_2->getOBB());

	Eigen::VectorXd x1 = attributes_1.get_attributes();
	Eigen::VectorXd x2 = attributes_2.get_attributes();
	Eigen::VectorXd x = Eigen::VectorXd::Zero(mat_size);
	x.block<num_attributes, 1>(start_index_1, 0) = x1;
	x.block<num_attributes, 1>(start_index_2, 0) = x2;

	Real potential = 0;
	potential += (x.transpose() * _quadratic_term * x);
	potential += (2 * _linear_term.transpose() * x);
	potential += _constant_term;


#ifdef DEBUG_TEST
	Real same_potential = relation_12->compute_error(_cuboid_1, _cuboid_2, &transformation_1, &transformation_2);
	CHECK_NUMERICAL_ERROR(__FUNCTION__, potential, same_potential);
#endif

	//std::stringstream filename_sstr;
	//filename_sstr << std::string("quadratic_mat_")
	//	<< _cuboid_index_1 << std::string("_")
	//	<< _cuboid_index_2 << std::string(".csv");
	//std::ofstream csv_file(filename_sstr.str());
	//Eigen::IOFormat csv_format(Eigen::StreamPrecision, 0, ",");
	//csv_file << quadratic_form.format(csv_format) << std::endl;
	//csv_file.close();

	return potential;
}
