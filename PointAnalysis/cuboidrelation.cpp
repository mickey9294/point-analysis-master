#include "cuboidrelation.h"

const Eigen::Vector3f CuboidAttributes::k_up_direction = Eigen::Vector3f(0, 1.0, 0);

CuboidAttributes::CuboidAttributes()
{
	init();
}

CuboidAttributes::CuboidAttributes(const std::string object_name)
	: m_object_name(object_name)
{

}

CuboidAttributes::CuboidAttributes(const CuboidAttributes &other)
	: m_attributes(other.m_attributes)
{

}

CuboidAttributes::~CuboidAttributes()
{

}

void CuboidAttributes::init()
{
	// Note:
	// Initialized with NaN.
	if (!std::numeric_limits<double>::has_quiet_NaN)
	{
		std::cerr << "Error: " << typeid(double).name() << " does not support quiet NaN." << std::endl;
		do {
			std::cout << '\n' << "Press the Enter key to continue.";
		} while (std::cin.get() != '\n');
	}

	const int num_attributes = static_cast<int>(k_num_attributes);

	m_attributes = Eigen::VectorXd::Zero(num_attributes);
	for (int attribute_index = 0; attribute_index < num_attributes; ++attribute_index)
	{
		m_attributes[attribute_index] = std::numeric_limits<double>::quiet_NaN();
	}
}

void CuboidAttributes::compute_attributes(const OBB *obb)
{
	assert(m_attributes.size() == k_num_attributes);

	for (int i = 0; i < 3; i++)
	{
		for (int corner_index = 0; corner_index < 8; ++corner_index)
			m_attributes[k_corner_index + 3 * corner_index + i] = obb->getVertex(corner_index)[i];
	}
}

void CuboidAttributes::compute_attributes(const PAPart * part)
{
	OBB * obb = part->getOBB();
	assert(obb);
	compute_attributes(obb);
}

void CuboidAttributes::get_attribute_collection_matrix(const std::list<CuboidAttributes *> & _stats, Eigen::MatrixXd &_values)
{
	int num_objects = _stats.size();

	const int num_attributes = k_num_attributes;
	_values = Eigen::MatrixXd(num_objects, num_attributes);
	
	int object_index = 0;
	for (std::list<CuboidAttributes *>::const_iterator stat_it = _stats.begin(); stat_it != _stats.end(); ++stat_it, ++object_index)
	{
		assert(*stat_it);
		for (int attribute_index = 0; attribute_index < num_attributes; ++attribute_index)
			_values(object_index, attribute_index) = (*stat_it)->m_attributes[attribute_index];
	}
}

bool CuboidAttributes::save_attribute_collection(const std::list<CuboidAttributes *> & _stats, const char *_filename)
{
	const int num_attributes = static_cast<int>(k_num_attributes);

	std::ofstream file(_filename);
	if (!file)
	{
		std::cerr << "Can't save file: \"" << _filename << "\"" << std::endl;
		return false;
	}
	std::setprecision(std::numeric_limits<long double>::digits10 + 1);
	std::cout << std::scientific;

	//unsigned int num_objects = _stats.size();

	// Write attribute values.
	unsigned int object_index = 0;
	for (std::list<CuboidAttributes *>::const_iterator stat_it = _stats.begin(); stat_it != _stats.end();
		++stat_it, ++object_index)
	{
		assert(*stat_it);

		for (int attribute_index = 0; attribute_index < num_attributes; ++attribute_index)
		{
			Real value = (*stat_it)->m_attributes[attribute_index];
			if (std::isnan(value))
			{
				// Note:
				// Undefined attributes are recorded as NaN.
				file << "NaN,";
			}
			else
			{
				file << value << ",";
			}
		}
		file << std::endl;
	}

	file.close();
	return true;
}

CuboidFeatures::CuboidFeatures()
{
	init();
}

CuboidFeatures::CuboidFeatures(const std::string object_name)
	: m_object_name(object_name)
{
	init();
}

CuboidFeatures::CuboidFeatures(const CuboidFeatures & other)
	: m_features(other.m_features)
{

}

CuboidFeatures::~CuboidFeatures()
{

}

void CuboidFeatures::init()
{
	// Initialized with NaN.
	if (!std::numeric_limits<double>::has_quiet_NaN)
	{
		std::cerr << "Error: " << typeid(double).name() << " does not support quiet NaN." << std::endl;
		do {
			std::cout << '\n' << "Press the Enter key to continue.";
		} while (std::cin.get() != '\n');
	}

	m_features = Eigen::VectorXd::Zero(CuboidFeatures::k_num_features);
	for (int attribute_index = 0; attribute_index < CuboidFeatures::k_num_features; ++attribute_index)
	{
		m_features[attribute_index] = std::numeric_limits<double>::quiet_NaN();
	}
}

void CuboidFeatures::compute_features(const OBB *obb, Eigen::MatrixXd *_attributes_to_features_map)
{
	const int num_attributes = CuboidAttributes::k_num_attributes;
	assert(m_features.size() == CuboidFeatures::k_num_features);

	/* Linear map */
	Eigen::MatrixXd attributes_to_features_map = Eigen::MatrixXd::Zero(CuboidFeatures::k_num_features, num_attributes);

	Eigen::RowVector3d up_direction_vec(0, 1.0, 0);

	int next_feature_index = 0;

	/* Center point */
	for (int i = 0; i < 3; i++)
	{
		m_features[next_feature_index] = obb->getCentroid()[i];
		for (int corner_index = 0; corner_index < 8; ++corner_index)
		{
			attributes_to_features_map.row(next_feature_index)(CuboidAttributes::k_corner_index + 3 * corner_index + i) =
				1.0 / 8.0;
		}

		++next_feature_index;
	}

	/* Corner points */
	for (int corner_index = 0; corner_index < 8; ++corner_index)
	{
		Eigen::Vector3f corner = obb->getVertex(corner_index);

		for (int i = 0; i < 3; ++i)
		{
			m_features[next_feature_index] = corner[i];
			attributes_to_features_map.row(next_feature_index)(CuboidAttributes::k_corner_index + 3 * corner_index + i) = 1.0;
			++next_feature_index;
		}
	}

	/* Center height */
	m_features[next_feature_index] = CuboidAttributes::k_up_direction.dot(obb->getCentroid());
	for (int i = 0; i < 3; i++)
	{
		//attributes_to_features_map.row(next_feature_index)(
		//	MeshCuboidAttributes::k_center_index + i) = MeshCuboidAttributes::k_up_direction[i];
		for (unsigned int corner_index = 0; corner_index < OBB::k_num_corners; ++corner_index)
		{
			attributes_to_features_map.row(next_feature_index)(
				CuboidAttributes::k_corner_index + 3 * corner_index + i) =
				(1.0 / OBB::k_num_corners) * CuboidAttributes::k_up_direction[i];
		}
	}
	++next_feature_index;

	/* Corner height */
	for (int corner_index = 0; corner_index < 8; corner_index++)
	{
		Eigen::Vector3f corner = obb->getVertex(corner_index);
		m_features[next_feature_index] = CuboidAttributes::k_up_direction.dot(corner);

		for (int i = 0; i < 3; i++)
		{
			attributes_to_features_map.row(next_feature_index)(CuboidAttributes::k_corner_index + 3 * corner_index + i) =
				CuboidAttributes::k_up_direction[i];
		}
		next_feature_index++;
	}
	
	assert(next_feature_index == CuboidFeatures::k_num_features);

#ifdef DEBUG_TEST
	CuboidAttributes attributes;
	attributes.compute_attributes(obb);
	Eigen::VectorXd same_features = attributes_to_features_map * attributes.get_attributes();

	assert(same_features.rows() == CuboidFeatures::k_num_features);
	double error = (same_features - m_features).array().abs().sum();

	Utils::CHECK_NUMERICAL_ERROR(__FUNCTION__, error);
#endif

	/* Optional */
	if (_attributes_to_features_map)
		(*_attributes_to_features_map) = attributes_to_features_map;
}

void CuboidFeatures::compute_features(const PAPart *part, Eigen::MatrixXd *_attributes_to_features_map)
{
	OBB * obb = part->getOBB();
	assert(obb);
	compute_features(obb, _attributes_to_features_map);
}

void CuboidFeatures::get_feature_collection_matrix(const std::list<CuboidFeatures *> & _stats, Eigen::MatrixXd & _values)
{
	int num_objects = _stats.size();

	const int num_features = k_num_features;
	_values = Eigen::MatrixXd(num_objects, num_features);

	int object_index = 0;
	for (std::list<CuboidFeatures *>::const_iterator stat_it = _stats.begin(); stat_it != _stats.end(); ++stat_it, ++object_index)
	{
		assert(*stat_it);
		for (int feature_index = 0; feature_index < num_features; ++feature_index)
			_values(object_index, feature_index) = (*stat_it)->m_features[feature_index];
	}
}

bool CuboidFeatures::load_feature_collection(const char *_filename, std::list<CuboidFeatures *> & _stats)
{
	for (std::list<CuboidFeatures *>::iterator it = _stats.begin(); it != _stats.end(); ++it)
		delete (*it);
	_stats.clear();

	std::ifstream file(_filename);
	if (!file)
	{
		std::cerr << "Can't load file: \"" << _filename << "\"" << std::endl;
		return false;
	}

	std::string buffer;
	std::stringstream strstr;
	std::string token;
	bool succeded = true;

	while (!file.eof())
	{
		std::getline(file, buffer);
		if (buffer == "") break;

		strstr.str(std::string());
		strstr.clear();
		strstr.str(buffer);

		CuboidFeatures *new_features = new CuboidFeatures();
		assert(new_features);

		for (int feature_index = 0; feature_index < CuboidFeatures::k_num_features; ++feature_index)
		{
			std::getline(strstr, token, ',');
			if (strstr.eof())
			{
				std::cerr << "Wrong file format: \"" << _filename << "\"" << std::endl;
				succeded = false;
				break;
			}

			if (token == "NaN")
			{
				// Note:
				// Undefined attributes are recorded as NaN.
				new_features->m_features[feature_index] = std::numeric_limits<Real>::quiet_NaN();
			}
			else
			{
				new_features->m_features[feature_index] = std::stof(token);
			}
		}

		if (!succeded) break;
		_stats.push_back(new_features);
	}

	if (!succeded)
	{
		for (std::list<CuboidFeatures *>::iterator it = _stats.begin(); it != _stats.end(); ++it)
			delete (*it);
		_stats.clear();
		return false;
	}

	return true;
}

bool CuboidFeatures::save_feature_collection(const char *_filename, const std::list<CuboidFeatures *> & _stats)
{
	std::ofstream file(_filename);
	if (!file)
	{
		std::cerr << "Can't save file: \"" << _filename << "\"" << std::endl;
		return false;
	}
	std::setprecision(std::numeric_limits<long double>::digits10 + 1);
	std::cout << std::scientific;

	//unsigned int num_objects = _stats.size();

	// Write attribute values.
	unsigned int object_index = 0;
	for (std::list<CuboidFeatures *>::const_iterator stat_it = _stats.begin(); stat_it != _stats.end();
		++stat_it, ++object_index)
	{
		assert(*stat_it);

		for (int feature_index = 0; feature_index < CuboidFeatures::k_num_features; ++feature_index)
		{
			Real value = (*stat_it)->m_features[feature_index];
			if (std::isnan(value))
			{
				// Note:
				// Undefined attributes are recorded as NaN.
				file << "NaN,";
			}
			else
			{
				file << value << ",";
			}
		}
		file << std::endl;
	}

	file.close();
	return true;
}

CuboidTransformation::CuboidTransformation()
	: m_object_name("")
{
	init();
}

CuboidTransformation::CuboidTransformation(const std::string object_name)
	: m_object_name(object_name)
{
	init();
}

CuboidTransformation::~CuboidTransformation()
{

}

void CuboidTransformation::init()
{
	m_first_translation = Eigen::Vector3d::Zero();
	m_second_rotation = Eigen::Matrix3d::Identity();
}

void CuboidTransformation::compute_transformation(const OBB *obb)
{
	assert(obb);

	Eigen::Matrix3f axes = obb->getAxes();

	for (int axis_index = 0; axis_index < 3; ++axis_index)
	{
		for (int i = 0; i < 3; i++)
		{
			m_second_rotation.row(axis_index)(i) = axes.col(axis_index)[i];
		}
		m_first_translation(axis_index) = -obb->getCentroid()[axis_index];
	}
}

void CuboidTransformation::compute_transformation(const PAPart * part)
{
	OBB * obb = part->getOBB();
	assert(obb);
	compute_transformation(obb);
}

Eigen::VectorXd CuboidTransformation::get_transformed_features(const CuboidFeatures & other_features) const
{
	Eigen::VectorXd transformed_features = other_features.get_features();

	for (int i = 0; i < CuboidFeatures::k_num_local_points; i++)
	{
		Eigen::VectorXd sub_values = transformed_features.block(3 * i, 0, 3, 1);
		sub_values = m_first_translation + sub_values;
		sub_values = m_second_rotation * sub_values;
		transformed_features.block(3 * i, 0, 3, 1) = sub_values;
	}

	return transformed_features;
}

Eigen::VectorXd CuboidTransformation::get_transformed_features(const OBB *other_obb) const
{
	assert(other_obb);
	CuboidFeatures other_features;
	other_features.compute_features(other_obb);

	return get_transformed_features(other_features);
}

Eigen::VectorXd CuboidTransformation::get_inverse_transformed_features(const CuboidFeatures & other_features) const
{
	Eigen::VectorXd inverse_transformed_features = other_features.get_features();

	for (int i = 0; i < CuboidFeatures::k_num_local_points; i++)
	{
		Eigen::VectorXd sub_values = inverse_transformed_features.block(3 * i, 0, 3, 1);
		sub_values = m_second_rotation.inverse() * sub_values;
		sub_values = -m_first_translation + sub_values;
		inverse_transformed_features.block(3 * i, 0, 3, 1) = sub_values;
	}

	return inverse_transformed_features;
}

Eigen::VectorXd CuboidTransformation::get_inverse_transformed_features(const OBB * other_obb) const
{
	assert(other_obb);
	CuboidFeatures other_features;
	other_features.compute_features(other_obb);

	return get_inverse_transformed_features(other_features);
}

void CuboidTransformation::get_transformation(Eigen::Matrix3d &_rotation, Eigen::Vector3d &_translation) const
{
	_rotation = m_second_rotation;
	_translation = m_first_translation;
	_translation = _rotation * _translation;
}

void CuboidTransformation::get_inverse_transformation(
	Eigen::Matrix3d &_rotation, Eigen::Vector3d &_translation) const
{
	_rotation = m_second_rotation.transpose();
	_translation = -m_first_translation;
}

void CuboidTransformation::get_linear_map_transformation(
	Eigen::MatrixXd &_rotation, Eigen::MatrixXd &_translation) const
{
	const unsigned int num_features = CuboidFeatures::k_num_features;
	_rotation = Eigen::MatrixXd::Identity(num_features, num_features);
	_translation = Eigen::MatrixXd::Zero(num_features, CuboidAttributes::k_num_attributes);

	for (unsigned int k = 0; k < CuboidFeatures::k_num_local_points; ++k)
	{
		_rotation.block<3, 3>(3 * k, 3 * k) = m_second_rotation;

		for (unsigned int i = 0; i < 3; ++i)
		{
			//_translation(3 * k + i, CuboidAttributes::k_center_index + i) = -1;
			for (unsigned int corner_index = 0; corner_index < 8; ++corner_index)
			{
				_translation(3 * k + i, CuboidAttributes::k_corner_index + 3 * corner_index + i) =
					-1.0 / 8.0;
			}
		}
	}

	_translation = _rotation * _translation;
}

void CuboidTransformation::get_linear_map_inverse_transformation(
	Eigen::MatrixXd &_rotation, Eigen::MatrixXd &_translation) const
{
	const unsigned int num_features = CuboidFeatures::k_num_features;
	_rotation = Eigen::MatrixXd::Identity(num_features, num_features);
	_translation = Eigen::MatrixXd::Zero(num_features, CuboidAttributes::k_num_attributes);

	for (unsigned int k = 0; k < CuboidFeatures::k_num_local_points; ++k)
	{
		_rotation.block<3, 3>(3 * k, 3 * k) = m_second_rotation.transpose();

		for (unsigned int i = 0; i < 3; ++i)
		{
			//_translation(3 * k + i, CuboidAttributes::k_center_index + i) = 1;
			for (unsigned int corner_index = 0; corner_index < 8; ++corner_index)
			{
				_translation(3 * k + i, CuboidAttributes::k_corner_index + 3 * corner_index + i) =
					1.0 / 8;
			}
		}
	}
}

bool CuboidTransformation::load_transformation_collection(const char * _filename, std::list<CuboidTransformation *> & _stats)
{
	for (std::list<CuboidTransformation *>::iterator it = _stats.begin(); it != _stats.end(); ++it)
		delete (*it);
	_stats.clear();

	std::ifstream file(_filename);
	if (!file)
	{
		std::cerr << "Can't load file: \"" << _filename << "\"" << std::endl;
		return false;
	}

	std::string buffer;
	std::stringstream strstr;
	std::string token;
	bool succeded = true;

	while (!file.eof())
	{
		std::getline(file, buffer);
		if (buffer == "") break;

		strstr.str(std::string());
		strstr.clear();
		strstr.str(buffer);

		CuboidTransformation *new_transformation = new CuboidTransformation();
		assert(new_transformation);

		for (unsigned int axis_index = 0; axis_index < 3; ++axis_index)
		{
			for (unsigned int i = 0; i < 3; ++i)
			{
				std::getline(strstr, token, ',');
				if (strstr.eof())
				{
					std::cerr << "Wrong file format: \"" << _filename << "\"" << std::endl;
					succeded = false;
					break;
				}

				if (!succeded) break;
				new_transformation->m_second_rotation.col(axis_index)(i) = std::stof(token);
			}
		}

		for (unsigned int axis_index = 0; axis_index < 3; ++axis_index)
		{
			std::getline(strstr, token, ',');
			if (strstr.eof())
			{
				std::cerr << "Wrong file format: \"" << _filename << "\"" << std::endl;
				succeded = false;
				break;
			}

			new_transformation->m_first_translation(axis_index) = std::stof(token);
		}

		if (!succeded) break;
		_stats.push_back(new_transformation);
	}

	if (!succeded)
	{
		for (std::list<CuboidTransformation *>::iterator it = _stats.begin(); it != _stats.end(); ++it)
			delete (*it);
		_stats.clear();
		return false;
	}

	return true;
}

bool CuboidTransformation::save_transformation_collection(const char * _filename, const std::list<CuboidTransformation *> & _stats)
{
	std::ofstream file(_filename);
	if (!file)
	{
		std::cerr << "Can't save file: \"" << _filename << "\"" << std::endl;
		return false;
	}
	std::setprecision(std::numeric_limits<long double>::digits10 + 1);
	std::cout << std::scientific;

	//unsigned int num_objects = _stats.size();

	// Write attribute values.
	unsigned int object_index = 0;
	for (std::list<CuboidTransformation *>::const_iterator stat_it = _stats.begin(); stat_it != _stats.end();
		++stat_it, ++object_index)
	{
		assert(*stat_it);

		for (unsigned int axis_index = 0; axis_index < 3; ++axis_index)
			for (unsigned int i = 0; i < 3; ++i) 
				file << (*stat_it)->m_second_rotation.col(axis_index)(i) << ",";

		for (unsigned int axis_index = 0; axis_index < 3; ++axis_index)
			file << (*stat_it)->m_first_translation(axis_index) << ",";

		file << std::endl;
	}

	file.close();
	return true;
}

CuboidJointNormalRelations::CuboidJointNormalRelations()
{
	mean_ = Eigen::VectorXd::Zero(k_mat_size);
	inv_cov_ = Eigen::MatrixXd::Zero(k_mat_size, k_mat_size);
}

CuboidJointNormalRelations::~CuboidJointNormalRelations()
{

}

void CuboidJointNormalRelations::get_pairwise_cuboid_features(
	const OBB *_cuboid_1, const OBB *_cuboid_2,
	const CuboidTransformation *_transformation_1, const CuboidTransformation *_transformation_2,
	Eigen::VectorXd &_pairwise_features_vec)
{
	assert(_cuboid_1);
	assert(_cuboid_2);

	CuboidFeatures features_1;
	features_1.compute_features(_cuboid_1);

	CuboidFeatures features_2;
	features_2.compute_features(_cuboid_2);

	get_pairwise_cuboid_features(features_1, features_2,
		_transformation_1, _transformation_2, _pairwise_features_vec);
}

void CuboidJointNormalRelations::get_pairwise_cuboid_features(
	const CuboidFeatures &_features_1, const CuboidFeatures &_features_2,
	const CuboidTransformation *_transformation_1, const CuboidTransformation *_transformation_2,
	Eigen::VectorXd &_pairwise_features_vec)
{
	assert(_transformation_1);
	assert(_transformation_2);

	Eigen::VectorXd transformed_features_vec_11 = _transformation_1->get_transformed_features(_features_1);
	assert(std::abs(transformed_features_vec_11[0]) < NUMERIAL_ERROR_THRESHOLD);
	assert(std::abs(transformed_features_vec_11[1]) < NUMERIAL_ERROR_THRESHOLD);
	assert(std::abs(transformed_features_vec_11[2]) < NUMERIAL_ERROR_THRESHOLD);
	assert(transformed_features_vec_11.rows() == CuboidFeatures::k_num_features);

	Eigen::VectorXd transformed_features_vec_12 = _transformation_1->get_transformed_features(_features_2);
	assert(transformed_features_vec_12.rows() == CuboidFeatures::k_num_features);


	Eigen::VectorXd transformed_features_vec_22 = _transformation_2->get_transformed_features(_features_2);
	assert(std::abs(transformed_features_vec_22[0]) < NUMERIAL_ERROR_THRESHOLD);
	assert(std::abs(transformed_features_vec_22[1]) < NUMERIAL_ERROR_THRESHOLD);
	assert(std::abs(transformed_features_vec_22[2]) < NUMERIAL_ERROR_THRESHOLD);
	assert(transformed_features_vec_22.rows() == CuboidFeatures::k_num_features);

	Eigen::VectorXd transformed_features_vec_21 = _transformation_2->get_transformed_features(_features_1);
	assert(transformed_features_vec_21.rows() == CuboidFeatures::k_num_features);


	// NOTE:
	// Since the center point is always the origin in the local coordinates,
	// it is not used as the feature values.
	assert(k_mat_size == 2 * (2 * CuboidFeatures::k_num_features - CuboidFeatures::k_corner_index));
	_pairwise_features_vec.resize(k_mat_size);
	_pairwise_features_vec <<
		transformed_features_vec_11.bottomRows(CuboidFeatures::k_num_features - CuboidFeatures::k_corner_index),
		transformed_features_vec_12,
		transformed_features_vec_22.bottomRows(CuboidFeatures::k_num_features - CuboidFeatures::k_corner_index),
		transformed_features_vec_21;
}

double CuboidJointNormalRelations::compute_error(const OBB *_cuboid_1, const OBB *_cuboid_2,
	const CuboidTransformation *_transformation_1, const CuboidTransformation *_transformation_2) const
{
	Eigen::VectorXd pairwise_cuboid_feature;
	get_pairwise_cuboid_features(_cuboid_1, _cuboid_2, _transformation_1, _transformation_2,
		pairwise_cuboid_feature);

	assert(mean_.rows() == pairwise_cuboid_feature.rows());
	assert(inv_cov_.rows() == pairwise_cuboid_feature.rows());
	assert(inv_cov_.cols() == pairwise_cuboid_feature.rows());

	Eigen::VectorXd diff = pairwise_cuboid_feature - mean_;

	// Mahalanobis norm.
	double error = diff.transpose() * inv_cov_ * diff;
	assert(error >= 0);

	//std::cerr << "Negative error value (error = " << error << ")" << std::endl;
	//Eigen::IOFormat csv_format(Eigen::StreamPrecision, 0, ",");

	//Real inv_cov_det = inv_cov_.determinant();
	//std::cout << "inv_cov_det = " << inv_cov_det << std::endl;

	//Real symmetry_diff = (inv_cov_ - inv_cov_.transpose()).array().abs().sum();
	//std::cerr << "symmetry_diff = " << symmetry_diff << std::endl;

	//std::cout << "Norm(diff) = " << diff.transpose() * diff << std::endl;

	//std::cout << "diff = " << std::endl;
	//std::cout << diff.format(csv_format) << std::endl;

	//do {
	//	std::cout << '\n' << "Press the Enter key to continue.";
	//} while (std::cin.get() != '\n');

	return error;
}

double CuboidJointNormalRelations::compute_conditional_error(const OBB *_cuboid_1, const OBB *_cuboid_2,
	const CuboidTransformation *_transformation_1) const
{
	Eigen::VectorXd conditional_pairwise_cuboid_feature;
	assert(_transformation_1);

	CuboidFeatures features_1;
	features_1.compute_features(_cuboid_1);

	CuboidFeatures features_2;
	features_2.compute_features(_cuboid_2);

	Eigen::VectorXd transformed_features_vec_11 = _transformation_1->get_transformed_features(features_1);
	assert(std::abs(transformed_features_vec_11[0]) < NUMERIAL_ERROR_THRESHOLD);
	assert(std::abs(transformed_features_vec_11[1]) < NUMERIAL_ERROR_THRESHOLD);
	assert(std::abs(transformed_features_vec_11[2]) < NUMERIAL_ERROR_THRESHOLD);
	assert(transformed_features_vec_11.rows() == CuboidFeatures::k_num_features);

	Eigen::VectorXd transformed_features_vec_12 = _transformation_1->get_transformed_features(features_2);
	assert(transformed_features_vec_12.rows() == CuboidFeatures::k_num_features);

	// NOTE:
	// Since the center point is always the origin in the local coordinates,
	// it is not used as the feature values.
	const int num_rows = 2 * CuboidFeatures::k_num_features - CuboidFeatures::k_corner_index;
	conditional_pairwise_cuboid_feature.resize(num_rows);
	conditional_pairwise_cuboid_feature <<
		transformed_features_vec_11.bottomRows(CuboidFeatures::k_num_features - CuboidFeatures::k_corner_index),
		transformed_features_vec_12;

	Eigen::VectorXd conditional_mean = mean_.segment(0, num_rows);
	Eigen::MatrixXd conditional_inv_cov = inv_cov_.block(0, 0, num_rows, num_rows);
	Eigen::VectorXd diff = conditional_pairwise_cuboid_feature - conditional_mean;

	// Mahalanobis norm.
	double error = diff.transpose() * conditional_inv_cov * diff;
	assert(error >= 0);

	return error;
}

CuboidCondNormalRelations::CuboidCondNormalRelations()
{
	assert(CuboidFeatures::k_num_global_feature_values > 0);

	mean_A_ = Eigen::MatrixXd::Zero(CuboidFeatures::k_num_features, CuboidFeatures::k_num_global_feature_values);
	mean_b_ = Eigen::VectorXd::Zero(CuboidFeatures::k_num_features);
	inv_cov_ = Eigen::MatrixXd::Zero(CuboidFeatures::k_num_features, CuboidFeatures::k_num_features);
}

CuboidCondNormalRelations::~CuboidCondNormalRelations()
{

}

void CuboidCondNormalRelations::get_pairwise_cuboid_features(
	const OBB *_cuboid_1, const OBB *_cuboid_2,
	const CuboidTransformation *_transformation_1, const CuboidTransformation *_transformation_2,
	Eigen::VectorXd &_global_features_vec_1, Eigen::VectorXd &_transformed_features_vec_12)
{
	assert(_cuboid_1);
	assert(_cuboid_2);

	CuboidFeatures features_1;
	features_1.compute_features(_cuboid_1);

	CuboidFeatures features_2;
	features_2.compute_features(_cuboid_2);

	get_pairwise_cuboid_features(features_1, features_2,
		_transformation_1, _transformation_2,
		_global_features_vec_1, _transformed_features_vec_12);
}

void CuboidCondNormalRelations::get_pairwise_cuboid_features(
	const CuboidFeatures &_features_1, const CuboidFeatures &_features_2,
	const CuboidTransformation *_transformation_1, const CuboidTransformation *_transformation_2,
	Eigen::VectorXd &_global_features_vec_1, Eigen::VectorXd &_transformed_features_vec_12)
{
	assert(_transformation_1);
	assert(_transformation_2);

	Eigen::VectorXd features_vec_1 = _features_1.get_features();
	assert(features_vec_1.rows() == CuboidFeatures::k_num_features);

	_transformed_features_vec_12 = _transformation_1->get_transformed_features(_features_2);
	assert(_transformed_features_vec_12.rows() == CuboidFeatures::k_num_features);

	_global_features_vec_1 = features_vec_1.bottomRows(CuboidFeatures::k_num_global_feature_values);
}

double CuboidCondNormalRelations::compute_error(const OBB *_cuboid_1, const OBB *_cuboid_2,
	const CuboidTransformation *_transformation_1, const CuboidTransformation *_transformation_2)const
{
	Eigen::VectorXd global_features_vec_1, transformed_features_vec_12;
	get_pairwise_cuboid_features(_cuboid_1, _cuboid_2, _transformation_1, _transformation_2,
		global_features_vec_1, transformed_features_vec_12);

	assert(mean_A_.rows() == transformed_features_vec_12.rows());
	assert(mean_A_.cols() == global_features_vec_1.rows());
	assert(mean_b_.rows() == transformed_features_vec_12.rows());
	assert(inv_cov_.rows() == transformed_features_vec_12.rows());
	assert(inv_cov_.cols() == transformed_features_vec_12.rows());

	const Eigen::VectorXd mean_2 = mean_A_ * global_features_vec_1 + mean_b_;
	Eigen::VectorXd diff = transformed_features_vec_12 - mean_2;

	// Mahalanobis norm.
	double error = diff.transpose() * inv_cov_ * diff;
	assert(error >= 0);

	//std::cerr << "Negative error value (error = " << error << ")" << std::endl;
	//Eigen::IOFormat csv_format(Eigen::StreamPrecision, 0, ",");

	//Real inv_cov_det = inv_cov_.determinant();
	//std::cout << "inv_cov_det = " << inv_cov_det << std::endl;

	//std::cout << "Norm(diff) = " << diff.transpose() * diff << std::endl;

	//std::cout << "diff = " << std::endl;
	//std::cout << diff.format(csv_format) << std::endl;

	//do {
	//	std::cout << '\n' << "Press the Enter key to continue.";
	//} while (std::cin.get() != '\n');

	return error;
}