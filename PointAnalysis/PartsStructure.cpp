#include "PartsStructure.h"


PartsStructure::PartsStructure(const Model *model)
	: m_model(model), m_null_label(-1)
{
	if (m_model != NULL)
	{
		m_radius = model->getRadius();
		m_points_assignments.resize(model->vertexCount());
	}
}

PartsStructure::PartsStructure(const PartsStructure & other)
{
	deep_copy(other);
}

PartsStructure::~PartsStructure()
{
	clear();
}

PartsStructure & PartsStructure::operator=(const PartsStructure &other)
{
	clear();
	deep_copy(other);
	return (*this);
}

int PartsStructure::num_of_labels() const
{
	return m_label_names.size();
}

int PartsStructure::num_of_points() const
{
	return m_model->vertexCount();
}

int PartsStructure::num_of_parts() const
{
	int count = 0;
	for (std::vector<std::vector<PAPart *>>::const_iterator it = m_label_parts.begin();
		it != m_label_parts.end(); ++it)
		count += it->size();

	return count;
}

void PartsStructure::clear()
{
	clear_labels();
}

void PartsStructure::clear_labels()
{
	clear_parts();

	m_label_names.clear();
	m_label_paraphrases.clear();
	m_label_symmetries.clear();

	m_symmetry_group_info.clear();
}

void PartsStructure::clear_parts()
{
	for (std::vector<std::vector<PAPart *>>::iterator it = m_label_parts.begin();
		it != m_label_parts.end(); ++it)
	{
		for (std::vector<PAPart *>::iterator jt = it->begin(); jt != it->end(); ++jt)
			delete(*jt);
	}

	m_label_parts.clear();
	m_label_parts.resize(num_of_labels());

	for (std::vector<CuboidReflectionSymmetryGroup *>::iterator it = m_reflection_symmetry_groups.begin();
		it != m_reflection_symmetry_groups.end(); ++it)
		delete(*it);
	m_reflection_symmetry_groups.clear();

	for (std::vector<CuboidRotationSymmetryGroup *>::iterator it = m_rotation_symmetry_groups.begin();
		it != m_rotation_symmetry_groups.end(); ++it)
		delete(*it);
	m_rotation_symmetry_groups.clear();
}

void PartsStructure::clear_parts_only()
{
	for (std::vector<std::vector<PAPart *>>::iterator it = m_label_parts.begin();
		it != m_label_parts.end(); ++it)
		it->clear();

	m_label_parts.clear();
	m_label_parts.resize(num_of_labels());
}

void PartsStructure::deep_copy(const PartsStructure & other)
{
	this->m_model = other.m_model;
	this->m_pointcloud = other.m_pointcloud;
	this->m_null_label = other.m_null_label;
	this->m_label_names = other.m_label_names;
	this->m_label_paraphrases = other.m_label_paraphrases;
	this->m_label_symmetries = other.m_label_symmetries;
	this->m_symmetry_group_info = other.m_symmetry_group_info;

	assert(other.m_label_parts.size() == other.num_of_labels());
	int num_labels = other.num_of_labels();
	this->m_label_parts.clear();
	this->m_label_parts.resize(num_labels);

	for (LabelIndex label_index = 0; label_index < num_labels; ++label_index)
	{
		this->m_label_parts.reserve(other.m_label_parts[label_index].size());
		for (std::vector<PAPart *>::const_iterator it = other.m_label_parts[label_index].begin();
			it != other.m_label_parts[label_index].end(); ++it)
		{
			PAPart *cuboid = new PAPart(**it);
			this->m_label_parts[label_index].push_back(cuboid);
		}
	}

	int num_reflection_symmetry_groups = other.m_reflection_symmetry_groups.size();
	this->m_reflection_symmetry_groups.clear();
	this->m_reflection_symmetry_groups.reserve(num_reflection_symmetry_groups);

	for (std::vector<CuboidReflectionSymmetryGroup *>::const_iterator it = other.m_reflection_symmetry_groups.begin();
		it != other.m_reflection_symmetry_groups.end(); ++it)
	{
		assert(*it);
		CuboidReflectionSymmetryGroup *symmetry_group = new CuboidReflectionSymmetryGroup(**it);
		this->m_reflection_symmetry_groups.push_back(symmetry_group);
	}

	int num_rotation_symmetry_groups = other.m_rotation_symmetry_groups.size();
	this->m_rotation_symmetry_groups.clear();
	this->m_rotation_symmetry_groups.reserve(num_rotation_symmetry_groups);

	for (std::vector<CuboidRotationSymmetryGroup *>::const_iterator it = other.m_rotation_symmetry_groups.begin();
		it != other.m_rotation_symmetry_groups.end(); ++it)
	{
		assert(*it);
		CuboidRotationSymmetryGroup *symmetry_group = new CuboidRotationSymmetryGroup(**it);
		this->m_rotation_symmetry_groups.push_back(symmetry_group);
	}
}

bool PartsStructure::load_label_symmetries(const char *_filename, bool _verbose)
{
	std::ifstream file(_filename);
	if (!file)
	{
		std::cerr << "Can't open file: \"" << _filename << "\"" << std::endl;
		return false;
	}

	if (_verbose)
		std::cout << "Loading " << _filename << "..." << std::endl;

	m_label_symmetries.clear();

	std::string buffer;
	int new_label = 0;

	while (!file.eof())
	{
		std::getline(file, buffer);
		if (buffer == "") break;

		std::stringstream sstr(buffer);
		std::list<LabelIndex> label_symmetry;

		while (!sstr.eof())
		{
			std::string token;
			std::getline(sstr, token, ' ');

			LabelIndex label_index = getLabelName(token);
			assert(label_index < num_of_labels());
			label_symmetry.push_back(label_index);
		}

		m_label_symmetries.push_back(label_symmetry);
	}

	file.close();


	std::cout << "Done." << std::endl;
	return true;
}

bool PartsStructure::load_labels(const char *_filename, bool _verbose)
{
	std::ifstream file(_filename);
	if (!file)
	{
		std::cerr << "Can't open file: \"" << _filename << "\"" << std::endl;
		return false;
	}

	if (_verbose)
		std::cout << "Loading " << _filename << "..." << std::endl;


	// NOTE:
	// All cuboids are also deleted.
	clear_parts();
	m_label_names.clear();
	m_label_paraphrases.clear();
	m_label_symmetries.clear();
	m_symmetry_group_info.clear();


	std::string buffer;
	int new_label = 0;

	while (!file.eof())
	{
		std::getline(file, buffer);
		if (buffer == "") break;

		std::stringstream sstr(buffer);

		const unsigned int num_tokens = 3;
		std::string tokens[num_tokens];

		for (unsigned int i = 0; i < num_tokens; ++i)
		{
			if (sstr.eof())
			{
				std::cerr << "Error: Wrong file format: \"" << _filename << "\"" << std::endl;
				return false;
			}

			std::getline(sstr, tokens[i], ' ');
		}

		if (tokens[1] != "pnts" || tokens[2] != "1")
		{
			std::cerr << "Error: Wrong file format: \"" << _filename << "\"" << std::endl;
			return false;
		}

		// NOTE:
		// In this file format, labels are defined by the recorded order.
		//labels_.push_back(new_label);
		//label_names_.push_back(tokens[0]);
		m_label_names.push_back(new_label);
		m_label_paraphrases.push_back(tokens[0]);
		//label_children_.push_back(std::list<LabelIndex>());
		m_label_parts.resize(m_label_names.size());
		++new_label;
	}

	file.close();

	std::cout << "Done." << std::endl;
	return true;
}

bool PartsStructure::load_symmetry_groups(const char *_filename, bool _verbose)
{
	std::ifstream file(_filename);
	if (!file)
	{
		std::cerr << "Can't open file: \"" << _filename << "\"" << std::endl;
		return false;
	}

	if (_verbose)
		std::cout << "Loading " << _filename << "..." << std::endl;


	for (std::vector<CuboidReflectionSymmetryGroup*>::iterator it = m_reflection_symmetry_groups.begin();
		it != m_reflection_symmetry_groups.end(); ++it)
		delete (*it);
	m_reflection_symmetry_groups.clear();

	for (std::vector< CuboidRotationSymmetryGroup* >::iterator it = m_rotation_symmetry_groups.begin();
		it != m_rotation_symmetry_groups.end(); ++it)
		delete (*it);
	m_rotation_symmetry_groups.clear();

	m_symmetry_group_info.clear();


	std::string buffer;
	CuboidSymmetryGroupInfo new_symmetry_group;

	while (!file.eof())
	{
		std::getline(file, buffer);
		if (buffer == "") break;

		std::stringstream sstr(buffer);
		assert(!sstr.eof());
		std::string head_element;
		std::getline(sstr, head_element, ' ');

		std::vector<std::string> tokens;
		for (std::string each; std::getline(sstr, each, ' '); tokens.push_back(each));
		const unsigned int num_tokens = tokens.size();

		if (head_element == "symmetry_group")
		{
			if (tokens.size() != 2)
			{
				std::cerr << "Error: Wrong file format: \"" << _filename << "\"" << std::endl;
				return false;
			}
			else
			{
				CuboidSymmetryGroupType symmetry_type;

				if (tokens[0] == "reflection")
				{
					symmetry_type = ReflectionSymmetryType;
				}
				else if (tokens[0] == "rotation")
				{
					symmetry_type = RotationSymmetryType;
				}
				else
				{
					std::cerr << "Error: Wrong file format: \"" << _filename << "\"" << std::endl;
					return false;
				}

				unsigned int aligned_global_axis_index = atoi(tokens[1].c_str());
				assert(aligned_global_axis_index < 3);

				if (!new_symmetry_group.single_label_indices_.empty()
					|| !new_symmetry_group.pair_label_indices_.empty())
					// Add the current symmetry group.
					m_symmetry_group_info.push_back(new_symmetry_group);

				new_symmetry_group = CuboidSymmetryGroupInfo(symmetry_type, aligned_global_axis_index);
			}
		}
		else if (head_element == "single_label_indices")
		{
			for (unsigned int i = 0; i < num_tokens; ++i)
			{
				LabelIndex label_index = atoi(tokens[i].c_str());
				assert(label_index < m_label_names.size());
				new_symmetry_group.single_label_indices_.push_back(label_index);
			}
		}
		else if (head_element == "pair_label_indices")
		{
			if ((num_tokens % 2) != 0)
			{
				std::cerr << "Error: Wrong file format: \"" << _filename << "\"" << std::endl;
				return false;
			}
			else
			{
				for (unsigned int i = 0; i < num_tokens / 2; ++i)
				{
					LabelIndex label_index_1 = atoi(tokens[2 * i + 0].c_str());
					LabelIndex label_index_2 = atoi(tokens[2 * i + 1].c_str());
					assert(label_index_1 < m_label_names.size());
					assert(label_index_2 < m_label_names.size());
					new_symmetry_group.pair_label_indices_.push_back(
						std::make_pair(label_index_1, label_index_2));
				}
			}
		}
		else
		{
			std::cerr << "Error: Wrong file format: \"" << _filename << "\"" << std::endl;
			return false;
		}
	}

	if (!new_symmetry_group.single_label_indices_.empty()
		|| !new_symmetry_group.pair_label_indices_.empty())
	{
		m_symmetry_group_info.push_back(new_symmetry_group);
		new_symmetry_group = CuboidSymmetryGroupInfo();
	}

	file.close();

	if (_verbose) std::cout << "Done." << std::endl;
	return true;
}

void PartsStructure::compute_symmetry_groups()
{
	for (std::vector<CuboidReflectionSymmetryGroup *>::iterator it = m_reflection_symmetry_groups.begin();
		it != m_reflection_symmetry_groups.end(); ++it)
		delete(*it);
	m_reflection_symmetry_groups.clear();

	for (std::vector<CuboidRotationSymmetryGroup *>::iterator it = m_rotation_symmetry_groups.begin();
		it != m_rotation_symmetry_groups.end(); ++it)
		delete(*it);
	m_rotation_symmetry_groups.clear();

	const std::vector<PAPart *> cuboids = get_all_parts();

	for (std::vector<CuboidSymmetryGroupInfo>::iterator it = m_symmetry_group_info.begin();
		it != m_symmetry_group_info.end(); ++it)
	{
		if (it->symmetry_type_ == ReflectionSymmetryType)
		{
			CuboidReflectionSymmetryGroup * group = CuboidReflectionSymmetryGroup::constructor(*it, cuboids);
			if (group)
				m_reflection_symmetry_groups.push_back(group);
		}
		else if (it->symmetry_type_ == RotationSymmetryType)
		{
			CuboidRotationSymmetryGroup * group = CuboidRotationSymmetryGroup::constructor(*it, cuboids);
			if (group)
				m_rotation_symmetry_groups.push_back(group);
		}
	}
}

int PartsStructure::getLabelName(std::string label_paraphrase)
{
	assert(m_label_paraphrases.size() == num_of_labels());

	for (int label_index = 0; label_index < num_of_labels(); label_index++)
		if (m_label_paraphrases[label_index].compare(label_paraphrase) == 0)
			return label_index;

	return 0;
}

std::vector<PAPart *> PartsStructure::get_all_parts() const
{
	std::vector<PAPart *> all_parts;

	for (std::vector<std::vector<PAPart *>>::const_iterator it = m_label_parts.begin();
		it != m_label_parts.end(); ++it)
		all_parts.insert(all_parts.end(), it->begin(), it->end());

	return all_parts;
}

PAPart * PartsStructure::get_part(int index)
{
	return m_label_parts[index].front();
}

void PartsStructure::add_part(int label, PAPart * part)
{
	m_label_parts[label].push_back(part);
}

void PartsStructure::set_model(Model * model)
{
	assert(model);
	m_model = model;
	m_radius = model->getRadius();
	m_points_assignments.resize(model->vertexCount());
	cout << "The size of m_points_assignments of PartsStructure = " << m_points_assignments.size() << endl;
}

void PartsStructure::set_points_assignments(QVector<int> assignments)
{
	assert(m_pointcloud);
	assert(assignments.size() == m_pointcloud->size());

	m_points_assignments.resize(assignments.size());
	for (int i = 0; i < assignments.size(); i++)
	{
		m_points_assignments[i] = assignments[i];

		m_pointcloud->operator[](i).setLabel(assignments[i]);
	}
}

PAPoint & PartsStructure::get_point(int index)
{
	assert(m_pointcloud);
	return m_pointcloud->operator[](index);
}

int PartsStructure::get_point_assignment(int index)
{
	assert(index < m_points_assignments.size());
	return m_points_assignments[index];
}

QVector<OBB *> PartsStructure::get_all_obb_copies()
{
	QVector<OBB *> obbs;

	/*int part_idx = 0;
	for (std::vector<std::vector<PAPart *>>::iterator part_it = m_label_parts.begin();
		part_it != m_label_parts.end(); ++part_it)
	{
		OBB * obb = new OBB(part_it->front()->getOBB());
		obbs[part_idx++] = obb;
	}*/

	for (int i = 0; i < m_label_parts.size(); i++)
	{
		if (m_label_parts[i].size() > 0)
		{
			OBB * obb = new OBB(m_label_parts[i][0]->getOBB());
			obbs.push_back(obb);
		}
	}

	return obbs;
}

void PartsStructure::draw(float scale)
{
	/* Draw parts and their OBBs */
	for (std::vector<std::vector<PAPart *>>::iterator list_it = m_label_parts.begin(); list_it != m_label_parts.end(); ++list_it)
	{
		if (list_it->size() > 0)
		{
			PAPart * part = list_it->operator[](0);
			if (part != NULL)
				part->draw(scale);
		}
	}

	/* Draw symmetry plane and symmetry axis */
	glColor4f(COLORS[9][0], COLORS[9][1], COLORS[9][2], 0.4);
	glBegin(GL_TRIANGLES);

	for (std::vector<CuboidReflectionSymmetryGroup *>::iterator reflection_it = m_reflection_symmetry_groups.begin();
		reflection_it != m_reflection_symmetry_groups.end(); ++reflection_it)
	{
		if (*reflection_it != NULL)
		{
			Eigen::Vector3f n;
			double d;
			(*reflection_it)->get_reflection_plane(n, d);

			std::vector<Eigen::Vector3f> triangles;
			Utils::computePlane(n, d, triangles);

			for (int i = 0; i < 6; i++)
			{
				glNormal3f(n.x(), n.y(), n.z());
				glVertex3f(scale * triangles[i].x(), scale * triangles[i].y(), scale * triangles[i].z());
			}
		}
	}
	glEnd();

	glLineWidth(2.2);
	glBegin(GL_LINES);

	for (std::vector<CuboidRotationSymmetryGroup *>::iterator rotate_it = m_rotation_symmetry_groups.begin();
		rotate_it != m_rotation_symmetry_groups.end(); ++rotate_it)
	{
		if (*rotate_it != NULL)
		{
			std::array<Eigen::Vector3f, 2> corners;
			(*rotate_it)->get_rotation_axis_corners(Eigen::Vector3f::Zero(), scale, corners);
			for (int i = 0; i < 2; i++)
				glVertex3f(corners[i].x(), corners[i].y(), corners[i].z());
		}
	}
	glEnd();
}

void PartsStructure::set_pointcloud(QSharedPointer<PAPointCloud> pointcloud)
{
	assert(pointcloud->size() == num_of_points());
	m_pointcloud = pointcloud;
}

void PartsStructure::output_samples()
{
	for (std::vector<std::vector<PAPart *>>::iterator it = m_label_parts.begin(); it != m_label_parts.end(); ++it)
	{
		if (it->size() > 0)
			it->front()->outputSamples();
	}
}