#ifndef PARTSSTRUCTURE_H
#define PARTSSTRUCTURE_H

#include <QSharedPointer>
#include "papart.h"
#include "model.h"
#include "CuboidSymmetryGroup.h"
#include "PAPointCloud.h"

class PartsStructure
{
public:
	PartsStructure(const Model *model = NULL);
	PartsStructure(const PartsStructure & other);
	~PartsStructure();

	const Model *m_model;
	QSharedPointer<PAPointCloud> m_pointcloud;
	std::vector<int> m_label_names;
	std::vector<std::string> m_label_paraphrases;
	std::vector<std::list<LabelIndex>> m_label_symmetries;
	std::vector<std::vector<PAPart *>> m_label_parts;
	std::vector<int> m_points_assignments;
	int m_null_label;
	float m_radius;

	std::vector<CuboidSymmetryGroupInfo> m_symmetry_group_info;
	std::vector<CuboidReflectionSymmetryGroup *> m_reflection_symmetry_groups;
	std::vector<CuboidRotationSymmetryGroup *> m_rotation_symmetry_groups;

	PartsStructure & operator=(const PartsStructure & other);

	void deep_copy(const PartsStructure & other);

	void clear();
	void clear_parts();
	void clear_labels();
	
	bool load_label_symmetries(const char *filename, bool verbose = true);
	bool load_symmetry_groups(const char *filename, bool verbose = true);
	bool load_labels(const char *filename, bool verbose = true);

	void compute_symmetry_groups();

	int num_of_labels() const;
	int num_of_points() const;
	int num_of_parts() const;
	int getLabelName(const std::string label_paraphrase);

	std::vector<PAPart *> get_all_parts() const;
	QVector<OBB *> get_all_obb_copies();
	PAPart * get_part(int index);
	void add_part(int label, PAPart * part);

	void set_model(Model *model);
	void set_points_assignments(QVector<int> assignments);
	void set_pointcloud(QSharedPointer<PAPointCloud> pointcloud);

	PAPoint & get_point(int index);
	int get_point_assignment(int index);

	void draw(float scale);
};

#endif