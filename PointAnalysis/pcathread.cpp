#include "pcathread.h"

PCAThread::PCAThread(QObject *parent) : QThread(parent)
{
	m_phase = PHASE::TESTING;
}

PCAThread::PCAThread(PCModel *pcModel, PHASE phase, QObject *parent)
	: QThread(parent)
{
	m_pcModel = pcModel;
	m_phase = phase;
}

void PCAThread::setPointCloud(PCModel *pcModel)
{
	m_parts.clear();
	m_OBBs.clear();

	m_pcModel = pcModel;
}

PCAThread::~PCAThread()
{
	if (isRunning())
		terminate();
}

using namespace pcl;
void PCAThread::loadParts(PCModel * pcModel)
{
	emit addDebugText("Load parts from the point cloud.");

	int size = pcModel->vertexCount();
	int classSize = pcModel->numOfClasses();
	QVector<int> labels = pcModel->getLabels();
	QList<int> label_names = pcModel->getLabelNames();

	for (int i = 0; i < classSize; i++)
	{
		PointCloud<PointXYZ>::Ptr part(new PointCloud<PointXYZ>);
		int label_name = label_names[i];
		m_parts.insert(label_name, part);
	}
	
	for (int i = 0; i < size; i++)
	{
		GLfloat *point = pcModel->data() + i * 9;
		int label = labels[i];
		PointXYZ pointxyz(point[0], point[1], point[2]);
		m_parts[label]->push_back(pointxyz);
	}
}

void PCAThread::run()
{
	emit addDebugText("Computing oriented bounding boxes...");
	if (m_pcModel->vertexCount() > 0)
	{
		loadParts(m_pcModel);
		QVector<PAPart> parts;
		QVector<int> labels(m_parts.size());

		m_OBBs.resize(m_parts.size());

		QMap<int, PointCloud<PointXYZ>::Ptr>::iterator it;
		int i = 0;
		for (it = m_parts.begin(); it != m_parts.end(); it++)
		{
			int label = it.key();
			labels[i] = i;
			emit addDebugText("Compute OBB of part " + QString::number(label));

			OBBEstimator obbe(label, it.value());
			OBB *obb = obbe.computeOBB();
			obb->triangulate();
			//obb->samplePoints();
			parts.push_back(PAPart(obb));
			m_OBBs[i++] = obb;	
		}

		ICP_procedure();
		
		emit addDebugText("Computing OBB done.");
		emit estimateOBBsCompleted(m_OBBs);
		emit estimatePartsDone(parts);
	}
}

void PCAThread::ICP_procedure()
{
	using namespace std;
	using namespace Eigen;
	const int NUM_OF_SAMPLES = 1000;

	if (m_phase == PHASE::TRAINING)
	{
		/* Sample points on each distinct part */
		QMap<int, vector<Vector3f>> parts_samples;
		sampleOnMesh(parts_samples);

		/* Sample points on each OBB */
		QMap<int, vector<Vector3f>> obbs_samples;
		sampleOnOBBs(obbs_samples);

		/* For each part, do iterative closest points procedure */
		QList<int> label_names = parts_samples.keys();
		for (QList<int>::iterator label_it = label_names.begin(); label_it != label_names.end(); ++label_it)
		{
			int label = *label_it;

			vector<Vector3f> part_samples = parts_samples.value(label);
			vector<Vector3f> obb_samples = obbs_samples.value(label);
			if (part_samples.size() <= 0 || obb_samples.size() <= 0)
				continue;

			/* Form 2 matrices for the samples set */
			MatrixXd part_samples_mat(3, part_samples.size());
			MatrixXd obb_samples_mat(3, obb_samples.size());

			int part_sample_idx = 0;
			for (vector<Vector3f>::iterator part_sample_it = part_samples.begin(); part_sample_it != part_samples.end(); ++part_sample_it)
			{
				Vector3f p = *part_sample_it;
				Vector3d sample(p.x(), p.y(), p.z());
				part_samples_mat.col(part_sample_idx++) = sample;
			}

			int obb_sample_idx = 0;
			for (vector<Vector3f>::iterator obb_sample_it = obb_samples.begin(); obb_sample_it != obb_samples.end(); ++obb_sample_it)
			{
				Vector3f p = *obb_sample_it;
				Vector3d sample(p.x(), p.y(), p.z());
				obb_samples_mat.col(obb_sample_idx++) = sample;
			}

			/* Save the data matrices before ICP */
			string part_out_path_before = "../data/debug/part_sample_before_" + to_string(label) + ".txt";
			string obb_out_path_before = "../data/debug/obb_sample_before_" + to_string(label) + ".txt";
			ofstream part_out_before(part_out_path_before.c_str());
			ofstream obb_out_before(obb_out_path_before.c_str());
			part_out_before << part_samples_mat.transpose() << endl;
			obb_out_before << obb_samples_mat.transpose() << endl;
			part_out_before.close();
			obb_out_before.close();

			/* Do ICP procedure */
			Matrix3d rotation_mat;
			Vector3d translation_vec;
			ICP::run_iterative_closest_points(obb_samples_mat, part_samples_mat, rotation_mat, translation_vec);

			cout << "Rotation mat:" << endl;
			cout << rotation_mat << endl;
			cout << "Translation mat:" << endl;
			cout << translation_vec << endl;

			/* Save the data matrices after ICP */
			string part_out_path = "../data/debug/part_sample_" + to_string(label) + ".txt";
			string obb_out_path = "../data/debug/obb_sample_" + to_string(label) + ".txt";
			ofstream part_out(part_out_path.c_str());
			ofstream obb_out(obb_out_path.c_str());
			part_out << part_samples_mat.transpose() << endl;
			obb_out << obb_samples_mat.transpose() << endl;
			part_out.close();
			obb_out.close();

			/* Adjust the OBB according to the result of ICP */
			if (label >= m_OBBs.size())
				continue;
			OBB * obb = m_OBBs[label];
			Vector3f x_axis = obb->getXAxis();
			Vector4d x_axis_d(x_axis.x(), x_axis.y(), x_axis.z(), 0.0);
			Vector3f y_axis = obb->getYAxis();
			Vector4d y_axis_d(y_axis.x(), y_axis.y(), y_axis.z(), 0.0);
			Vector3f z_axis = obb->getZAxis();
			Vector4d z_axis_d(z_axis.x(), z_axis.y(), z_axis.z(), 0.0);
			Vector3f centroid = obb->getCentroid();
			Vector4d centroid_d(centroid.x(), centroid.y(), centroid.z(), 1.0);

			Matrix4d rotation_aug;
			rotation_aug.block<3, 3>(0, 0) = rotation_mat;
			rotation_aug.block<3, 1>(0, 3) = Vector3d::Zero();
			rotation_aug.block<1, 3>(3, 0) = Vector3d::Zero();
			rotation_aug(3, 3) = 1.0;

			Vector4d trans_aug(translation_vec.x(), translation_vec.y(), translation_vec.z(), 0.0);

			Vector4d x_axis_t = rotation_aug * x_axis_d;
			Vector4d y_axis_t = rotation_aug * y_axis_d;
			Vector4d z_axis_t = rotation_aug * z_axis_d;
			x_axis_t.normalize();
			y_axis_t.normalize();
			z_axis_t.normalize();
			Vector4d centroid_t = rotation_aug * centroid_d + trans_aug;

			Vector3f x_axis_f((float)x_axis_t.x(), (float)x_axis_t.y(), (float)x_axis_t.z());
			Vector3f y_axis_f((float)y_axis_t.x(), (float)y_axis_t.y(), (float)y_axis_t.z());
			Vector3f z_axis_f((float)z_axis_t.x(), (float)z_axis_t.y(), (float)z_axis_t.z());
			Vector3f centroid_f((float)centroid_t.x(), (float)centroid_t.y(), (float)centroid_t.z());

			/*Vector3d x_axis_t = rotation_mat * x_axis_d + translation_vec;
			Vector3d y_axis_t = rotation_mat * y_axis_d + translation_vec;
			Vector3d z_axis_t = rotation_mat * z_axis_d + translation_vec;
			Vector3d centroid_t = rotation_mat * centroid_d + translation_vec;*/

			obb->setXAxis(x_axis_f);
			obb->setYAxis(y_axis_f);
			obb->setZAxis(z_axis_f);
			obb->setCentroid(centroid_f);
		}
	}
}

void PCAThread::sampleOnOBBs(QMap<int, std::vector<Eigen::Vector3f>> &obbs_samples)
{
	using namespace std;
	const int NUM_OF_SAMPLES = 1000;

	for (QVector<OBB *>::iterator obb_it = m_OBBs.begin(); obb_it != m_OBBs.end(); ++obb_it)
	{
		OBB * obb = *obb_it;
		int label = obb->getLabel();

		vector<Utils_sampling::Vec3> verts(obb->vertexCount());
		vector<Utils_sampling::Vec3> nors(obb->vertexCount());
		vector<int> tris(obb->vertexCount());

		int sample_idx = 0;
		int tri_idx = 0;
		for (int i = 0; i < obb->count(); i += 12)
		{
			float * triangle = obb->data() + i;

			Utils_sampling::Vec3 v0(triangle[0], triangle[1], triangle[2]);
			Utils_sampling::Vec3 v1(triangle[3], triangle[4], triangle[5]);
			Utils_sampling::Vec3 v2(triangle[6], triangle[7], triangle[8]);
			Utils_sampling::Vec3 n(triangle[9], triangle[10], triangle[11]);

			verts[sample_idx] = v0;
			tris[sample_idx] = sample_idx;
			nors[sample_idx++] = n;
			verts[sample_idx] = v1;
			tris[sample_idx] = sample_idx;
			nors[sample_idx++] = n;
			verts[sample_idx] = v2;
			tris[sample_idx] = sample_idx;
			nors[sample_idx++] = n;	
		}

		vector<Utils_sampling::Vec3> samples_pos;
		vector<Utils_sampling::Vec3> samples_nors;

		Utils_sampling::poisson_disk(0, NUM_OF_SAMPLES, verts, nors, tris, samples_pos, samples_nors);
		vector<Eigen::Vector3f> samples(samples_pos.size());
		int idx = 0;
		for (vector<Utils_sampling::Vec3>::iterator sample_it = samples_pos.begin(); sample_it != samples_pos.end(); ++sample_it)
		{
			Utils_sampling::Vec3 p = *sample_it;
			samples[idx++] = Eigen::Vector3f(p.x, p.y, p.z);
		}

		obbs_samples.insert(label, samples);
	}
}

void PCAThread::sampleOnMesh(QMap<int, std::vector<Eigen::Vector3f>> &parts_samples)
{
	using namespace std;
	const int NUM_OF_SAMPLES = 1000;

	vector<Utils_sampling::Vec3> points_list;
	vector<Eigen::Vector3i> faces_list;
	QMap<int, QList<int>> label_faces_map;

	loadMesh(points_list, faces_list, label_faces_map);

	/* Sample points on each distinct part */
	for (QMap<int, QList<int>>::iterator part_it = label_faces_map.begin(); part_it != label_faces_map.end(); ++part_it)
	{
		int label = part_it.key();
		QList<int> faces_indices = part_it.value();
		int num_of_faces = faces_indices.size();
		int num_of_tri_vertices = 3 * num_of_faces;

		vector<Utils_sampling::Vec3> points;
		QMap<int, int> origin_sub_map;    /* used to indicate the index in the points vector of a point in original points list */
		vector<Utils_sampling::Vec3> nors;
		vector<int> tris(num_of_tri_vertices);

		/* Extract the sub mesh from the orignial mesh */
		int tri_idx = 0;
		int point_idx = 0;
		for (QList<int>::iterator face_it = faces_indices.begin(); face_it != faces_indices.end(); ++face_it)
		{
			int face_index = *face_it;
			Eigen::Vector3i face = faces_list[face_index];
			int v0_idx = face[0];
			int v1_idx = face[1];
			int v2_idx = face[2];

			if (!origin_sub_map.contains(v0_idx))
			{
				points.push_back(points_list[v0_idx]);
				origin_sub_map.insert(v0_idx, point_idx);
				point_idx++;
			}
			if (!origin_sub_map.contains(v1_idx))
			{
				points.push_back(points_list[v1_idx]);
				origin_sub_map.insert(v1_idx, point_idx);
				point_idx++;
			}
			if (!origin_sub_map.contains(v2_idx))
			{
				points.push_back(points_list[v2_idx]);
				origin_sub_map.insert(v2_idx, point_idx);
				point_idx++;
			}

			tris[tri_idx++] = origin_sub_map.value(v0_idx);
			tris[tri_idx++] = origin_sub_map.value(v1_idx);
			tris[tri_idx++] = origin_sub_map.value(v2_idx);
		}

		nors.resize(points.size());

		vector<Utils_sampling::Vec3> samples_pos;
		vector<Utils_sampling::Vec3> samples_nors;

		Utils_sampling::poisson_disk(0, NUM_OF_SAMPLES, points, nors, tris, samples_pos, samples_nors);

		vector<Eigen::Vector3f> samples(samples_pos.size());
		int sample_idx = 0;
		for (vector<Utils_sampling::Vec3>::iterator sample_it = samples_pos.begin(); sample_it != samples_pos.end(); ++sample_it)
		{
			Utils_sampling::Vec3 p = *sample_it;
			samples[sample_idx++] = Eigen::Vector3f(p.x, p.y, p.z);
		}

		parts_samples.insert(label, samples);
	}
}

void PCAThread::loadMesh(std::vector<Utils_sampling::Vec3> &points_list, std::vector<Eigen::Vector3i> &faces_list, QMap<int, QList<int>> &label_faces_map)
{
	using namespace std;

	string model_file_path = m_pcModel->getInputFilename();
	string seg_file_path = Utils::getSegFilename(QString::fromStdString(model_file_path)).toStdString();

	ifstream mesh_in(model_file_path.c_str());
	ifstream seg_in(seg_file_path.c_str());

	int nvertices;
	int nfaces;

	/* Read the mesh model file */
	if (mesh_in.is_open())
	{
		char buffer[64];

		/* Check the type of mesh model file */
		mesh_in.getline(buffer, 64);
		assert(strcmp(buffer, "OFF") == 0);

		mesh_in.getline(buffer, 64);
		QString num_str(buffer);
		nvertices = num_str.section(' ', 0, 0).toInt();
		nfaces = num_str.section(' ', 1, 1).toInt();
		points_list.resize(nvertices);
		faces_list.resize(nfaces);

		/* Read the vertices */
		for (int i = 0; i < nvertices; i++)
		{
			mesh_in.getline(buffer, 64);

			if (strlen(buffer) > 0)
			{
				QStringList line = QString(buffer).split(' ');

				float x = line[0].toFloat();
				float y = line[1].toFloat();
				float z = line[2].toFloat();

				Utils_sampling::Vec3 point(x, y, z);
				points_list[i] = point;
			}
		}

		/* Read the faces */
		for (int i = 0; i < nfaces; i++)
		{
			mesh_in.getline(buffer, 64);

			if (strlen(buffer) > 0)
			{
				QStringList line = QString(buffer).split(' ');
				int v0_idx = line[1].toInt();
				int v1_idx = line[2].toInt();
				int v2_idx = line[3].toInt();

				Eigen::Vector3i face(v0_idx, v1_idx, v2_idx);
				faces_list[i] = face;
			}
		}
	}

	/* Read the segmentation file */
	if (seg_in.is_open())
	{
		char buffer[16];

		for (int i = 0; i < nfaces; i++)
		{
			seg_in.getline(buffer, 16);
			int label = atoi(buffer);

			if (!label_faces_map.contains(label))
				label_faces_map.insert(label, QList<int>());

			label_faces_map[label].push_back(i);
		}
	}
}