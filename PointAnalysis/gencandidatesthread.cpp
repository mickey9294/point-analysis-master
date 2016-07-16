#include "gencandidatesthread.h"

GenCandidatesThread::GenCandidatesThread(QObject *parent)
	: QThread(parent), m_num_of_candidates(0)
{
	qRegisterMetaType<PAPart>("PAPart");
	qRegisterMetaType<Part_Candidates>("PartCandidates");
	m_pointcloud = NULL;
}

GenCandidatesThread::GenCandidatesThread(PAPointCloud *pointcloud, std::string model_name, QVector<QMap<int, float>> distribution, QObject *parent)
	: QThread(parent), m_num_of_candidates(0), m_model_name(model_name)
{
	qRegisterMetaType<PAPart>("PAPart");
	qRegisterMetaType<Part_Candidates>("PartCandidates");
	m_pointcloud = pointcloud;
	m_distribution = QVector<QMap<int, float>>(distribution);
}

GenCandidatesThread::GenCandidatesThread(std::string model_name, int num_of_candidates, QObject *parent)
	: QThread(parent), m_num_of_candidates(num_of_candidates), m_model_name(model_name)
{

}

GenCandidatesThread::~GenCandidatesThread()
{
	//if (m_pointcloud != NULL)
	//{
	//	delete(m_pointcloud);
	//	m_pointcloud = NULL;
	//}

	if (isRunning())
		terminate();
}

void GenCandidatesThread::run()
{
	if (m_num_of_candidates == 0)
		generateCandidates();
	else
		loadCandidatesFromFiles();
}

using namespace pcl;
using namespace std;
void GenCandidatesThread::generateCandidates()
{
	int num_of_candidates = 0;

	/* Check whether the candidats of the model have already been genearted */
	string cand_path = "../data/candidates/" + m_model_name + "/0.txt";
	ifstream cand_in(cand_path.c_str());
	if (cand_in.is_open())    /* If the candidates have aready been generated and saved to local files */
	{
		cand_in.close();
		loadCandidatesFromFiles();
	}
	else    /* If the candidates of the model have not ever been estimates, then gnerate them */
	{
		onDebugTextAdded("Generating parts candidates...");
		qDebug() << "Generating parts candidates...";

		const float CLASS_CONFIDENCE = 0.7;
		int numOfClasses = m_distribution[0].size();
		int size = m_pointcloud->size();
		int overall_cand_count = 0;
		QVector<bool> point_assigned(size, false);
		OBBEstimator *obbest = new OBBEstimator();
		Part_Candidates part_candidates;  /* The container holding all the parts candidates, whose first template type means the part label
													  * and the second template type means all the candidates of the particlular part label(class) */

		/* Define the symmetry group, parts contained in the group will be regarded as the same part */
		QList<QVector<int>> symmetry_groups;
		QSet<int> symmetry_set;
		/* Load symmetry groups from file */
		std::ifstream sym_in("../data/symmetry_groups.txt");
		if (sym_in.is_open())
		{
			char buffer[128];
			while (!sym_in.eof())
			{
				sym_in.getline(buffer, 128);
				if (strlen(buffer) > 0)
				{
					QStringList sym_str_list = QString(buffer).split(' ');
					QVector<int> group(sym_str_list.size());
					for (int i = 0; i < sym_str_list.size(); i++)
					{
						int label = sym_str_list[i].toInt();
						group[i] = label;
						symmetry_set.insert(label);
					}
					symmetry_groups.push_back(group);
				}
			}
		}

		/* Calculate the number of classes after merge the symmetry groups */
		for (int i = 0; i < symmetry_groups.size(); i++)
		{
			int group_size = symmetry_groups.at(i).size();
			numOfClasses -= group_size - 1;
		}

		/* Create numOfClasses containers to store points belonging to different part class */
		QMap<int, PointCloud<PointXYZ>::Ptr> parts_clouds;
		QMap<int, QList<int>> vertices_indices;    /* The indices of points in each part point cloud */
		QList<int> keys = m_distribution[0].keys();
		QList<int>::iterator key_it;
		for (key_it = keys.begin(); key_it != keys.end(); ++key_it)
		{
			int part_label = *key_it;
			foreach(QVector<int> group, symmetry_groups)    /* Check if the part is in one of symmetry groups */
			{
				if (group.contains(part_label))    /* If the part is contained in a symmetry group */
				{
					part_label = group[0];    /* set the first part label of the group as the label of this part */
					break;
				}
			}
			if (!parts_clouds.contains(part_label))
			{
				PointCloud<PointXYZ>::Ptr part_cloud(new PointCloud<PointXYZ>);
				parts_clouds.insert(part_label, part_cloud);
				vertices_indices.insert(part_label, QList<int>());
			}
		}

		/* Assign each point to a certain container of parts_clouds */
		onDebugTextAdded("Assign each point to a certain part point cloud.");
		qDebug() << "Assign each point to a certain part point cloud.";
		for (int i = 0; i < size; i++)
		{
			QMap<int, float> distribution = m_distribution[i];
			PAPoint point = m_pointcloud->at(i);
			bool ok = false;

			QVector<float> symmetry_scores(symmetry_groups.size());    /* The classification scores of symmetry groups */
			/* Check whether the point belongs to a symmetry group first */
			for (QList<QVector<int>>::iterator it = symmetry_groups.begin(); it != symmetry_groups.end(); ++it)
			{
				QVector<int> group = *it;
				float score_sum = 0;
				for (QVector<int>::iterator label_it = group.begin(); label_it != group.end(); label_it++)
					score_sum += distribution.value(*label_it);
				if (score_sum > 0.7 || Utils::float_equal(score_sum, 0.7))
				{
					parts_clouds[group[0]]->push_back(PointXYZ(point.x(), point.y(), point.z()));
					vertices_indices[group[0]].push_back(i);
					ok = true;
					break;
				}
			}

			if (!ok)    /* If the point is not likely belonging to the parts in symmetry groups */
			{
				float score = 0;
				for (int j = 0; j < keys.size(); j++)
				{
					int label = keys[j];
					if (!symmetry_set.contains(label) && (distribution.value(label) > 0.7 || Utils::float_equal(distribution.value(label), 0.7)))
					{
						parts_clouds[label]->push_back(PointXYZ(point.x(), point.y(), point.z()));
						vertices_indices[label].push_back(i);
						ok = true;
						break;
					}
				}
			}
		}

		/* Generate part candidates for each part point cloud */
		int cluster_count = 0;    /* The index of connected component, used to mark which point cluster each candidate stands for */
		QVector<OBB *> point_clusters_obbs;    /* The OBBs of the point clusters used to display */
		QMap<int, PointCloud<PointXYZ>::Ptr>::iterator part_cloud_it;
		for (part_cloud_it = parts_clouds.begin(); part_cloud_it != parts_clouds.end(); ++part_cloud_it)
		{
			int label_name = part_cloud_it.key();
			PointCloud<PointXYZ>::Ptr part_cloud = part_cloud_it.value();
			QList<int> part_indices = vertices_indices.value(label_name);

			onDebugTextAdded("Generate candidates for part " + QString::number(label_name));
			qDebug("Generating candidates for part-%d...", label_name);

			Graph graph;
			int nvertices = part_cloud->size();    /* The number of vertices in this graph */

			/* If the point cloud has a part of label i */
			if (nvertices > 3)
			{
				/* Introduce edges if the distance between two points is below 0.2 of the scan radius */
				/* Genarate a kd-tree for searching */
				onDebugTextAdded("Part-" + QString::number(label_name) + ": Add edges to the graph.");
				qDebug("Part-%d: Add edges to the graph.", label_name);

				KdTreeFLANN<PointXYZ> kdtree;
				kdtree.setInputCloud(part_cloud);
				qDebug("Part-%d: Kd-tree is built successfully.", label_name);

				std::vector<int> pointIdxRadiusSearch;
				std::vector<float> pointRadiusSquaredDistance;

				float radius = 0.2 * (m_pointcloud->getRadius() == 0 ? 1.0 : m_pointcloud->getRadius());

				for (int j = 0; j < nvertices; j++)
				{
					//qDebug("Part-%d: Find neighbors of vertex-%d/%d.", label_name, j, nvertices);
					PointXYZ searchPoint = part_cloud->at(j);
					if (kdtree.radiusSearch(searchPoint, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0)
					{
						for (int k = 0; k < pointIdxRadiusSearch.size(); k++)
						{
							int idx = pointIdxRadiusSearch[k];
							if (idx > j)    /* Only add edge between j and vertices behind it */
							{
								boost::add_edge(j, idx, graph);
							}
						}
					}
				}

				/* Obtain connected component of the graph */
				onDebugTextAdded("Part-" + QString::number(label_name) + ": Compute connected component of the graph.");
				qDebug("Part-%d: Compute connected component of the graph.", label_name);

				std::vector<int> components(boost::num_vertices(graph));
				int num_of_components = boost::connected_components(graph, &components[0]);    /* Calculate the connected components */
				QVector<QList<int>> components_indices(num_of_components);    /* The indices of points which belong to each connected component(point cluster) */
				/* Create the container cloud for each connected component of the part graph */
				QVector<pcl::PointCloud<PointXYZ>::Ptr> point_clusters(num_of_components);    /* The point cluster genearted by each connected component */
				for (int j = 0; j < num_of_components; j++)
					point_clusters[j] = PointCloud<PointXYZ>::Ptr(new PointCloud<PointXYZ>);
				/* Fill the vertices(points) to the component container that they belong to */
				for (int j = 0; j < components.size(); j++)
				{
					point_clusters[components[j]]->push_back(part_cloud->at(j));
					components_indices[components[j]].push_back(part_indices[j]);
				}

				/* For each connected component, generate 24 candidate OBBs */
				QVector<PAPart> candidates;

				onDebugTextAdded("Part-" + QString::number(label_name) + ": Generating candidates...");
				qDebug("Part-%d: Generating candidates...", label_name);

				int candidates_count = 0;
				for (int j = 0; j < num_of_components; j++)
				{
					onDebugTextAdded("Part-" + QString::number(label_name) + ": Generate candidate OBBs for connected-component-" + QString::number(j) + ".");
					qDebug("Part-%d: Generate candidate OBBs for connected-component-%d.", label_name, j);

					/* Just compute the connected component with at least 2 vertices in it,
					* because it will cause an error to compute eigen vectors of the OBB which contains only 1 point */
					if (point_clusters[j]->size() >= 3)
					{
						obbest->reset(label_name, point_clusters[j]);
						QVector<OBB *> cand_obbs = obbest->computeOBBCandidates();    /* Compute all 24 candidate OBB of the point cluster */
						OBB * point_cluster_obb = obbest->computeOBB();    /* Compute the OBB of the point cluster to display */
						point_cluster_obb->triangulate();
						point_clusters_obbs.push_back(point_cluster_obb);

						for (int k = 0; k < cand_obbs.size(); k++)
						{
							/* Create PAPart object from OBB */
							onDebugTextAdded("Part-" + QString::number(label_name) + ": Create a part for OBB-" + QString::number(j) + "-" + QString::number(k) + " as a candidate.");
							qDebug("Part-%d: Create a part for OBB-%d-%d as a candidate.", label_name, j, k);
							PAPart candidate(cand_obbs[k]);
							candidate.setClusterNo(cluster_count);  /* Set the cluster number to the index of the current connected component */
							/* Set the indices of points assigned to this part to the PAPart object */
							candidate.setVerticesIndices(components_indices[j]);
							candidates.push_back(candidate);
							/* Save the candidate to local file */
							candidate.saveToFile(m_model_name + "/" + std::to_string(overall_cand_count++));

							num_of_candidates++;
						}

						cluster_count++;    /* Shift to the next valid connect component */
					}
				}
				part_candidates.append(candidates);

				onDebugTextAdded("Part-" + QString::number(label_name) + ":Generating part candidates for part-" + QString::number(label_name) + " done.");
				qDebug("Part-%d: Generating part candidates for part-%d done.", label_name, label_name);
			}
			/* If the point cloud does not has a part of label i */
			else
			{
				onDebugTextAdded("Part-" + QString::number(label_name) + ": The point cloud does not have part-" + QString::number(label_name) + ".");
				qDebug("Part-%d: The point cloud does not have part-%d.", label_name, label_name);
			}
		}

		m_num_of_candidates = overall_cand_count;
		emit genCandidatesDone(num_of_candidates, part_candidates);
		//emit setOBBs(point_clusters_obbs);

		onDebugTextAdded("Parts candidates generating done.");
		qDebug() << "Parts candidates generating done.";
	}
}

void GenCandidatesThread::onDebugTextAdded(QString text)
{
	emit addDebugText(text);
}

void GenCandidatesThread::loadCandidatesFromFiles()
{
	onDebugTextAdded("Load candidates from local files.");
	qDebug() << "Load candidates from local files.";

	Part_Candidates candidates(m_num_of_candidates);
	for (int i = 0; i < m_num_of_candidates; i++)
	{
		string cand_path = "../data/candidates/" + m_model_name + "/" + to_string(i) + ".txt";
		PAPart cand(cand_path);
		candidates[i] = cand;
	}

	emit genCandidatesDone(m_num_of_candidates, candidates);

	onDebugTextAdded("Parts candidates loading done.");
	qDebug() << "Parts candidates loading done.";
}