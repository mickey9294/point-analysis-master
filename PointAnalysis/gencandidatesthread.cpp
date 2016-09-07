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
	m_npoints = pointcloud->size();
}

GenCandidatesThread::GenCandidatesThread(std::string model_name, int npoints, int num_of_candidates, QObject *parent)
	: QThread(parent), m_num_of_candidates(num_of_candidates), m_model_name(model_name)
{
	m_npoints = npoints;
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

		int numOfClasses = m_distribution[0].size();
		int size = m_pointcloud->size();
		int overall_cand_count = 0;
		QVector<bool> point_assigned(size, false);
		OBBEstimator *obbest = new OBBEstimator();
		Part_Candidates part_candidates;  /* The container holding all the parts candidates, whose first template type means the part label
													  * and the second template type means all the candidates of the particlular part label(class) */
		QVector<int> point_cluster_map(size);  /* The i-th component represents the point cluster that the i-th point belongs to */
		point_cluster_map.fill(-1);  /* Initialize the map with the null cluster number(namely -1) */

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

		/* Calculate the number of classes after merging the symmetry groups */
		for (int i = 0; i < symmetry_groups.size(); i++)
		{
			int group_size = symmetry_groups.at(i).size();
			numOfClasses -= group_size - 1;
		}

		/* Create numOfClasses containers to store points belonging to different part class */
		QMap<int, PointCloud<PointXYZ>::Ptr> parts_clouds;
		QMap<int, QList<int>> vertices_indices;    /* The indices(in the original point cloud) of points in each part point cloud */
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
			//QMap<int, float> distribution = m_distribution[i];
			//PAPoint point = m_pointcloud->at(i);
			//bool ok = false;

			//QVector<float> symmetry_scores(symmetry_groups.size());    /* The classification scores of symmetry groups */
			///* Check whether the point belongs to a symmetry group first */
			//for (QList<QVector<int>>::iterator it = symmetry_groups.begin(); it != symmetry_groups.end(); ++it)
			//{
			//	QVector<int> group = *it;
			//	float score_sum = 0;
			//	for (QVector<int>::iterator label_it = group.begin(); label_it != group.end(); label_it++)
			//		score_sum += distribution.value(*label_it);
			//	if (score_sum > MIN_POINT_CONFIDENCE)
			//	{
			//		parts_clouds[group[0]]->push_back(PointXYZ(point.x(), point.y(), point.z()));
			//		vertices_indices[group[0]].push_back(i);
			//		ok = true;
			//		break;
			//	}
			//}

			//if (!ok)    /* If the point is not likely belonging to the parts in symmetry groups */
			//{
			//	float score = 0;
			//	for (int j = 0; j < keys.size(); j++)
			//	{
			//		int label = keys[j];
			//		if (!symmetry_set.contains(label) && (distribution.value(label) > MIN_POINT_CONFIDENCE || Utils::float_equal(distribution.value(label), MIN_POINT_CONFIDENCE)))
			//		{
			//			parts_clouds[label]->push_back(PointXYZ(point.x(), point.y(), point.z()));
			//			vertices_indices[label].push_back(i);
			//			ok = true;
			//			break;
			//		}
			//	}
			//}

			PAPoint point = m_pointcloud->at(i);
			QMap<int, float> class_confidences = point.getClassConfidences();

			for (QMap<int, float>::iterator it = class_confidences.begin(); it != class_confidences.end(); ++it)
			{
				if (*it > MIN_POINT_CONFIDENCE || Utils::float_equal(*it, MIN_POINT_CONFIDENCE))
				{
					parts_clouds[it.key()]->push_back(PointXYZ(point.x(), point.y(), point.z()));
					vertices_indices[it.key()].push_back(i);
					break;
				}
			}
		}

		/* Generate part candidates for each part point cloud */
		int cluster_count = 0;    /* The index of connected component, used to mark which point cluster each candidate stands for */
		int num_of_components;    /* The number of point clusters */
		QVector<OBB *> point_clusters_obbs;    /* The OBBs of the point clusters used to display */
		QMap<int, PointCloud<PointXYZ>::Ptr>::iterator part_cloud_it;
		for (part_cloud_it = parts_clouds.begin(); part_cloud_it != parts_clouds.end(); ++part_cloud_it)
		{
			int label_name = part_cloud_it.key();
			PointCloud<PointXYZ>::Ptr part_cloud = *part_cloud_it;
			QList<int> part_indices = vertices_indices[label_name];

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

				float radius = 0.2 * ((std::fabs(m_pointcloud->getRadius()) < 1e-6 || m_pointcloud->getRadius() > 1.0)? 1.0 : m_pointcloud->getRadius());

				for (int j = 0; j < nvertices; j++)
				{
					//qDebug("Part-%d: Find neighbors of vertex-%d/%d.", label_name, j, nvertices);
					PointXYZ searchPoint = part_cloud->at(j);
					//if (kdtree.radiusSearch(searchPoint, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0)
					//{
					//	for (int k = 0; k < pointIdxRadiusSearch.size(); k++)
					//	{
					//		int idx = pointIdxRadiusSearch[k];
					//		if (idx > j)    /* Only add edge between j and vertices behind it */
					//		{
					//			boost::add_edge(j, idx, graph);
					//		}
					//	}
					//}

					for (int k = j + 1; k < nvertices; k++)
					{
						PointXYZ travPoint = part_cloud->at(k);
						float distance = Utils::euclideanDistance(searchPoint, travPoint);
						if (distance < radius || Utils::float_equal(distance, 0))
							boost::add_edge(j, k, graph);
					}
				}

				/* Obtain connected component of the graph */
				onDebugTextAdded("Part-" + QString::number(label_name) + ": Compute connected component of the graph.");
				qDebug("Part-%d: Compute connected component of the graph.", label_name);

				std::vector<int> components(boost::num_vertices(graph));
				num_of_components = boost::connected_components(graph, &components[0]);    /* Calculate the connected components */
				if (label_name == 1)
					cout << "num_of_components = " << num_of_components << endl;

				QVector<QList<int>> components_indices(num_of_components);    /* The indices of points which belong to each connected component(point cluster) */
				/* Create the container cloud for each connected component of the part graph */
				QVector<pcl::PointCloud<PointXYZ>::Ptr> point_clusters(num_of_components);    /* The point cluster genearted by each connected component */
				for (int j = 0; j < num_of_components; j++)
					point_clusters[j] = PointCloud<PointXYZ>::Ptr(new PointCloud<PointXYZ>);
				/* Fill the vertices(points) to the component(point cluster) container that they belong to */
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

					/* Just compute the connected component with at least 10 vertices in it,
					* because it will cause an error to compute eigen vectors of the OBB which contains less than 3 point */
					if (point_clusters[j]->size() >= 10)
					{
						obbest->reset(label_name, point_clusters[j]);
						//cout << "Point_cluster_" << j << " has " << point_clusters[j]->size() << " points." << endl;

						/* Save the poitn cluster */
						//std::string cluster_out_path = "../data/point_clusters/point_cluster" + std::to_string(cluster_count) + ".off";
						//std::ofstream cluster_out(cluster_out_path.c_str());
						//cluster_out << "OFF" << endl;
						//cluster_out << point_clusters[j]->size() << " 0 0" << endl;
						//for (pcl::PointCloud<pcl::PointXYZ>::iterator it = point_clusters[j]->begin(); it != point_clusters[j]->end(); ++it)
						//	cluster_out << it->x << " " << it->y << " " << it->z << endl;
						//cluster_out.close();

						QVector<OBB *> cand_obbs = obbest->computeOBBCandidates();    /* Compute all 24 candidate OBB of the point cluster */
						OBB * point_cluster_obb = new OBB(cand_obbs.first());   /* Compute the OBB of the point cluster to display */
						point_cluster_obb->setColor(QVector3D(COLORS[cluster_count][0], COLORS[cluster_count][1], COLORS[cluster_count][2]));
						point_cluster_obb->triangulate();
						point_clusters_obbs.push_back(point_cluster_obb);

						/* Check the classification probability distribution of the points in the point cluster */
						//if (cluster_count >= 1 && cluster_count <= 3)
						//{
						//	string out_path = "../data/point_clusters/" + to_string(cluster_count) + ".csv";
						//	ofstream dout(out_path.c_str());
						//	dout << ",0,1,2,3,4,5,6,7,8" << endl;

						//	for (int pnt_idx = 0; pnt_idx < point_clusters[j]->size(); pnt_idx++)
						//	{
						//		int origin_idx = part_indices[pnt_idx];   /* The index of the point in origin point cloud */
						//		QMap<int, float> distribution = m_distribution[origin_idx];
						//		int labelNum = distribution.size();
						//		int dcount = 0;
						//		dout << origin_idx << ",";

						//		for (QMap<int, float>::iterator dit = distribution.begin(); dit != distribution.end(); dit++)
						//		{
						//			if (dcount < labelNum - 1)
						//				dout << dit.value() << ",";
						//			else
						//				dout << dit.value() << endl;
						//			dcount++;
						//		}
						//	}

						//	dout.close();
						//}

						/* Set the point cluster number of all the points in the current connected component(point cluster) */
						QList<int> component_indices = components_indices[j];
						for (QList<int>::iterator indice_it = component_indices.begin(); indice_it != component_indices.end(); ++indice_it)
							point_cluster_map[*indice_it] = cluster_count;
						
						for (int k = 0; k < cand_obbs.size(); k++) 
						//for (int k = 0; k < 2; k++)
						{
							/* Create PAPart object from OBB */
							onDebugTextAdded("Part-" + QString::number(label_name) + ": Create a part for OBB-" + QString::number(cluster_count) + "-" + QString::number(num_of_candidates) + " as a candidate.");
							qDebug("Part-%d: Create a part for OBB-%d-%d as a candidate.", label_name, cluster_count, num_of_candidates);

							PAPart candidate(cand_obbs[k]);
							candidate.setClusterNo(cluster_count);  /* Set the cluster number to the index of the current connected component */

							/* Set the indices of points assigned to this part to the PAPart object */
							QList<int> vertices_indices = components_indices[j];
							//candidate.setVerticesIndices(vertices_indices);

							/* Set the vertices list and vertices normals list of the part */
							std::vector<Eigen::Vector3f> vertices, normals;
							//vertices.resize(vertices_indices.size());
							//normals.resize(vertices_indices.size());
							int index = 0;
							for (QList<int>::iterator point_idx_it = vertices_indices.begin(); point_idx_it != vertices_indices.end(); ++point_idx_it)
							{
								PAPoint point = m_pointcloud->operator[](*point_idx_it);
								/*vertices[index] = point.getPosition();
								normals[index++] = point.getNormal();
								if (overall_cand_count == 0 && (*point_idx_it) == 1914)
									cout << "debug: " << vertices[index].transpose() << endl;*/
								candidate.addVertex(*point_idx_it, point.getPosition(), point.getNormal());
							}
							//candidate.setVertices(vertices);
							//candidate.setVerticesNormals(normals);

							//cout << "Run ICP procedure to adjust the OBB" << endl;
							candidate.ICP_adjust_OBB();
							//cout << "done." << endl;

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
		emit genCandidatesDone(num_of_candidates, part_candidates, point_cluster_map);
		/* Show the best candidate of each point cluster(the best means to be closest to the global system */
		emit setOBBs(point_clusters_obbs);

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

	int index = 0;
	string cand_path = "../data/candidates/" + m_model_name + "/" + to_string(index) + ".txt";
	ifstream part_in(cand_path.c_str());
	Part_Candidates candidates;
	QVector<int> point_cluster_map(m_npoints);  /* The i-th component represents the point cluster that the i-th point belongs to */
	QVector<OBB *> obbs_to_show;

	while (part_in.is_open())
	{
		part_in.close();

		PAPart cand(cand_path);
		candidates.push_back(cand);

		/* Set the point cluster number for each point */
		if (index % 24 == 0)  /* Set only once for one point cluster (which generates 24 candidates) */
		{
			for (vector<int>::iterator point_index_it = cand.vertices_begin(); point_index_it != cand.vertices_end();
				++point_index_it)
				point_cluster_map[*point_index_it] = index / 24;

			obbs_to_show.push_back(new OBB(cand.getOBB()));
		}

		index++;

		/* Shift to the next candidate part file */
		cand_path = "../data/candidates/" + m_model_name + "/" + to_string(index) + ".txt";
		part_in.open(cand_path.c_str());
	}
	/*for (int i = 0; i < m_num_of_candidates; i++)
	{
		string cand_path = "../data/candidates/" + m_model_name + "/" + to_string(i) + ".txt";
		PAPart cand(cand_path);
		candidates[i] = cand;
	}*/
	m_num_of_candidates = candidates.size();
	if (candidates.capacity() != m_num_of_candidates)
		candidates.resize(m_num_of_candidates);

	emit genCandidatesDone(m_num_of_candidates, candidates, point_cluster_map);
	emit setOBBs(obbs_to_show);

	onDebugTextAdded("Parts candidates loading done.");
	qDebug() << "Parts candidates loading done.";
}