#include "candidatesgenerator.h"

using namespace std;

CandidatesGenerator::CandidatesGenerator(QObject *parent)
	: QObject(parent)
{
	qRegisterMetaType<PAPart>("PAPart");
	qRegisterMetaType<Part_Candidates>("PartCandidates");
}

CandidatesGenerator::~CandidatesGenerator()
{

}

void CandidatesGenerator::generateCandidates(std::string model_name, QSharedPointer<PAPointCloud> pointcloud,
	const QVector<QMap<int, float>> & distributions, Part_Candidates & part_candidates, QVector<int> & point_cluster_map)
{
	using namespace pcl;

	int num_of_candidates = 0;

	/* Check whether the candidats of the model have already been genearted */
	string cand_path = "../data/candidates/" + model_name + "/0.txt";
	ifstream cand_in(cand_path.c_str());
	if (cand_in.is_open())    /* If the candidates have aready been generated and saved to local files */
	{
		cand_in.close();
		loadCandidatesFromFiles(model_name, pointcloud->size(), part_candidates, point_cluster_map);
	}
	else
	{
		cout << "Generating parts candidates..." << endl;

		int numOfClasses = distributions[0].size();
		int size = pointcloud->size();
		int overall_cand_count = 0;
		QVector<bool> point_assigned(size, false);
		OBBEstimator *obbest = new OBBEstimator();
		
		point_cluster_map.resize(size);
		point_cluster_map.fill(-1);

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

		/* Create numOfClasses containers to store points belonging to different part class */
		QMap<int, PointCloud<PointXYZ>::Ptr> parts_clouds;
		QMap<int, QList<int>> vertices_indices;    /* The indices(in the original point cloud) of points in each part point cloud */
		QList<int> keys = distributions[0].keys();
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
		cout << "Assign each point to a certain part point cloud." << endl;
		for (int i = 0; i < size; i++)
		{
			QMap<int, float> distribution = distributions[i];
			PAPoint point = pointcloud->at(i);
			bool ok = false;

			QVector<float> symmetry_scores(symmetry_groups.size());    /* The classification scores of symmetry groups */
			/* Check whether the point belongs to a symmetry group first */
			for (QList<QVector<int>>::iterator it = symmetry_groups.begin(); it != symmetry_groups.end(); ++it)
			{
				QVector<int> group = *it;
				float score_sum = distribution[group[0]];
				/*for (QVector<int>::iterator label_it = group.begin(); label_it != group.end(); label_it++)
					score_sum += distribution.value(*label_it);*/
				if (score_sum > MIN_POINT_CONFIDENCE)
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
					if (!symmetry_set.contains(label) && (distribution.value(label) > MIN_POINT_CONFIDENCE || Utils::float_equal(distribution.value(label), MIN_POINT_CONFIDENCE)))
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
		int num_of_components;    /* The number of point clusters */
		QVector<OBB *> point_clusters_obbs;    /* The OBBs of the point clusters used to display */
		QMap<int, PointCloud<PointXYZ>::Ptr>::iterator part_cloud_it;
		for (part_cloud_it = parts_clouds.begin(); part_cloud_it != parts_clouds.end(); ++part_cloud_it)
		{
			int label_name = part_cloud_it.key();
			PointCloud<PointXYZ>::Ptr part_cloud = *part_cloud_it;
			QList<int> part_indices = vertices_indices[label_name];

			cout << "Generating candidates for part-" << label_name << "..." << endl;

			Graph graph;
			int nvertices = part_cloud->size();    /* The number of vertices in this graph */

			/* If the point cloud has a part of label i */
			if (nvertices > 3)
			{
				/* Introduce edges if the distance between two points is below 0.2 of the scan radius */
				/* Genarate a kd-tree for searching */
				cout << "Part-" << label_name << ": Add edges to the graph";

				KdTreeFLANN<PointXYZ> kdtree;
				kdtree.setInputCloud(part_cloud);
				qDebug("Part-%d: Kd-tree is built successfully.", label_name);

				std::vector<int> pointIdxRadiusSearch;
				std::vector<float> pointRadiusSquaredDistance;

				float radius = 0.2 * ((std::fabs(pointcloud->getRadius()) < 1e-6 || pointcloud->getRadius() > 1.0) ? 1.0 : pointcloud->getRadius());

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
				cout << "Part-" << label_name << ": Compute connected component of the graph." << endl;

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
				QVector<PAPart *> candidates;

				cout << "Part-" << label_name << ": Generating candidates..." << endl;

				int candidates_count = 0;
				for (int j = 0; j < num_of_components; j++)
				{
					cout << "Part-" << label_name << ": Generate candidate OBBs for connected-component-" << j << "." << endl;

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
							cout << "Part-" << label_name << ": Create a part for OBB-" << cluster_count << "-" << num_of_candidates << " as a candidate." << endl;

							PAPart *candidate = new PAPart(cand_obbs[k]);
							candidate->setClusterNo(cluster_count);  /* Set the cluster number to the index of the current connected component */

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
								PAPoint point = pointcloud->operator[](*point_idx_it);
								/*vertices[index] = point.getPosition();
								normals[index++] = point.getNormal();
								if (overall_cand_count == 0 && (*point_idx_it) == 1914)
								cout << "debug: " << vertices[index].transpose() << endl;*/
								candidate->addVertex(*point_idx_it, point.getPosition(), point.getNormal());
							}
							//candidate.setVertices(vertices);
							//candidate.setVerticesNormals(normals);

							//cout << "Run ICP procedure to adjust the OBB" << endl;
							candidate->ICP_adjust_OBB();
							//cout << "done." << endl;

							candidates.push_back(candidate);
							/* Save the candidate to local file */
							candidate->saveToFile(model_name + "/" + std::to_string(overall_cand_count++));

							num_of_candidates++;
						}

						cluster_count++;    /* Shift to the next valid connect component */
					}
				}
				part_candidates.append(candidates);

				cout << "Part-" << label_name << ": Generating part candidates for part-" << label_name << " done." << endl;
			}
			/* If the point cloud does not has a part of label i */
			else
				cout << "Part-" << label_name << ": The point cloud does not have part-" << label_name << "." << endl;
		}

		/* Show the best candidate of each point cluster(the best means to be closest to the global system */
		emit setOBBs(point_clusters_obbs);

		cout << "Parts candidates generating done." << endl;
	}
}

int CandidatesGenerator::loadCandidatesFromFiles(std::string model_name, int npoints,
	Part_Candidates & candidates, QVector<int> & point_cluster_map)
{
	std::cout << "Load candidates from local files." << std::endl;

	int index = 0;
	string cand_path = "../data/candidates/" + model_name + "/" + to_string(index) + ".txt";
	ifstream part_in(cand_path.c_str());
	//Part_Candidates candidates;
	point_cluster_map.resize(npoints);  /* The i-th component represents the point cluster that the i-th point belongs to */
	QVector<OBB *> obbs_to_show;

	while (part_in.is_open())
	{
		part_in.close();

		PAPart *cand = new PAPart(cand_path);
		candidates.push_back(cand);

		/* Set the point cluster number for each point */
		if (index % 24 == 0)  /* Set only once for one point cluster (which generates 24 candidates) */
		{
			for (vector<int>::iterator point_index_it = cand->vertices_begin(); point_index_it != cand->vertices_end();
				++point_index_it)
				point_cluster_map[*point_index_it] = index / 24;

			obbs_to_show.push_back(new OBB(cand->getOBB()));
		}

		index++;

		/* Shift to the next candidate part file */
		cand_path = "../data/candidates/" + model_name + "/" + to_string(index) + ".txt";
		part_in.open(cand_path.c_str());
	}
	/*for (int i = 0; i < m_num_of_candidates; i++)
	{
	string cand_path = "../data/candidates/" + m_model_name + "/" + to_string(i) + ".txt";
	PAPart cand(cand_path);
	candidates[i] = cand;
	}*/

	emit setOBBs(obbs_to_show);

	std::cout << "Parts candidates loading done." << std::endl;
	return candidates.size();
}
