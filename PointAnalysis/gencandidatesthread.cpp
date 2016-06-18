#include "gencandidatesthread.h"

GenCandidatesThread::GenCandidatesThread(QObject *parent)
	: QThread(parent)
{
	qRegisterMetaType<PAPart>("PAPart");
	qRegisterMetaType<Part_Candidates>("PartCandidates");
	m_pointcloud = NULL;
}

GenCandidatesThread::GenCandidatesThread(PAPointCloud *pointcloud, QVector<QMap<int, float>> distribution, QObject *parent)
	: QThread(parent)
{
	qRegisterMetaType<PAPart>("PAPart");
	qRegisterMetaType<Part_Candidates>("PartCandidates");
	m_pointcloud = pointcloud;
	m_distribution = QVector<QMap<int, float>>(distribution);
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
	generateCandidates();
}

using namespace pcl;
void GenCandidatesThread::generateCandidates()
{
	int num_of_candidates = 0;
	onDebugTextAdded("Generating parts candidates...");
	qDebug() << "Generating parts candidates...";

	const float CLASS_CONFIDENCE = 0.7;
	int numOfClasses = m_distribution[0].size();
	int size = m_pointcloud->size();
	QVector<bool> point_assigned(size, false);
	OBBEstimator *obbest = new OBBEstimator();
	Part_Candidates part_candidates;  /* The container holding all the parts candidates, whose first template type means the part label 
												  * and the second template type means all the candidates of the particlular part label(class) */

	for (int i = 0; i < numOfClasses; i++)    /* For each possible part label */
	{
		int label_name = (m_distribution[0].begin() + i).key();    /* The i-th part label name */

		onDebugTextAdded("Generate candidates for part " + QString::number(label_name));
		qDebug("Generating candidates for part-%d...", label_name);

		Graph graph;
		pcl::PointCloud<pcl::PointXYZ>::Ptr part_cloud(new pcl::PointCloud<pcl::PointXYZ>);
		QList<int> vertices_indices; /* Used to indicate the indices in original pointcloud of a point in part_cloud */
		int nvertices = 0;    /* The number of vertices in this graph */

		/* Add all points that are assigned to label i with high conficence as vertices of a graph */
		/* An important assuption: the orders of vertices in part_cloud and graph are the same */
		onDebugTextAdded("Part-" + QString::number(label_name) + ": Create graph for points.");
		qDebug("Part-%d: Create graph for points.", label_name);
		onDebugTextAdded("Part-" + QString::number(label_name) + ": Add vertices to each part point cloud.");
		qDebug("Part-%d: Add vertices to each part point cloud.", label_name);

		for (int j = 0; j < size; j++)
		{
			if (!point_assigned[j] && m_distribution[j].value(label_name) >= CLASS_CONFIDENCE)
			{
				point_assigned[j] = true;

				PAPoint point = m_pointcloud->at(j);
				part_cloud->push_back(PointXYZ(point.x(), point.y(), point.z()));
				vertices_indices.push_back(j);
				nvertices++;
			}
		}

		/* If the point cloud has a part of label i */
		if (part_cloud->size() >= 3)
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

			///std::ofstream log_out("../data/graph_log.txt");  /* Open a log file recording how the edges are added to the graph */

			for (int j = 0; j < nvertices; j++)
			{
				qDebug("Part-%d: Find neighbors of vertex-%d/%d.", label_name, j, nvertices);
				PointXYZ searchPoint = part_cloud->at(j);
				if (kdtree.radiusSearch(searchPoint, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0)
				{
					for (int k = 0; k < pointIdxRadiusSearch.size(); k++)
					{
						int idx = pointIdxRadiusSearch[k];
						if (idx > j)    /* Only add edge between j and vertices behind it */
						{
							//log_out << std::to_string(j) << " " << std::to_string(idx) << std::endl;
							boost::add_edge(j, idx, graph);
						}
					}
				}
			}

			//log_out.close();

			/* Obtain connected component of the graph */
			onDebugTextAdded("Part-" + QString::number(label_name) + ": Compute connected component of the graph.");
			qDebug("Part-%d: Compute connected component of the graph.", label_name);

			std::vector<int> components(boost::num_vertices(graph));
			int num_of_components = boost::connected_components(graph, &components[0]);    /* Calculate the connected components */
			/* Create the container cloud for each connected component of the part graph */
			QVector<pcl::PointCloud<PointXYZ>::Ptr> component_cand(num_of_components);
			for (int j = 0; j < num_of_components; j++)
				component_cand[j] = PointCloud<PointXYZ>::Ptr(new PointCloud<PointXYZ>);
			/* Fill the vertices(points) to the component container that they belong to */
			for (int j = 0; j < components.size(); j++)
			{
				component_cand[components[j]]->push_back(part_cloud->at(j));
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
				if (component_cand[j]->size() >= 3)
				{
					obbest->reset(label_name, component_cand[j]);
					QVector<OBB *> cand_obbs = obbest->computeOBBCandidates();
					for (int k = 0; k < cand_obbs.size(); k++)
					{
						/* Create PAPart object from OBB */
						onDebugTextAdded("Part-" + QString::number(label_name) + ": Create a part for OBB-" + QString::number(j) + "-" + QString::number(k) + " as a candidate.");
						qDebug("Part-%d: Create a part for OBB-%d-%d as a candidate.", label_name, j, k);
						PAPart candidate(cand_obbs[k]);
						candidates.push_back(candidate);
						num_of_candidates++;
					}
				}
			}
			part_candidates.insert(label_name, candidates);

			onDebugTextAdded("Part-" + QString::number(label_name) + ":Generating part candidates for part-" + QString::number(label_name) + " done.");
			qDebug("Part-%d: Generating part candidates for part-%d done.", label_name, i);
		}
		/* If the point cloud does not has a part of label i */
		else
		{
			onDebugTextAdded("Part-" + QString::number(label_name) + ": The point cloud does not have part-" + QString::number(label_name) + ".");
			qDebug("Part-%d: The point cloud does not have part-%d.", label_name, label_name);
		}
	}
	emit genCandidatesDone(num_of_candidates, part_candidates);

	onDebugTextAdded("Parts candidates generating done.");
	qDebug() << "Parts candidates generating done."; 
}

void GenCandidatesThread::onDebugTextAdded(QString text)
{
	emit addDebugText(text);
}