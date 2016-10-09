#include "pointsegmentation.h"

PointSegmentation::PointSegmentation(QObject *parent)
	: QObject(parent), m_iteration(0)
{

}

PointSegmentation::~PointSegmentation()
{

}

int PointSegmentation::segmentPoints(PartsStructure & parts_structure, 
	const QVector<int> & label_names, QVector<int> &point_assignments_segmented, bool first_run)
{
	std::ifstream ass_in("../data/point_assignments/assignments.csv");
	if (first_run && ass_in.is_open())
	{
		char buffer[16];

		int idx = 0;
		while (!ass_in.eof())
		{
			ass_in.getline(buffer, 16);
			if (std::strlen(buffer) > 0)
			{
				point_assignments_segmented.push_back(std::atoi(buffer));
				idx++;
			}
		}

		return idx;
	}

	float param_null_cuboid_probability = 0.1;

	assert(parts_structure.m_model != NULL);
	double squared_neighbor_distance = SPARSE_NEIGHBOR_DISTANCE * SPARSE_NEIGHBOR_DISTANCE * parts_structure.m_radius;
	double lambda = -squared_neighbor_distance / std::log(param_null_cuboid_probability);

	int num_parts_points = parts_structure.num_of_points();
	const int num_neighbors = std::min(param_num_point_neighbors, num_parts_points);

	std::vector<PAPart *> all_cuboids = parts_structure.get_all_parts();
	int num_cuboids = all_cuboids.size();

	std::string confi_path = "../data/class_confidences/" + std::to_string(m_iteration++) + ".csv";
	parts_structure.m_pointcloud->outputConfidences(confi_path.c_str());

	//std::vector<ANNpointArray> cuboid_ann_points(num_cuboids);
	//std::vector<ANNkd_tree*> cuboid_ann_kd_tree(num_cuboids);
	std::vector<pcl::KdTreeFLANN<pcl::PointXYZ>> cuboid_flann_kd_tree(num_cuboids);

	for (unsigned int cuboid_index = 0; cuboid_index < num_cuboids; ++cuboid_index)
	{
		PAPart *cuboid = all_cuboids[cuboid_index];
		OBB * cuboid_obb = cuboid->getOBB();

		unsigned int num_cuboid_surface_points = cuboid->num_cuboid_surface_points();
		//Eigen::MatrixXd cuboid_surface_points(3, num_cuboid_surface_points);
		assert(num_cuboid_surface_points > 0);

		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_for_flann(new pcl::PointCloud<pcl::PointXYZ>);
		cloud_for_flann->width = num_cuboid_surface_points;
		cloud_for_flann->height = 1;
		cloud_for_flann->points.resize(cloud_for_flann->width * cloud_for_flann->height);

		for (unsigned int point_index = 0; point_index < num_cuboid_surface_points; ++point_index)
		{
			//for (unsigned int i = 0; i < 3; ++i)
			//	cuboid_surface_points.col(point_index)(i) =
			//	cuboid_obb->getSample(point_index)[i];

			cloud_for_flann->points[point_index].x = cuboid_obb->getSample(point_index).x();
			cloud_for_flann->points[point_index].y = cuboid_obb->getSample(point_index).y();
			cloud_for_flann->points[point_index].z = cuboid_obb->getSample(point_index).z();
		}

		//cuboid_ann_kd_tree[cuboid_index] = ICP::create_kd_tree(cuboid_surface_points,
		//	cuboid_ann_points[cuboid_index]);

		cuboid_flann_kd_tree[cuboid_index].setInputCloud(cloud_for_flann);

		//assert(cuboid_ann_points[cuboid_index]);
		//assert(cuboid_ann_kd_tree[cuboid_index]);
	}

	//ANNpoint q = annAllocPt(3);
	//ANNidxArray nn_idx = new ANNidx[1];
	//ANNdistArray dd = new ANNdist[1];

	// Single potential.
	// NOTE: The last column is for the null cuboid.
	Eigen::MatrixXd single_potentials(num_parts_points, num_cuboids + 1);

	for (unsigned int point_index = 0; point_index < num_parts_points; ++point_index)
	{
		PAPoint part_point = parts_structure.get_point(point_index);
		//for (unsigned int i = 0; i < 3; ++i)
		//	q[i] = part_point.getPosition()[i];

		pcl::PointXYZ searchPoint;
		searchPoint.x = part_point.getPosition().x();
		searchPoint.y = part_point.getPosition().y();
		searchPoint.z = part_point.getPosition().z();

		for (unsigned int cuboid_index = 0; cuboid_index < num_cuboids; ++cuboid_index)
		{
			PAPart *cuboid = all_cuboids[cuboid_index];
			unsigned int label_index = cuboid->getLabel();

			//assert(cuboid_ann_kd_tree[cuboid_index]);
			//cuboid_ann_kd_tree[cuboid_index]->annkSearch(q, 1, nn_idx, dd);
			//double squared_distance = dd[0];
			//assert(squared_distance >= 0);

			std::vector<int> pointIdxNKNSearch(1);
			std::vector<float> pointNKNSquaredDistance(1);
			cuboid_flann_kd_tree[cuboid_index].nearestKSearch(searchPoint, 1, pointIdxNKNSearch, pointNKNSquaredDistance);
			double flann_squared_distance = pointNKNSquaredDistance.front();
			assert(flann_squared_distance >= 0);

			double label_probability = part_point.getClassConfidence(label_index);
			//if (std::fabs(label_probability) < 1.0e-8)
				//label_probability = 1.0e-8;

			//
			if (disable_per_point_classifier_terms)
				label_probability = 1.0;
			//

			//double energy = squared_distance - lambda * std::log(label_probability);
			double energy = 0;
			if (std::fabs(label_probability) < 0.1)
				energy = 1.0e4;
			else
				energy = flann_squared_distance - lambda * std::log(label_probability);

			//if (cuboid->is_group_cuboid())
			//	energy = FLAGS_param_max_potential;

			single_potentials(point_index, cuboid_index) = energy;
		}

		// For null cuboid.
		double energy = squared_neighbor_distance - lambda * std::log(param_null_cuboid_probability);
		single_potentials(point_index, num_cuboids) = 10 * energy;
	}

	// Deallocate ANN.
	//annDeallocPt(q);
	//delete[] nn_idx;
	//delete[] dd;

	//for (unsigned int cuboid_index = 0; cuboid_index < num_cuboids; ++cuboid_index)
	//{
	//	annDeallocPts(cuboid_ann_points[cuboid_index]);
	//	delete cuboid_ann_kd_tree[cuboid_index];
	//}


	// Construct a KD-tree.
	Eigen::MatrixXd parts_points(3, num_parts_points);
	for (SamplePointIndex point_index = 0; point_index < num_parts_points; ++point_index)
	{
		for (unsigned int i = 0; i < 3; ++i)
			parts_points.col(point_index)(i) =
			parts_structure.get_point(point_index).getPosition()[i];
	}

	//const int dim = 3;
	//ANNpointArray sample_ann_points = annAllocPts(num_parts_points, dim);	// allocate data points

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_for_flann(new pcl::PointCloud<pcl::PointXYZ>);
	cloud_for_flann->width = num_parts_points;
	cloud_for_flann->height = 1;
	cloud_for_flann->points.resize(cloud_for_flann->width * cloud_for_flann->height);

	for (SamplePointIndex point_index = 0; point_index < num_parts_points; ++point_index)
	{
		//for (int i = 0; i < dim; i++)
		//sample_ann_points[point_index][i] = parts_points.col(point_index)[i];

		cloud_for_flann->points[point_index].x = parts_points.col(point_index)[0];
		cloud_for_flann->points[point_index].y = parts_points.col(point_index)[1];
		cloud_for_flann->points[point_index].z = parts_points.col(point_index)[2];
	}

	//ANNkd_tree *sample_kd_tree = new ANNkd_tree(sample_ann_points, num_parts_points, dim);
	//q = annAllocPt(dim);
	//nn_idx = new ANNidx[num_neighbors];
	//dd = new ANNdist[num_neighbors];

	pcl::KdTreeFLANN<pcl::PointXYZ> sample_flann_kd_tree;
	sample_flann_kd_tree.setInputCloud(cloud_for_flann);

	// Pair potentials.
	std::vector< Eigen::Triplet<double> > pair_potentials;

	if (!disable_label_smoothness_terms)
	{
		pair_potentials.reserve(num_parts_points * num_neighbors);

		std::vector<int> pointIdxRadiusSearch;
		std::vector<float> pointRadiusSquaredDistantce;

		for (unsigned int point_index = 0; point_index < num_parts_points; ++point_index)
		{
			//for (unsigned int i = 0; i < 3; ++i)
			//	q[i] = parts_points.col(point_index)[i];

			pcl::PointXYZ searchPoint;
			searchPoint.x = parts_points.col(point_index)[0];
			searchPoint.y = parts_points.col(point_index)[1];
			searchPoint.z = parts_points.col(point_index)[2];

			//int num_searched_neighbors = sample_kd_tree->annkFRSearch(q,
			//	squared_neighbor_distance, num_neighbors, nn_idx, dd);

			sample_flann_kd_tree.radiusSearch(searchPoint, std::sqrt(squared_neighbor_distance), pointIdxRadiusSearch, pointRadiusSquaredDistantce, num_neighbors);
			int num_flann_searched_neighbors = pointIdxRadiusSearch.size();
			int max_neighbors = std::min(num_neighbors, num_flann_searched_neighbors);

			for (int i = 0; i < std::min(num_neighbors, num_flann_searched_neighbors); i++)
			{
				//unsigned int n_point_index = (int)nn_idx[i];
				int flann_n_point_index = pointIdxRadiusSearch[i];

				// NOTE: Avoid symmetric pairs.
				if (flann_n_point_index <= point_index)
					continue;

				/* Make sure that the assignments of two point are different */
				if (parts_structure.get_point_assignment(flann_n_point_index) == parts_structure.get_point_assignment(point_index))
					continue;

				//
				//double nd = dd[0];
				//double distance = (std::sqrt(squared_neighbor_distance) - std::sqrt(dd[0]));
				//assert(distance >= 0);
				//

				double flann_distance = (std::sqrt(squared_neighbor_distance) - std::sqrt(pointRadiusSquaredDistantce[i]));

				double energy = flann_distance * flann_distance;
				pair_potentials.push_back(Eigen::Triplet<double>(point_index, flann_n_point_index, energy));
			}
		}
	}

	//delete[] nn_idx;
	//delete[] dd;
	//annDeallocPt(q);
	//annDeallocPts(sample_ann_points);
	//delete sample_kd_tree;
	//annClose();

	// MRF.
	MRFEnergy<TypePotts>* mrf_trw;
	MRFEnergy<TypePotts>* mrf_bp;
	MRFEnergy<TypePotts>::NodeId* nodes_trw;
	MRFEnergy<TypePotts>::NodeId * nodes_bp;
	MRFEnergy<TypePotts>::Options options_trw;
	MRFEnergy<TypePotts>::Options options_bp;
	TypePotts::REAL energy, lower_bound;
	TypePotts::REAL energy_bp;

	const int num_nodes = single_potentials.rows();
	const int num_labels = single_potentials.cols();

	std::list<TypeGeneral::REAL *> energy_term_list;
	mrf_trw = new MRFEnergy<TypePotts>(TypePotts::GlobalSize(num_labels));
	mrf_bp = new MRFEnergy<TypePotts>(TypePotts::GlobalSize(num_labels));
	nodes_trw = new MRFEnergy<TypePotts>::NodeId[num_nodes];
	nodes_bp = new MRFEnergy<TypePotts>::NodeId[num_nodes];

	// Data term.
	for (unsigned int node_index = 0; node_index < num_nodes; ++node_index)
	{
		TypeGeneral::REAL *D = new TypeGeneral::REAL[num_labels];
		energy_term_list.push_back(D);

		for (unsigned int label_index = 0; label_index < num_labels; ++label_index)
			D[label_index] = static_cast<TypeGeneral::REAL>(
			single_potentials(node_index, label_index));
		nodes_trw[node_index] = mrf_trw->AddNode(TypePotts::LocalSize(), TypePotts::NodeData(D));
		nodes_bp[node_index] = mrf_bp->AddNode(TypePotts::LocalSize(), TypePotts::NodeData(D));
	}

	// Smoothness term.
	for (std::vector< Eigen::Triplet<double> >::iterator it = pair_potentials.begin();
		it != pair_potentials.end(); ++it)
	{
		unsigned int node_index_i = (*it).row();
		assert(node_index_i < num_nodes);
		unsigned int node_index_j = (*it).col();
		assert(node_index_j < num_nodes);
		double potential = (*it).value();
		mrf_trw->AddEdge(nodes_trw[node_index_i], nodes_trw[node_index_j], TypePotts::EdgeData(potential));
		mrf_bp->AddEdge(nodes_bp[node_index_i], nodes_bp[node_index_j], TypePotts::EdgeData(potential));
	}


	// Function below is optional - it may help if, for example, nodes are added in a random order
	//mrf->SetAutomaticOrdering();
	options_trw.m_iterMax = 100; // maximum number of iterations
	options_trw.m_printIter = 10;
	options_trw.m_printMinIter = 0;
	options_bp.m_iterMax = 100; // maximum number of iterations
	options_bp.m_printIter = 10;
	options_bp.m_printMinIter = 0;

	//////////////////////// BP algorithm ////////////////////////
	std::cout << "Execute BP algorithm." << std::endl;
	mrf_bp->ZeroMessages();
	mrf_bp->AddRandomMessages(0, 0.0, 1.0);
	mrf_bp->Minimize_BP(options_bp, energy_bp);
	std::cout << "done." << std::endl;

	/////////////////////// TRW-S algorithm //////////////////////
	std::cout << "Execute TRW_S algorithm." << std::endl;
	mrf_trw->ZeroMessages();
	mrf_trw->AddRandomMessages(0, 0.0, 1.0);
	mrf_trw->Minimize_TRW_S(options_trw, lower_bound, energy);
	std::cout << "done." << std::endl;

	/* Take the lower energy as the result */
	MRFEnergy<TypePotts>* mrf_optimized = NULL;
	MRFEnergy<TypePotts>::NodeId* nodes_optimized = NULL;
	if (energy < energy_bp)
	{
		std::cout << "TRW_S algorithm is more optimized. Energy = " << energy << std::endl;
		mrf_optimized = mrf_trw;
		nodes_optimized = nodes_trw;
	}
	else
	{
		std::cout << "BP algorithm is more optimized. Energy = " << energy_bp << std::endl;
		mrf_optimized = mrf_bp;
		nodes_optimized = nodes_bp;
	}

	/* Read the solutions */
	std::vector<int> output_labels(num_nodes);
	for (unsigned int node_index = 0; node_index < num_nodes; ++node_index)
		output_labels[node_index] = mrf_optimized->GetSolution(nodes_optimized[node_index]);

	/* Count the number of changed labels */
	int changed_count = 0;
	assert(parts_structure.num_of_points() == output_labels.size());
	for (int i = 0; i < parts_structure.num_of_points(); i++)
	{
		int assignment_index = output_labels[i];
		int new_assignment;
		if (assignment_index >= 0 && assignment_index < num_cuboids)
		{
			int part_label = all_cuboids[assignment_index]->getLabel();
			assert(part_label != parts_structure.m_null_label);
			new_assignment = part_label;
		}
		else
			new_assignment = parts_structure.m_null_label;

		if (parts_structure.get_point_assignment(i) != new_assignment)
			changed_count++;
	}


	/* Reassign points to the parts */
	std::vector<PAPart *> all_parts = parts_structure.get_all_parts();
	int num_parts = all_parts.size();
	/* Clear previous points assignments in each part */
	for (std::vector<PAPart *>::iterator it = all_parts.begin(); it != all_parts.end(); ++it)
		(*it)->clearVertices();

	/* Reassign */
	for (int point_index = 0; point_index < output_labels.size(); point_index++)
	{
		int point_assignment = output_labels[point_index];

		PAPoint point = parts_structure.get_point(point_index);

		if (point_assignment >= 0 && point_assignment < num_cuboids)
		{
			all_parts[point_assignment]->addVertex(point_index, point.getPosition(), point.getNormal());
			int new_label = all_parts[point_assignment]->getLabel();
			parts_structure.m_points_assignments[point_index] = new_label;
			parts_structure.get_point(point_index).setLabel(new_label);
		}
		else
		{
			parts_structure.m_points_assignments[point_index] = parts_structure.m_null_label;
			parts_structure.get_point(point_index).setLabel(parts_structure.m_null_label);
		}
	}

	/* Update the samples of the parts and the samples correspondences */
	for (std::vector<PAPart *>::iterator part_it = all_parts.begin(); part_it != all_parts.end(); ++part_it)
	{
		(*part_it)->updateOBB();
		(*part_it)->samplePoints();
		//(*part_it)->update_sample_correspondences();
	}

	point_assignments_segmented = QVector<int>::fromStdVector(parts_structure.m_points_assignments);

	/*std::ofstream ass_out("../data/point_assignments/assignments.csv");
	for (int i = 0; i < point_assignments_segmented.size(); i++)
	{
		ass_out << point_assignments_segmented[i] << endl;
	}
	ass_out.close();*/

	//emit pointSegmentationDone(returned_point_assignments);

	for (std::list<TypeGeneral::REAL *>::iterator it = energy_term_list.end();
		it != energy_term_list.end(); ++it)
		delete[](*it);
	delete[] nodes_trw;
	delete[] nodes_bp;
	delete mrf_trw;
	delete mrf_bp;

	return changed_count;
}
