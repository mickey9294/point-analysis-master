#include "pointsegmentationthread.h"

using namespace std;

PointSegmentationThread::PointSegmentationThread(PartsStructure *parts_structure, QVector<int> label_names,  QObject *parent)
	: QThread(parent)
{
	cout << "PointSegmentationThread is created." << endl;

	m_parts_structure = parts_structure;
	m_label_names = label_names;
}

PointSegmentationThread::~PointSegmentationThread()
{

}

void PointSegmentationThread::run()
{
	cout << "Start segmenting..." << endl;
	int changed_count = segment_sample_points();
	cout << "Segmentation done. The labels of " << changed_count << " points have changed." << endl;
}

int PointSegmentationThread::segment_sample_points()
{
	float param_null_cuboid_probability = 0.1;

	assert(m_parts_structure->m_model != NULL);
	double squared_neighbor_distance = SPARSE_NEIGHBOR_DISTANCE * SPARSE_NEIGHBOR_DISTANCE * m_parts_structure->m_radius;
	double lambda = -squared_neighbor_distance / std::log(param_null_cuboid_probability);

	int num_parts_points = m_parts_structure->num_of_points();
	const int num_neighbors = std::min(param_num_point_neighbors, num_parts_points);

	std::vector<PAPart *> all_cuboids = m_parts_structure->get_all_parts();
	int num_cuboids = all_cuboids.size();

	std::vector<ANNpointArray> cuboid_ann_points(num_cuboids);
	std::vector<ANNkd_tree*> cuboid_ann_kd_tree(num_cuboids);

	for (unsigned int cuboid_index = 0; cuboid_index < num_cuboids; ++cuboid_index)
	{
		PAPart *cuboid = all_cuboids[cuboid_index];
		OBB * cuboid_obb = cuboid->getOBB();

		unsigned int num_cuboid_surface_points = cuboid->num_cuboid_surface_points();
		Eigen::MatrixXd cuboid_surface_points(3, num_cuboid_surface_points);
		assert(num_cuboid_surface_points > 0);

		for (unsigned int point_index = 0; point_index < num_cuboid_surface_points; ++point_index)
		{
			for (unsigned int i = 0; i < 3; ++i)
				cuboid_surface_points.col(point_index)(i) =
				cuboid_obb->getSample(point_index)[i];
		}

		cuboid_ann_kd_tree[cuboid_index] = ICP::create_kd_tree(cuboid_surface_points,
			cuboid_ann_points[cuboid_index]);
		assert(cuboid_ann_points[cuboid_index]);
		assert(cuboid_ann_kd_tree[cuboid_index]);
	}

	ANNpoint q = annAllocPt(3);
	ANNidxArray nn_idx = new ANNidx[1];
	ANNdistArray dd = new ANNdist[1];

	// Single potential.
	// NOTE: The last column is for the null cuboid.
	Eigen::MatrixXd single_potentials(num_parts_points, num_cuboids + 1);

	for (unsigned int point_index = 0; point_index < num_parts_points; ++point_index)
	{
		PAPoint part_point = m_parts_structure->get_point(point_index);
		for (unsigned int i = 0; i < 3; ++i)
			q[i] = part_point.getPosition()[i];

		for (unsigned int cuboid_index = 0; cuboid_index < num_cuboids; ++cuboid_index)
		{
			PAPart *cuboid = all_cuboids[cuboid_index];
			unsigned int label_index = cuboid->getLabel();

			assert(cuboid_ann_kd_tree[cuboid_index]);
			cuboid_ann_kd_tree[cuboid_index]->annkSearch(q, 1, nn_idx, dd);
			double squared_distance = dd[0];
			assert(squared_distance >= 0);

			double label_probability = part_point.getClassConfidence(label_index);

			//
			if (disable_per_point_classifier_terms)
				label_probability = 1.0;
			//

			double energy = squared_distance - lambda * std::log(label_probability);

			//if (cuboid->is_group_cuboid())
			//	energy = FLAGS_param_max_potential;

			single_potentials(point_index, cuboid_index) = energy;
		}

		// For null cuboid.
		double energy = squared_neighbor_distance - lambda * std::log(param_null_cuboid_probability);
		single_potentials(point_index, num_cuboids) = energy;
	}

	// Deallocate ANN.
	annDeallocPt(q);
	delete[] nn_idx;
	delete[] dd;

	for (unsigned int cuboid_index = 0; cuboid_index < num_cuboids; ++cuboid_index)
	{
		annDeallocPts(cuboid_ann_points[cuboid_index]);
		delete cuboid_ann_kd_tree[cuboid_index];
	}


	// Construct a KD-tree.
	Eigen::MatrixXd parts_points(3, num_parts_points);
	for (SamplePointIndex point_index = 0; point_index < num_parts_points; ++point_index)
	{
		for (unsigned int i = 0; i < 3; ++i)
			parts_points.col(point_index)(i) =
			m_parts_structure->get_point(point_index).getPosition()[i];
	}

	const int dim = 3;
	ANNpointArray sample_ann_points = annAllocPts(num_parts_points, dim);	// allocate data points

	for (SamplePointIndex point_index = 0; point_index < num_parts_points; ++point_index)
	{
		for (int i = 0; i < dim; i++)
			sample_ann_points[point_index][i] = parts_points.col(point_index)[i];
	}

	ANNkd_tree *sample_kd_tree = new ANNkd_tree(sample_ann_points, num_parts_points, dim);
	q = annAllocPt(dim);
	nn_idx = new ANNidx[num_neighbors];
	dd = new ANNdist[num_neighbors];

	// Pair potentials.
	std::vector< Eigen::Triplet<double> > pair_potentials;

	if (!disable_label_smoothness_terms)
	{
		pair_potentials.reserve(num_parts_points * num_neighbors);

		for (unsigned int point_index = 0; point_index < num_parts_points; ++point_index)
		{
			for (unsigned int i = 0; i < 3; ++i)
				q[i] = parts_points.col(point_index)[i];

			int num_searched_neighbors = sample_kd_tree->annkFRSearch(q,
				squared_neighbor_distance, num_neighbors, nn_idx, dd);

			for (int i = 0; i < std::min(num_neighbors, num_searched_neighbors); i++)
			{
				unsigned int n_point_index = (int)nn_idx[i];

				// NOTE: Avoid symmetric pairs.
				if (n_point_index <= point_index)
					continue;

				/* Make sure that the assignments of two point are different */
				if (m_parts_structure->get_point_assignment(n_point_index) == m_parts_structure->get_point_assignment(point_index))
					continue;

				//
				double distance = (std::sqrt(squared_neighbor_distance) - std::sqrt(dd[0]));
				assert(distance >= 0);
				//

				double energy = distance * distance;
				pair_potentials.push_back(Eigen::Triplet<double>(point_index, n_point_index, energy));
			}
		}
	}

	delete[] nn_idx;
	delete[] dd;
	annDeallocPt(q);
	annDeallocPts(sample_ann_points);
	delete sample_kd_tree;
	annClose();

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
	cout << "Execute BP algorithm." << endl;
	mrf_bp->ZeroMessages();
	mrf_bp->AddRandomMessages(0, 0.0, 1.0);
	mrf_bp->Minimize_BP(options_bp, energy_bp);
	cout << "done." << endl;

	/////////////////////// TRW-S algorithm //////////////////////
	cout << "Execute TRW_S algorithm." << endl;
	mrf_trw->ZeroMessages();
	mrf_trw->AddRandomMessages(0, 0.0, 1.0);
	mrf_trw->Minimize_TRW_S(options_trw, lower_bound, energy);
	cout << "done." << endl;

	/* Take the lower energy as the result */
	MRFEnergy<TypePotts>* mrf_optimized = NULL;
	MRFEnergy<TypePotts>::NodeId* nodes_optimized = NULL;
	if (energy < energy_bp)
	{
		cout << "TRW_S algorithm is more optimized. Energy = " << energy << endl;
		mrf_optimized = mrf_trw;
		nodes_optimized = nodes_trw;
	}
	else
	{
		cout << "BP algorithm is more optimized. Energy = " << energy_bp << endl;
		mrf_optimized = mrf_bp;
		nodes_optimized = nodes_bp;
	}

	/* Read the solutions */
	std::vector<int> output_labels(num_nodes);
	for (unsigned int node_index = 0; node_index < num_nodes; ++node_index)
		output_labels[node_index] = mrf_optimized->GetSolution(nodes_optimized[node_index]);

	/* Count the number of changed labels */
	int changed_count = 0;
	assert(m_parts_structure->num_of_points() == output_labels.size());
	for (int i = 0; i < m_parts_structure->num_of_points(); i++)
	{
		if (m_parts_structure->get_point_assignment(i) != output_labels[i])
			changed_count++;
	}

	QVector<int> new_point_assignments = QVector<int>::fromStdVector(output_labels);
	emit pointSegmentationDone(new_point_assignments);

	/* Reassign points to the parts */
	std::vector<PAPart *> all_parts = m_parts_structure->get_all_parts();
	int num_parts = all_parts.size();
	/* Clear previous points assignments in each part */
	for (std::vector<PAPart *>::iterator it = all_parts.begin(); it != all_parts.end(); ++it)
		(*it)->clearVertices();

	/* Reassign */
	for (int point_index = 0; point_index < new_point_assignments.size(); point_index++)
	{
		int point_assignment = new_point_assignments[point_index];
		PAPoint point = m_parts_structure->get_point(point_index);
		point.setLabel(point_assignment);

		if (point_assignment >= 0 && point_assignment != m_parts_structure->m_null_label)
		{
			all_parts[point_assignment]->addVertex(point_index, point.getPosition(), point.getNormal());
			m_parts_structure->m_points_assignments[point_index] = point_assignment;
		}
		else
			m_parts_structure->m_points_assignments[point_index] = m_parts_structure->m_null_label;
	}

	/* Update the samples of the parts and the samples correspondences */
	for (std::vector<PAPart *>::iterator part_it = all_parts.begin(); part_it != all_parts.end(); ++part_it)
	{
		(*part_it)->samplePoints();
		(*part_it)->update_sample_correspondences();
	}

	for (std::list<TypeGeneral::REAL *>::iterator it = energy_term_list.end();
		it != energy_term_list.end(); ++it)
		delete[](*it);
	delete[] nodes_trw;
	delete[] nodes_bp;
	delete mrf_trw;
	delete mrf_bp;

	return changed_count;
}