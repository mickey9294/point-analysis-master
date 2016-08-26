#include "pointsegmentationthread.h"

using namespace std;

PointSegmentationThread::PointSegmentationThread(PAPointCloud *pointcloud, QVector<int> label_names, 
	EnergyFunctions *energy_functions, QObject *parent)
	: QThread(parent)
{
	cout << "PointSegmentationThread is created." << endl;

	m_pointcloud = pointcloud;
	m_energy_functions = energy_functions;
	m_label_names = label_names;
}

PointSegmentationThread::~PointSegmentationThread()
{

}

void PointSegmentationThread::run()
{
	cout << "Start segmenting..." << endl;
	int changed_count = optimize();
	cout << "Segmentation done. The labels of " << changed_count << " points have changed." << endl;
}

int PointSegmentationThread::optimize()
{
	double squared_neighbor_distance = pow(param_sparse_neighbor_distance * m_pointcloud->getRadius(), 2);
	double lambda = -squared_neighbor_distance / log(param_null_cuboid_probability);

	MRFEnergy<TypePotts>* mrf_trw;
	MRFEnergy<TypePotts> *mrf_bp;
	MRFEnergy<TypePotts>::NodeId* nodes_trw;
	MRFEnergy<TypePotts>::NodeId *nodes_bp;
	MRFEnergy<TypePotts>::Options options_trw;
	MRFEnergy<TypePotts>::Options options_bp;
	TypePotts::REAL energy, lowerBound;

	const int labelNum = m_label_names.size();
	const int nodeNum = m_pointcloud->size();

	mrf_trw = new MRFEnergy<TypePotts>(TypePotts::GlobalSize(labelNum));
	nodes_trw = new MRFEnergy<TypePotts>::NodeId[nodeNum];
	mrf_bp = new MRFEnergy<TypePotts>(TypePotts::GlobalSize(labelNum));
	nodes_bp = new MRFEnergy<TypePotts>::NodeId[nodeNum];

	TypePotts::REAL *D = new TypePotts::REAL[labelNum];
	TypePotts::REAL *V = new TypePotts::REAL[labelNum * labelNum];

	/* Set the unary potentisals */
	cout << "Set unary potentials." << endl;
	for (int i = 0; i < nodeNum; i++)
	{
		for (int j = 0; j < labelNum; j++)
		{
			double unary = lambda * m_energy_functions->Epnt_single(i, j) + m_energy_functions->Ep_q(i, j);
			D[j] = unary;
		}

		nodes_trw[i] = mrf_trw->AddNode(TypePotts::LocalSize(), TypePotts::NodeData(D));
		nodes_bp[i] = mrf_bp->AddNode(TypePotts::LocalSize(), TypePotts::NodeData(D));
	}

	/* Set the pairwise potentials */
	cout << "Set pairwise potentials." << endl;
	for (int i = 0; i < nodeNum; i++)
	{
		QVector<Eigen::Triplet<double>> pairwise_potentials = m_energy_functions->Esmooth(i);
		for (QVector<Eigen::Triplet<double>>::iterator potentials_it = pairwise_potentials.begin(); potentials_it != pairwise_potentials.end();
			++potentials_it)
		{
			int j = potentials_it->col();
			double potential = potentials_it->value();
			mrf_trw->AddEdge(nodes_trw[i], nodes_trw[j], TypePotts::EdgeData(potential));
			mrf_bp->AddEdge(nodes_bp[i], nodes_bp[j], TypePotts::EdgeData(potential));
		}
	}

	/* Function below is optional - it may help if, for example, nodes are added in a random order */
	//mrf->SetAutomaticOrdering();

	/* Execute TRW-S algorithm */
	cout << "Do TRW_S algorithm." << endl;
	options_trw.m_iterMax = 100; // maximum number of iterations
	options_trw.m_printIter = 10;
	options_trw.m_printMinIter = 0;
	mrf_trw->ZeroMessages();
	mrf_trw->AddRandomMessages(0, 0.0, 1.0);
	mrf_trw->Minimize_TRW_S(options_trw, lowerBound, energy);
	cout << "Minimization done." << endl;

	/* Execute BP algorithm */
	cout << "Do BP algorithm." << endl;
	TypeGeneral::REAL energy_bp;
	options_bp.m_iterMax = 100;
	options_bp.m_printIter = 10;
	options_bp.m_printMinIter = 0;
	mrf_bp->ZeroMessages();
	mrf_bp->AddRandomMessages(0, 0.0, 1.0);
	mrf_bp->Minimize_BP(options_bp, energy_bp);
	cout << "Minimization done." << endl;

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
	vector<int> output_labels(nodeNum);
	for (int node_index = 0; node_index < nodeNum; node_index++)
		output_labels[node_index] = mrf_optimized->GetSolution(nodes_optimized[node_index]);

	/* Count the number of changed labels */
	int changed_count = 0;
	QVector<int> previous_assignments = m_energy_functions->getPointAssignments();
	assert(previous_assignments.size() == output_labels.size());
	for (int i = 0; i < previous_assignments.size(); i++)
	{
		if (previous_assignments[i] != output_labels[i])
			changed_count++;
	}

	/* update the point assignments of the EnergyFunctions */
	QVector<int> new_point_assignments = QVector<int>::fromStdVector(output_labels);
	m_energy_functions->setPointAssignments(new_point_assignments);

	emit pointSegmentationDone(new_point_assignments);

	delete[] nodes_trw;
	delete[] nodes_bp;
	delete mrf_trw;
	delete mrf_bp;
	return changed_count;
}

void PointSegmentationThread::segment_sample_points()
{
	float param_null_cuboid_probability = 0.1;

	assert(m_parts_structure.m_model);
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
		PAPoint part_point = m_parts_structure->get_point[point_index];
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

	std::vector<int> output_labels(num_nodes);
	for (unsigned int node_index = 0; node_index < num_nodes; ++node_index)
		output_labels[node_index] = mrf_optimized->GetSolution(nodes_optimized[node_index]);

	for (std::list<TypeGeneral::REAL *>::iterator it = energy_term_list.end();
		it != energy_term_list.end(); ++it)
		delete[](*it);
	delete[] nodes_trw;
	delete[] nodes_bp;
	delete mrf_trw;
	delete mrf_bp;
}