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
	const double param_null_cuboid_probability = 0.1;
	const double param_sparse_neighbor_distance = 0.05;
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

	QVector<int> new_point_assignments = QVector<int>::fromStdVector(output_labels);
	m_energy_functions->setPointAssignments(new_point_assignments);

	emit pointSegmentationDone(new_point_assignments);
	return changed_count;
}