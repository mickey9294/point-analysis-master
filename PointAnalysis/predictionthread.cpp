#include "predictionthread.h"

PredictionThread::PredictionThread(QObject *parent)
	: QThread(parent)
{
	qRegisterMetaType<QMap<int, int>>("PartsPicked");
}

PredictionThread::PredictionThread(EnergyFunctions *energy_functions, Part_Candidates part_candidates, QList<int> label_names, QObject *parent)
	: QThread(parent)
{
	m_energy_functions = energy_functions;
	m_ncandidates = part_candidates.size();
	m_part_candidates = Part_Candidates(part_candidates);
	m_label_names = label_names;

	qRegisterMetaType<QMap<int, int>>("PartsPicked");
}

PredictionThread::~PredictionThread()
{

}

void PredictionThread::run()
{
	predictLabelsAndOrientations();
}

void PredictionThread::predictLabelsAndOrientations()
{
	MRFEnergy<TypeGeneral>* mrf;
	MRFEnergy<TypeGeneral>::NodeId* nodes;
	MRFEnergy<TypeGeneral>::Options options;
	TypeGeneral::REAL energy, lowerBound;

	const int nodeNum = m_ncandidates;   /* the number of nodes */
	const int labelNum = m_label_names.size();   /* the number of labels */

	TypeGeneral::REAL *D = new TypeGeneral::REAL[labelNum];
	TypeGeneral::REAL *V = new TypeGeneral::REAL[labelNum * labelNum];
	
	mrf = new MRFEnergy<TypeGeneral>(TypeGeneral::GlobalSize());
	nodes = new MRFEnergy<TypeGeneral>::NodeId[nodeNum];

	/* Construct energy */
	int num_of_classes = labelNum - 1;    /* '-1' is to remove the null label */

	/* Set the unary potentials of the nodes, where each node is a candidate part */
	int count = 0;
	for (QVector<PAPart>::iterator cand_it = m_part_candidates.begin(); cand_it != m_part_candidates.end(); ++cand_it)
	{
		PAPart cand = *cand_it;
		for (int i = 0; i < labelNum; i++)
			D[i] = m_energy_functions->Epnt(cand, m_label_names[i]);

		nodes[count++] = mrf->AddNode(TypeGeneral::LocalSize(labelNum), TypeGeneral::NodeData(D));
		QString unary_potential = "Add Node_" + QString::number(count) + ": ";
		for (int j = 0; j < labelNum - 1; j++)
			unary_potential.append(QString::number(D[j]) + " ");
		unary_potential.append(QString::number(D[labelNum - 1]));
		qDebug() << unary_potential;
	}

	/* Set the pairwise potentials, each pairwise term is an edge between two nodes */
	std::cout << "Setting pairwise potentials..." << std::endl;
	for (int i = 0; i < nodeNum; i++)
	{
		PAPart cand1 = m_part_candidates[i];
		for (int j = i + 1; j < nodeNum; j++)
		{
			PAPart cand2 = m_part_candidates[j];
			int label1, label2;

			/* For each label pair of the two candidates, compute a energy value. 
			 * Note that the last label in label_names is null label 
			 */
			for (int l_idx_1 = 0; l_idx_1 < labelNum; l_idx_1++)    
			{
				label1 = m_label_names[l_idx_1];
				for (int l_idx_2 = 0; l_idx_2 < labelNum; l_idx_2++)
				{
					label2 = m_label_names[l_idx_2];
					/* Compute the energy of certain assumed labels. 
					 * Note that Epair() function would deal with the issues of null labels and same labels
					 */
					V[l_idx_1 + l_idx_2 * labelNum] = m_energy_functions->Epair(cand1, cand2, label1, label2);
					//qDebug("Node_%d - Node_%d: V(%d, %d) = %f.", i, j, label1, label2, V[l_idx_1 + l_idx_2 * labelNum]);
				}
			}

			/* Set the pairwise potential to the MRF */
			mrf->AddEdge(nodes[i], nodes[j], TypeGeneral::EdgeData(TypeGeneral::GENERAL, V));

			qDebug();
		}
	}

	/* Function below is optional - it may help if, for example, nodes are added in a random order */
	//mrf->SetAutomaticOrdering();

	/* Execute TRW-S algorithm */
	options.m_iterMax = 100; // maximum number of iterations
	options.m_printIter = 10;
	options.m_printMinIter = 0;
	mrf->ZeroMessages();
	mrf->AddRandomMessages(0, 0.0, 1.0);
	mrf->Minimize_TRW_S(options, lowerBound, energy);

	/* Read soluntions */
	std::vector<int> labels(nodeNum);
	QMap<int, int> parts_picked;  /* The parts picked out of candidates. key is part label, value is the index of candidate in parts_candidates*/
	int null_label = m_energy_functions->getNullLabelName();

	for (int i = 0; i < nodeNum; i++)
	{
		labels[i] = mrf->GetSolution(nodes[i]);
		if (labels[i] != null_label)
		{
			//assert(parts_picked.contains(labels[i]));
			if (parts_picked.contains(labels[i]))
				qDebug() << "parts_picked.contains(labels[i])!";
			else
				parts_picked.insert(labels[i], i);
			std::cout << "Part label " << labels[i] << " corresponds to Candidate-" << i << std:: endl;
		}
	}

	emit predictionDone(parts_picked);
	
	delete(D);
	delete(V);
}