#include "predictionthread.h"

PredictionThread::PredictionThread(QObject *parent)
	: QThread(parent), m_is_clean(true), m_use_symmetry(true), mrf(NULL), nodes(NULL)
{
	qRegisterMetaType<QMap<int, int>>("PartsPicked");
	qRegisterMetaType<std::vector<int>>("IntVector");
	m_modelClassName = "coseg_chairs_8";  
}

PredictionThread::PredictionThread(EnergyFunctions *energy_functions, Part_Candidates part_candidates, 
	QVector<int> label_names, std::string modelClassName, bool use_symmetry, QObject *parent)
	: QThread(parent), m_is_clean(true), m_use_symmetry(use_symmetry), mrf(NULL), nodes(NULL)
{
	m_energy_functions = energy_functions;
	m_ncandidates = part_candidates.size();
	m_part_candidates = Part_Candidates(part_candidates);
	m_label_names = label_names;
	m_modelClassName = modelClassName;

	qRegisterMetaType<QMap<int, int>>("PartsPicked");
	qRegisterMetaType<std::vector<int>>("IntVector");

	const int nodeNum = m_ncandidates;   /* the number of nodes */
	//connect(this, SIGNAL(testSignal), this, SLOT(onGetTest()));
}

PredictionThread::~PredictionThread()
{
	clean();
	for (QVector<UnaryTermThread *>::iterator it = m_unary_threads.begin(); it != m_unary_threads.end(); ++it)
	{
		UnaryTermThread * thread = *it;
		if (thread != NULL)
		{
			if (thread->isRunning())
				thread->terminate();
			delete(thread);
			thread = NULL;
		}
	}
	m_unary_threads.clear();

	for (QVector<PairwiseTermThread *>::iterator it = m_pairwise_threads.begin(); it != m_pairwise_threads.end(); ++it)
	{
		PairwiseTermThread * thread = *it;
		if (thread != NULL)
		{
			if (thread->isRunning())
				thread->terminate();
			delete(thread);
			thread = NULL;
		}
	}
	m_pairwise_threads.clear();
}

void PredictionThread::clean()
{
	if (mrf != NULL)
	{
		delete(mrf);
		mrf = NULL;
	}
	if (nodes != NULL)
	{
		delete(nodes);
		nodes = NULL;
	}

	m_is_clean = true;
}

void PredictionThread::run()
{
	/*predictLabelsAndOrientations();
	clean();*/

	/* Judge whether the  part labels and orientations have already been predicted */
	std::string winners_path = "../data/winners/" + m_modelClassName + ".txt";
	std::ifstream in(winners_path.c_str());
	if (in.is_open())
	{
		in.close();
		
		QMap<int, int> parts_picked = Utils::loadPredictionResult(winners_path);

		std::vector<int> candidates_labels(m_part_candidates.size(), m_energy_functions->getNullLabelName());

		for (QMap<int, int>::iterator winner_it = parts_picked.begin(); winner_it != parts_picked.end(); ++winner_it)
			candidates_labels[winner_it.value()] = winner_it.key();

		emit predictionDone(parts_picked, candidates_labels);
	}
	else
		singleThreadOptimize();
}

void PredictionThread::singleThreadOptimize()
{
	using namespace std;
	ofstream e_out("../data/pairwise_potentials.csv");
	ofstream u_out("../data/unary_potentials.csv");

	MRFEnergy<TypeGeneral>* mrf_s;
	MRFEnergy<TypeGeneral> *mrf_bp;
	MRFEnergy<TypeGeneral>::NodeId* nodes_s;
	MRFEnergy<TypeGeneral>::NodeId *nodes_bp;
	MRFEnergy<TypeGeneral>::Options options;
	MRFEnergy<TypeGeneral>::Options options_bp;
	TypeGeneral::REAL energy, lowerBound;

	const int labelNum = m_label_names.size();   /* the number of labels */
	const int nodeNum = m_ncandidates;   /* the number of nodes */

	mrf_s = new MRFEnergy<TypeGeneral>(TypeGeneral::GlobalSize());
	nodes_s = new MRFEnergy<TypeGeneral>::NodeId[nodeNum];
	mrf_bp = new MRFEnergy<TypeGeneral>(TypeGeneral::GlobalSize());
	nodes_bp = new MRFEnergy<TypeGeneral>::NodeId[nodeNum];

	TypeGeneral::REAL *D = new TypeGeneral::REAL[labelNum];
	TypeGeneral::REAL *V = new TypeGeneral::REAL[labelNum * labelNum];

	cout << "Set unary potentials." << endl;
	for (int i = 0; i < nodeNum; i++)
	{
		u_out << i << endl;

		PAPart cand = m_part_candidates[i];
		string unary_str = "Node_" + to_string(cand.getClusterNo()) + "_" + to_string(i) + ": ";

		Eigen::VectorXf unary_vec(labelNum);
		for (int j = 0; j < labelNum; j++)
		{
			D[j] = m_energy_functions->Epnt(&cand, j, m_use_symmetry);
			unary_vec[j] = D[j];

			if (j < labelNum - 1)
				u_out << D[j] << ",";
			else
				u_out << D[j] << endl;
		}
		
		nodes_s[i] = mrf_s->AddNode(TypeGeneral::LocalSize(labelNum), TypeGeneral::NodeData(D));
		nodes_bp[i] = mrf_bp->AddNode(TypeGeneral::LocalSize(labelNum), TypeGeneral::NodeData(D));

		cout << unary_str << endl;
		cout << unary_vec.transpose() << endl;
	}
	//e_out << endl;
	cout << "done." << endl;

	cout << "Set pairwise potentials." << endl;
	for (int i = 0; i < nodeNum; i++)
	{
		PAPart cand1 = m_part_candidates[i];
		int point_cluster_no_1 = cand1.getClusterNo();

		for (int j = i + 1; j < nodeNum; j++)
		{
			PAPart cand2 = m_part_candidates[j];
			int point_cluster_no_2 = cand2.getClusterNo();

			PAPartRelation relation(cand1, cand2);

			//e_out << "Node_" << cand1.getClusterNo() << "_" << i << " - Node_" << cand2.getClusterNo() << "_" << j << ":" << endl;
			e_out << i << "," << j << endl;

			//e_out << ",";
			/*for (int idx = 0; idx < labelNum; idx++)
			{
				if (idx < labelNum - 1)
					e_out << idx << ",";
				else
					e_out << idx << endl;
			}*/

			for (int m = 0; m < labelNum; m++)
			{
				//e_out << m << ",";
				for (int n = 0; n < labelNum; n++)
				{
					V[m + n * labelNum] = m_energy_functions->Epair(relation, point_cluster_no_1, point_cluster_no_2, m, n);
					//e_out << "V(" << m << ", " << n << ")=" << V[m + n * labelNum] << "\t";
					if (n < labelNum - 1)
						e_out << V[m + n * labelNum] << ",";
					else
						e_out << V[m + n * labelNum] << endl;
				} 
			}
			
			mrf_s->AddEdge(nodes_s[i], nodes_s[j], TypeGeneral::EdgeData(TypeGeneral::GENERAL, V));
			mrf_bp->AddEdge(nodes_bp[i], nodes_bp[j], TypeGeneral::EdgeData(TypeGeneral::GENERAL, V));
		}
	}
	cout << "done." << endl;
	e_out.close();
	u_out.close();

	/* Function below is optional - it may help if, for example, nodes are added in a random order */
	//mrf->SetAutomaticOrdering();

	/* Execute TRW-S algorithm */
	cout << "Do TRW_S algorithm." << endl;
	options.m_iterMax = 100; // maximum number of iterations
	options.m_printIter = 10;
	options.m_printMinIter = 0;
	mrf_s->ZeroMessages();
	mrf_s->AddRandomMessages(0, 0.0, 1.0);
	mrf_s->Minimize_TRW_S(options, lowerBound, energy);
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

	MRFEnergy<TypeGeneral>* mrf_optimized;
	MRFEnergy<TypeGeneral>::NodeId * nodes_optimized;
	if (energy < energy_bp)
	{
		//cout << "TRW_S algorithm is more optimized. Energy = " << energy << endl;
		qDebug("TRW_S algorithm is more optimized. Energy = %f.", energy);
		mrf_optimized = mrf_s;
		nodes_optimized = nodes_s;
	}
	else
	{
		//cout << "BP algorithm is more optimized. Energy = " << energy_bp << endl;
		qDebug("BP algorithm is more optimized. Energy = %f.", energy);
		mrf_optimized = mrf_bp;
		nodes_optimized = nodes_bp;
	}

	/* Read soluntions */
	std::vector<int> labels(nodeNum);    /* The i-th component of labels represents the label assigned to the i-th candidate node */
	QMap<int, int> parts_picked;  /* The parts picked out of candidates. key is part label, value is the index of candidate in parts_candidates*/
	int null_label = m_energy_functions->getNullLabelName();

	QMap<int, QList<int>> winners;
	QList<int> labels_with_multi_winners;
	
	for (int i = 0; i < nodeNum; i++)
	{
		labels[i] = mrf_optimized->GetSolution(nodes_optimized[i]);
		if (labels[i] != null_label)
		{
			if (winners.contains(labels[i]))
			{
				winners[labels[i]].push_back(i);
				if (winners[labels[i]].size() == 2)
					labels_with_multi_winners.push_back(labels[i]);
			}
			else
			{
				winners.insert(labels[i], QList<int>());
				winners[labels[i]].push_back(i);
			}
		}
	}

	if (labels_with_multi_winners.size() > 0)
	{
		//qSort(labels_with_multi_winners.begin(), labels_with_multi_winners.end());
		QSet<int> labels_with_multi_winners_set = QSet<int>::fromList(labels_with_multi_winners);

		/* Generate all winners combinations */
		QList<QMap<int, int>> winner_combinations;
		generateCombinations(winners, labels_with_multi_winners, winner_combinations);

		selectMostOptimized(winner_combinations, parts_picked);
	}
	else
	{
		for (QMap<int, QList<int>>::iterator it = winners.begin(); it != winners.end(); ++it)
			parts_picked.insert(it.key(), it->front());
	}

	/* Output the winner parts indicese to local files */
	Utils::savePredictionResult(parts_picked, "../data/winners/" + m_modelClassName + ".txt");

	emit predictionDone(parts_picked, labels);
}

void PredictionThread::generateCombinations(const QMap<int, QList<int>> & winners,
	const QList<int> & labels_with_multi_winners, QList<QMap<int, int>> & combinations)
{
	int num_levels = labels_with_multi_winners.size();
	QVector<QList<int>> multi_winners(num_levels);
	
	int idx = 0;
	for (QList<int>::const_iterator it = labels_with_multi_winners.begin(); it != labels_with_multi_winners.end(); ++it)
		multi_winners[idx++] = winners[*it];

	QSet<int> all_labels = QSet<int>::fromList(winners.keys());
	QSet<int> single_labels = all_labels.subtract(QSet<int>::fromList(labels_with_multi_winners));

	QList<QList<int>> combs;

	recurGenerateCombinations(multi_winners, QList<int>(), 0, combs);

	for (QList<QList<int>>::iterator comb_it = combs.begin(); comb_it != combs.end(); ++comb_it)
	{
		QMap<int, int> combination;
		for (QSet<int>::iterator single_it = single_labels.begin(); single_it != single_labels.end(); ++single_it)
			combination.insert(*single_it, winners[*single_it].front());

		QList<int>::iterator multi_it;
		QList<int>::const_iterator multi_idx_it;
		for (multi_it = comb_it->begin(), multi_idx_it = labels_with_multi_winners.begin(); 
			multi_it != comb_it->end() && multi_idx_it != labels_with_multi_winners.end(); ++multi_idx_it, ++multi_it)
		{
			int multi_winner_label = *multi_idx_it;
			int multi_winner_index = *multi_it;
			combination.insert(multi_winner_label, multi_winner_index);
		}

		combinations.push_back(combination);
	}
}

void PredictionThread::recurGenerateCombinations(const QVector<QList<int>> & multi_winners, const QList<int> & current, int level, QList<QList<int>> & output)
{
	QList<int> level_winners = multi_winners[level];
	int level_num = level_winners.size();

	if (level == multi_winners.size() - 1)  /* If it is the bottom level */
	{
		for (int i = 0; i < level_num; i++)
		{
			QList<int> sub_output(current);
			sub_output.push_back(level_winners[i]);
			output.push_back(sub_output);
		}
	}
	else
	{
		for (int i = 0; i < level_num; i++)
		{
			QList<int> next(current);
			next.push_back(level_winners[i]);
			QList<QList<int>> next_output;
			recurGenerateCombinations(multi_winners, next, level + 1, next_output);
			
			for (QList<QList<int>>::iterator it = next_output.begin(); it != next_output.end(); ++it)
				output.push_back(*it);
		}
	}
}

void PredictionThread::predictLabelsAndOrientations()
{
	MRFEnergy<TypeGeneral>::Options options;
	TypeGeneral::REAL energy, lowerBound;

	const int nodeNum = m_ncandidates;   /* the number of nodes */
	const int labelNum = m_label_names.size();   /* the number of labels */

	/* Function below is optional - it may help if, for example, nodes are added in a random order */
	//mrf->SetAutomaticOrdering();

	/* Execute TRW-S algorithm */
	options.m_iterMax = 100; // maximum number of iterations
	//options.m_printIter = 10;
	//options.m_printMinIter = 0;
	//mrf->ZeroMessages();
	//mrf->AddRandomMessages(0, 0.0, 1.0);
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
			std::cout << "Part label " << labels[i] << " corresponds to Candidate-" << i << std::endl;
		}
	}

	emit predictionDone(parts_picked, labels);
}

void PredictionThread::execute()
{
	if (!m_is_clean)
		clean();

	/* Construct energy */
	const int NUM_OF_SUBTHREADS = 8;
	const int labelNum = m_label_names.size();   /* the number of labels */
	const int nodeNum = m_ncandidates;   /* the number of nodes */
	int num_of_classes = labelNum - 1;    /* '-1' is to remove the null label */

	mrf = new MRFEnergy<TypeGeneral>(TypeGeneral::GlobalSize());
	nodes = new MRFEnergy<TypeGeneral>::NodeId[nodeNum];
	m_is_clean = false;

	/* Create 8 subthreads to set unary potentials */
	int num_of_rounds, candidates_num_per_thread;
	if (nodeNum > NUM_OF_SUBTHREADS)
	{
		num_of_rounds = NUM_OF_SUBTHREADS;
		candidates_num_per_thread = nodeNum / NUM_OF_SUBTHREADS;
	}
	else
	{
		num_of_rounds = nodeNum;
		candidates_num_per_thread = 1;
	}

	m_unary_threads.resize(num_of_rounds);
	m_unary_threads.fill(NULL);
	unfinished_unary_threads = num_of_rounds;

	for (int i = 0; i < num_of_rounds; i++)
	{
		int start = i * candidates_num_per_thread;
		int end;
		if (i < num_of_rounds - 1)
			end = (i + 1) * candidates_num_per_thread - 1;
		else
			end = nodeNum - 1;

		UnaryTermThread *unary_thread = new UnaryTermThread(i, m_part_candidates, start, end, m_energy_functions, m_label_names, this);
		connect(unary_thread, SIGNAL(computeDone(int, int, Unary_Potentials)), this, SLOT(onGetUnaryPotentials(int, int, Unary_Potentials)));
		m_unary_threads[i] = unary_thread;
		unary_thread->start();
	}

	/* Test time of adding pairwise potentials */
	//long start_time = Utils::getCurrentTime();
	//PAPart part1 = m_part_candidates.first();
	//PAPart part2 = m_part_candidates.last();
	//PAPartRelation relation(part1, part2);
	//std::cout << "Time test start." << std::endl;
	//for (int i = 0; i < 1; i++)
	//{
	//	double value = m_energy_functions->Epair(relation, part1.getClusterNo(), part2.getClusterNo(), 0, 5);
	//}
	//long end_time = Utils::getCurrentTime();
	//std::cout << "Duration = " << end_time - start_time << " ms." << std::endl;
}

void PredictionThread::onGetUnaryPotentials(int id, int start_idx, Unary_Potentials unary_potentials)
{
	qDebug("Received unary potentials of Node-%d to Node-%d from UnaryTermThread-%d.", start_idx, start_idx + unary_potentials.size() - 1, id);
	
	int labelNum = m_label_names.size();
	int count = 0;
	for (QVector<double *>::iterator it = unary_potentials.begin(); it != unary_potentials.end(); ++it)
	{
		TypeGeneral::REAL *D = *it;
		int node_idx = start_idx + count;
		nodes[node_idx] = mrf->AddNode(TypeGeneral::LocalSize(labelNum), TypeGeneral::NodeData(D));

		QString unary_potential_str = "Add Node_" + QString::number(node_idx) + ": ";
		for (int j = 0; j < labelNum - 1; j++)
			unary_potential_str.append(QString::number(D[j]) + " ");
		unary_potential_str.append(QString::number(D[labelNum - 1]));
		qDebug() << unary_potential_str;

		count++;
		delete(D);
	}

	unfinished_unary_threads--;

	/* If it is the last thread to compute unary potentials, then start threads to compute pairwise potentials */
	if (unfinished_unary_threads == 0)
	{
		qDebug() << "The unary potentials setting done.";

		const int NUM_OF_SUBTHREADS = 16;
		int num_of_cands = m_part_candidates.size();
		int num_of_units = (int)((1.0 + (float)NUM_OF_SUBTHREADS) * (float)NUM_OF_SUBTHREADS / 2.0);
		float unit = (float)num_of_cands / (float)num_of_units;

		m_pairwise_threads.resize(NUM_OF_SUBTHREADS);
		m_pairwise_threads.fill(NULL);
		unfinished_pairwise_threads = NUM_OF_SUBTHREADS;

		int start_idx = 0;
		int i;
		start_time = Utils::getCurrentTime();
		for (i = 0; i < NUM_OF_SUBTHREADS && start_idx < num_of_cands; i++)
		{
			int end;
			if (i != NUM_OF_SUBTHREADS - 1)
			{
				int n = (int)((i + 1) * unit);
				if (n < 1)
					n = 1;

				end = start_idx + n - 1;
				if (end > num_of_cands - 1)
					end = num_of_cands - 1;
			}
			else
				end = num_of_cands - 1;

			PairwiseTermThread *thread = new PairwiseTermThread(i, m_part_candidates, start_idx, end, m_energy_functions, m_label_names, this);
			connect(thread, SIGNAL(computeDone(int, int, Pairwise_Potentials)), this, SLOT(onGetPairwisePotentials(int, int, Pairwise_Potentials)));
			m_pairwise_threads[i] = thread;
			thread->start();

			start_idx = end + 1;
		}
		if (i < NUM_OF_SUBTHREADS)
		{
			m_pairwise_threads.resize(i);
			unfinished_pairwise_threads = i;
		}
	}
}

void PredictionThread::onGetPairwisePotentials(int id, int start_idx, Pairwise_Potentials pairwise_potentials)
{
	qDebug("Received pairwise potentials of Node-%d to Node-%d from PairwiseTermThread-%d.", start_idx, start_idx + pairwise_potentials.size() - 1, id);

	int outter_count = 0;
	for (Pairwise_Potentials::iterator outter_it = pairwise_potentials.begin(); outter_it != pairwise_potentials.end(); ++outter_it)
	{
		QVector<double *> potentials = *outter_it;
		int first_cand_idx = start_idx + outter_count;

		int inner_count = 1;
		for (QVector<double *>::iterator inner_it = potentials.begin(); inner_it != potentials.end(); ++inner_it)
		{
			double * V = *inner_it;
			int second_cand_idx = first_cand_idx + inner_count;

			mrf->AddEdge(nodes[first_cand_idx], nodes[second_cand_idx], TypeGeneral::EdgeData(TypeGeneral::GENERAL, V));

			inner_count++;
			delete(V);
		}

		outter_count++;
	}

	unfinished_pairwise_threads--;

	if (unfinished_pairwise_threads == 0)
	{
		qDebug() << "The pairwise potentials setting done.";
		end_time = Utils::getCurrentTime();
		int duration = end_time - start_time;
		qDebug("Time spent: %d ms.", duration);
		start();
	}
}

void PredictionThread::selectMostOptimized(const QList<QMap<int, int>> & combinations, QMap<int, int> & most_optimized)
{
	int min_comb_index = 0;
	double min_energy = 1.0e9;

	int idx = 0;
	for (QList<QMap<int, int>>::const_iterator it = combinations.begin(); it != combinations.end(); ++it)
	{
		double energy = computeEnergy(*it);
		//cout << "Part_1 choose " << it->value(1) << ", energy = " << energy << endl;
		qDebug("Part_1 choose Cand_%d, energy = %f.", it->value(1), energy);
		if (energy < min_energy)
		{
			min_energy = energy;
			min_comb_index = idx;
		}

		idx++;
	}

	QMap<int, int> test = combinations.front();
	test[1] = 48;
	double e = computeEnergy(test);
	qDebug("Part_1 choose Cand_48, energy = %f.", e);

	most_optimized = combinations[min_comb_index];
}

double PredictionThread::computeEnergy(const QMap<int, int> & combination)
{
	double energy = 0;
	
	/* Add unary energies */
	for (QMap<int, int>::const_iterator it = combination.begin(); it != combination.end(); ++it)
		energy += m_energy_functions->Epnt(&m_part_candidates[it.value()], it.key());
	
	/* Add pairwise energies */
	for (QMap<int, int>::const_iterator it_1 = combination.begin(); it_1 != combination.end(); ++it_1)
	{
		for (QMap<int, int>::const_iterator it_2 = combination.begin(); it_2 != combination.end(); ++it_2)
		{
			PAPart cand1 = m_part_candidates[it_1.value()];
			PAPart cand2 = m_part_candidates[it_2.value()];
			PAPartRelation relation(cand1, cand2);
			energy += m_energy_functions->Epair(relation, cand1.getClusterNo(), cand2.getClusterNo(), it_1.key(), it_2.key());
		}
	}
	
	QVector<int> labels = QVector<int>::fromList(combination.keys());
	for (int i = 0; i < labels.size(); i++)
	{
		for (int j = i + 1; j < labels.size(); j++)
		{
			PAPart cand1 = m_part_candidates[labels[i]];
			PAPart cand2 = m_part_candidates[labels[j]];
			PAPartRelation relation(cand1, cand2);
			energy += m_energy_functions->Epair(relation, cand1.getClusterNo(), cand2.getClusterNo(), labels[i], labels[j]);
		}
	}

	return energy;
}