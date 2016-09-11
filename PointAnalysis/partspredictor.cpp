#include "partspredictor.h"

PartsPredictor::PartsPredictor(QObject *parent)
	: QObject(parent), m_use_symmetry(true), m_modelClassName("coseg_chairs_8")
{

}

PartsPredictor::~PartsPredictor()
{

}

void PartsPredictor::predictLabelsAndOrientations(const Part_Candidates & part_candidates, const QVector<int> & label_names,
	QMap<int, int> &parts_picked, std::vector<int> &candidate_labels, QSharedPointer<EnergyFunctions> m_energy_functions, bool first_run)
{
	using namespace std;

	/* Judge whether the  part labels and orientations have already been predicted */
	std::string winners_path = "../data/winners/" + m_modelClassName + ".txt";
	std::ifstream in(winners_path.c_str());
	if (first_run && in.is_open())
	{
		cout << "Load part labels and orientations from file." << endl;
		in.close();

		parts_picked = Utils::loadPredictionResult(winners_path);

		candidate_labels.resize(part_candidates.size(), m_energy_functions->getNullLabelName());

		for (QMap<int, int>::iterator winner_it = parts_picked.begin(); winner_it != parts_picked.end(); ++winner_it)
			candidate_labels[winner_it.value()] = winner_it.key();

		cout << "Loading done." << endl;
	}
	else
	{
		cout << "Predict part labels and orientations." << endl;

		ofstream e_out("../data/pairwise_potentials.csv");
		ofstream u_out("../data/unary_potentials.csv");

		MRFEnergy<TypeGeneral>* mrf_s;
		MRFEnergy<TypeGeneral> *mrf_bp;
		MRFEnergy<TypeGeneral>::NodeId* nodes_s;
		MRFEnergy<TypeGeneral>::NodeId *nodes_bp;
		MRFEnergy<TypeGeneral>::Options options;
		MRFEnergy<TypeGeneral>::Options options_bp;
		TypeGeneral::REAL energy, lowerBound;

		const int labelNum = label_names.size();   /* the number of labels */
		const int nodeNum = part_candidates.size();   /* the number of nodes */

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

			PAPart cand = part_candidates[i];
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
			PAPart cand1 = part_candidates[i];
			int point_cluster_no_1 = cand1.getClusterNo();

			for (int j = i + 1; j < nodeNum; j++)
			{
				PAPart cand2 = part_candidates[j];
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
			qDebug("BP algorithm is more optimized. Energy = %f.", energy_bp);
			mrf_optimized = mrf_bp;
			nodes_optimized = nodes_bp;
		}

		/* Read soluntions */
		std::vector<int> labels(nodeNum);    /* The i-th component of labels represents the label assigned to the i-th candidate node */
		//QMap<int, int> parts_picked;  /* The parts picked out of candidates. key is part label, value is the index of candidate in parts_candidates*/
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

			selectMostOptimized(winner_combinations, parts_picked, part_candidates, m_energy_functions);
		}
		else
		{
			for (QMap<int, QList<int>>::iterator it = winners.begin(); it != winners.end(); ++it)
				parts_picked.insert(it.key(), it->front());
		}

		candidate_labels.resize(nodeNum, null_label);
		for (QMap<int, int>::iterator pick_it = parts_picked.begin(); pick_it != parts_picked.end(); ++pick_it)
			candidate_labels[*pick_it] = pick_it.key();

		/* Output the winner parts indicese to local files */
		Utils::savePredictionResult(parts_picked, "../data/winners/" + m_modelClassName + ".txt");

		cout << "Part labels and orientations prediction done." << endl;
	}
}

void PartsPredictor::generateCombinations(const QMap<int, QList<int>> & winners,
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

void PartsPredictor::recurGenerateCombinations(const QVector<QList<int>> & multi_winners, const QList<int> & current, int level, QList<QList<int>> & output)
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

void PartsPredictor::selectMostOptimized(const QList<QMap<int, int>> & combinations, QMap<int, int> & most_optimized,
	const Part_Candidates & part_candidates, QSharedPointer<EnergyFunctions> energy_functions)
{
	int min_comb_index = 0;
	double min_energy = 1.0e9;

	int idx = 0;
	for (QList<QMap<int, int>>::const_iterator it = combinations.begin(); it != combinations.end(); ++it)
	{
		double energy = computeEnergy(*it, part_candidates, energy_functions);
		//cout << "Part_1 choose " << it->value(1) << ", energy = " << energy << endl;
		qDebug("Part_1 choose Cand_%d, energy = %f.", it->value(1), energy);
		if (energy < min_energy)
		{
			min_energy = energy;
			min_comb_index = idx;
		}

		idx++;
	}

	/*QMap<int, int> test = combinations.front();
	test[1] = 48;
	double e = computeEnergy(test, part_candidates, energy_functions);
	qDebug("Part_1 choose Cand_48, energy = %f.", e);*/

	most_optimized = combinations[min_comb_index];
}

double PartsPredictor::computeEnergy(const QMap<int, int> & combination, const Part_Candidates & part_candidates, QSharedPointer<EnergyFunctions> energy_functions)
{
	assert(!energy_functions.isNull());

	double energy = 0;

	/* Add unary energies */
	for (QMap<int, int>::const_iterator it = combination.begin(); it != combination.end(); ++it)
		energy += energy_functions->Epnt(&part_candidates[it.value()], it.key(), m_use_symmetry);

	/* Add pairwise energies */
	for (QMap<int, int>::const_iterator it_1 = combination.begin(); it_1 != combination.end(); ++it_1)
	{
		for (QMap<int, int>::const_iterator it_2 = combination.begin(); it_2 != combination.end(); ++it_2)
		{
			PAPart cand1 = part_candidates[it_1.value()];
			PAPart cand2 = part_candidates[it_2.value()];
			PAPartRelation relation(cand1, cand2);
			energy += energy_functions->Epair(relation, cand1.getClusterNo(), cand2.getClusterNo(), it_1.key(), it_2.key());
		}
	}

	/*QVector<int> labels = QVector<int>::fromList(combination.keys());
	for (int i = 0; i < labels.size(); i++)
	{
		for (int j = i + 1; j < labels.size(); j++)
		{
			PAPart cand1 = part_candidates[labels[i]];
			PAPart cand2 = part_candidates[labels[j]];
			PAPartRelation relation(cand1, cand2);
			energy += energy_functions->Epair(relation, cand1.getClusterNo(), cand2.getClusterNo(), labels[i], labels[j]);
		}
	}*/

	return energy;
}

void PartsPredictor::setUseSymmetry(bool use_symmetry)
{
	m_use_symmetry = use_symmetry;
}

void PartsPredictor::setModelClassName(std::string modelClassName)
{
	m_modelClassName = modelClassName;
}