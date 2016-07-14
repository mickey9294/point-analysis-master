#include "pairwisetermthread.h"

PairwiseTermThread::PairwiseTermThread(QObject *parent)
	: QThread(parent), m_energy_functions(NULL)
{
	qRegisterMetaType<Pairwise_Potentials>("PairwisePotentials");
}

PairwiseTermThread::PairwiseTermThread(int id, Part_Candidates part_candidates, int start, int end, 
	EnergyFunctions *energy_functions, QList<int> label_names, QObject *parent)
	: QThread(parent), m_part_candidates(part_candidates), m_start(start), m_end(end), m_energy_functions(energy_functions),
	m_label_names(label_names), m_id(id)
{
	qDebug("Create PairwiseTermThread-%d to compute pairwise potentials of Node-%d to Node-%d", id, start, end);
	qRegisterMetaType<Pairwise_Potentials>("PairwisePotentials");
}

PairwiseTermThread::~PairwiseTermThread()
{

}

void PairwiseTermThread::run()
{
	computePairwisePotentials();
}

void PairwiseTermThread::computePairwisePotentials()
{
	int num_of_candidates = m_part_candidates.size();
	int labelNum = m_label_names.size();
	int N1 = m_end - m_start + 1;
	Pairwise_Potentials pairwise_potentials(N1);

	int outter_count = 0;
	for (int i = m_start; i <= m_end; i++)
	{
		int N2 = num_of_candidates - i - 1;
		QVector<double *> potentials(N2);

		PAPart cand1 = m_part_candidates[i];

		int inner_count = 0;
		for (int j = i + 1; j < num_of_candidates; j++)
		{
			double *V = new double[labelNum * labelNum];
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
					qDebug("PairwiseTermThread-%d: Node_%d - Node_%d: V(%d, %d) = %f.", m_id, i, j, label1, label2, V[l_idx_1 + l_idx_2 * labelNum]);
				}
			}

			potentials[inner_count++] = V;
		}

		pairwise_potentials[outter_count++] = potentials;
	}

	emit computeDone(m_id, m_start, pairwise_potentials);
	//qDebug("PairwiseTermThread-%d done.", m_id);
}