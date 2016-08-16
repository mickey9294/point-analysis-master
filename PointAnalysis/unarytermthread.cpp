#include "unarytermthread.h"

UnaryTermThread::UnaryTermThread(QObject *parent)
	: QThread(parent)
{
	qRegisterMetaType<Unary_Potentials>("UnaryPotentials");
}

UnaryTermThread::UnaryTermThread(int id, Part_Candidates part_candidates, int start, int end, 
	EnergyFunctions * energy_functions, QVector<int> label_names, QObject *parent)
	: QThread(parent), m_id(id), m_part_candidates(part_candidates), 
	m_start(start), m_end(end), m_energy_functions(energy_functions), m_label_names(label_names)
{
	qDebug("Create UnaryTermThread-%d to compute unary potentials of Node-%d to Node-%d", id, start, end);
	qRegisterMetaType<Unary_Potentials>("UnaryPotentials");
}

UnaryTermThread::~UnaryTermThread()
{

}

void UnaryTermThread::run()
{
	computeUnaryPotentials();
}

void UnaryTermThread::computeUnaryPotentials()
{
	Unary_Potentials unary_potentials(m_end - m_start + 1);
	int labelNum = m_label_names.size();

	for (int i = m_start; i <= m_end; i++)
	{
		PAPart cand = m_part_candidates[i];
		double * D = new double[labelNum];

		for (int j = 0; j < labelNum; j++)
			D[j] = m_energy_functions->Epnt(cand, m_label_names[j]);

		unary_potentials[i - m_start] = D;
	}

	emit computeDone(m_id, m_start, unary_potentials);
}
