#ifndef PAIRWISETERMTHREAD_H
#define PAIRWISETERMTHREAD_H

#include <QThread>
#include <qvector.h>
#include <QDebug>
#include <QList>
#include "energyfunctions.h"
#include "papart.h"
#include <MRFEnergy.h>

typedef QVector<QVector<double *>> Pairwise_Potentials;
Q_DECLARE_METATYPE(Pairwise_Potentials)

class PairwiseTermThread : public QThread
{
	Q_OBJECT

public:
	PairwiseTermThread(QObject *parent = 0);
	PairwiseTermThread(int id, Part_Candidates part_candidates, int start, int end, EnergyFunctions *energy_functions,
		QList<int> label_names, QObject *parent = 0);
	~PairwiseTermThread();

signals:
	void computeDone(int id, int first_cand_no, Pairwise_Potentials pairwise_potentials);
	//void computeDone();

protected:
	void run();

private:
	int m_id;
	int m_start;
	int m_end;
	QList<int> m_label_names;
	Part_Candidates m_part_candidates;
	EnergyFunctions *m_energy_functions;
	//Pairwise_Potentials m_pairwise_potentials;

	void computePairwisePotentials();
};

#endif // PAIRWISETERMTHREAD_H
