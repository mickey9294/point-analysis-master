#ifndef PREDICTIONTHREAD_H
#define PREDICTIONTHREAD_H

#include <QThread>
#include <QMap>
#include <qvector.h>
#include <assert.h>
#include "gencandidatesthread.h"
#include "MRFEnergy.h"
#include "energyfunctions.h"
#include "pairwisetermthread.h"
#include "unarytermthread.h"

/*
 * The sub thread used to do  part lebels and orientations prediction. 
 */
class PredictionThread : public QThread
{
	Q_OBJECT

public:
	PredictionThread(QObject *parent = 0);
	PredictionThread(EnergyFunctions *energy_functions, Part_Candidates part_candidates, QList<int> label_names, QObject *parent = 0);
	~PredictionThread();

	void execute();

	public slots:
	void onGetUnaryPotentials(int id, int start_idx, Unary_Potentials unary_potentials);
	void onGetPairwisePotentials(int id, int start_idx, Pairwise_Potentials pairwise_potentials);
	//void onGetTest();

signals:
	void predictionDone(QMap<int, int> parts_picked);
	//void predictionDone();
	//void testSignal();

protected:
	void run();

private:
	Part_Candidates m_part_candidates;
	QList<int> m_label_names;    /* The label set. Note that the label with the largest number is null label */
	int m_ncandidates;
	EnergyFunctions *m_energy_functions;
	QVector<PairwiseTermThread *> m_pairwise_threads;
	QVector<UnaryTermThread *> m_unary_threads;
	MRFEnergy<TypeGeneral>* mrf;
	MRFEnergy<TypeGeneral>::NodeId* nodes;
	int unfinished_unary_threads;
	int unfinished_pairwise_threads;
	bool m_is_clean;
	long start_time;
	long end_time;

	void predictLabelsAndOrientations();
	void clean();
	void singleThreadOptimize();
};

#endif // PREDICTIONTHREAD_H
