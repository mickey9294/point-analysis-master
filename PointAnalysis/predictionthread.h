#ifndef PREDICTIONTHREAD_H
#define PREDICTIONTHREAD_H

#include <QThread>
#include <QMap>
#include <qvector.h>
#include <assert.h>
#include "gencandidatesthread.h"
#include "MRFEnergy.h"
#include "energyfunctions.h"

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

signals:
	void predictionDone(QMap<int, int> parts_picked);

protected:
	void run();

private:
	Part_Candidates m_part_candidates;
	QList<int> m_label_names;    /* The label set. Note that the label with the largest number is null label */
	int m_ncandidates;
	EnergyFunctions *m_energy_functions;

	void predictLabelsAndOrientations();
};

#endif // PREDICTIONTHREAD_H
