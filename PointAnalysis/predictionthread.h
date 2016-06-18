#ifndef PREDICTIONTHREAD_H
#define PREDICTIONTHREAD_H

#include <QThread>
#include <QMap>
#include <qvector.h>
#include "gencandidatesthread.h"
#include "MRFEnergy.h"

/*
 * The sub thread used to do  part lebels and orientations prediction. 
 */
class PredictionThread : public QThread
{
	Q_OBJECT

public:
	PredictionThread(QObject *parent = 0);
	PredictionThread(int ncandidates, Part_Candidates part_candidates, QVector<QMap<int, float>> distribution, QObject *parent = 0);
	~PredictionThread();

protected:
	void run();

private:
	Part_Candidates m_part_candidates;
	QVector<QMap<int, float>> m_distribution;
	int m_ncandidates;

	void predictLabelsAndOrientations();
};

#endif // PREDICTIONTHREAD_H
