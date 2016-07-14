#ifndef UNARYTERMTHREAD_H
#define UNARYTERMTHREAD_H

#include <QThread>
#include <QList>
#include "MRFEnergy.h"
#include "energyfunctions.h"

typedef QVector<double *> Unary_Potentials;
Q_DECLARE_METATYPE(Unary_Potentials)

class UnaryTermThread : public QThread
{
	Q_OBJECT

public:
	UnaryTermThread(QObject *parent = 0);
	UnaryTermThread(int id, Part_Candidates part_candidates, int start, int end, 
		EnergyFunctions * energy_functions, QList<int> label_names, QObject *parent = 0);
	~UnaryTermThread();

signals:
	void computeDone(int id, int start_idx, Unary_Potentials unary_potentials);

protected:
	void run();

private:
	int m_id;
	Part_Candidates m_part_candidates;
	int m_start;
	int m_end;
	EnergyFunctions * m_energy_functions;
	QList<int> m_label_names;

	void computeUnaryPotentials();
};

#endif // UNARYTERMTHREAD_H
