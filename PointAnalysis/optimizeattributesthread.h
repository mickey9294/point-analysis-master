#ifndef OPTIMIZEATTRIBUTESTHREAD_H
#define OPTIMIZEATTRIBUTESTHREAD_H

#include <QThread>
#include "constants.h"
#include "energyfunctions.h"

class OptimizeAttributesThread : public QThread
{
	Q_OBJECT

public:
	OptimizeAttributesThread(QObject *parent = 0);
	OptimizeAttributesThread(Part_Candidates parts, EnergyFunctions *energy_functions, QObject *parent = 0);
	~OptimizeAttributesThread();

protected:
	void run();

private:
	EnergyFunctions * m_energy_functions;
	Parts_Vector m_parts;

	void optimizeAttributes(const double _single_energy_term_weight, const double _symmetry_energy_term_weight, const int _max_num_iterations);
};

#endif // OPTIMIZEATTRIBUTESTHREAD_H
