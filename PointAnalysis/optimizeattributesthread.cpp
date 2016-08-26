#include "optimizeattributesthread.h"

OptimizeAttributesThread::OptimizeAttributesThread(QObject *parent)
	: QThread(parent)
{

}

OptimizeAttributesThread::OptimizeAttributesThread(Part_Candidates parts, EnergyFunctions *energy_functions, QObject *parent)
	: QThread(parent), m_energy_functions(energy_functions), m_parts(parts)
{

}

OptimizeAttributesThread::~OptimizeAttributesThread()
{

}

void OptimizeAttributesThread::run()
{

}

void OptimizeAttributesThread::optimizeAttributes(const double _single_energy_term_weight, 
	const double _symmetry_energy_term_weight, 
	const int _max_num_iterations)
{
	int num_parts = m_parts.size();

	double single_total_energy, pair_total_energy, total_energy;

	double final_total_energy = std::numeric_limits<double>::max();
	int final_part_iteration = 0;


}
