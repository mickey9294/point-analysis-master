#include "partposeoptthread.h"

PartPoseOptThread::PartPoseOptThread(PartsStructure * parts_structure, 
	QSharedPointer<CuboidPredictor> predictor, QObject *parent)
	: QThread(parent), m_parts_structure(parts_structure), m_predictor(predictor)
{

}

PartPoseOptThread::~PartPoseOptThread()
{

}

void PartPoseOptThread::run()
{
	m_parts_solver.optimizeAttributes(*m_parts_structure, *m_predictor, opt_single_energy_term_weight,
		opt_symmetry_energy_term_weight, opt_max_iterations, false);

	const bool use_symmetry = !(disable_symmetry_terms);
	if (use_symmetry)
	{
		m_parts_structure->compute_symmetry_groups();

		m_parts_solver.optimizeAttributes(*m_parts_structure, *m_predictor, opt_single_energy_term_weight,
			opt_symmetry_energy_term_weight, opt_max_iterations, true);
	}
}