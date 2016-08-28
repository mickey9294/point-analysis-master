#ifndef PARTPOSEOPTTHREAD_H
#define PARTPOSEOPTTHREAD_H

#include <QThread>
#include "PartsStructure.h"
#include "partssolver.h"
#include "CuboidPredictor.h"

class PartPoseOptThread : public QThread
{
	Q_OBJECT

public:
	PartPoseOptThread(PartsStructure *parts_structure, CuboidPredictor *predictor, QObject *parent = 0);
	~PartPoseOptThread();

protected:
	void run();

private:
	PartsStructure *m_parts_structure;
	CuboidPredictor *m_predictor;
	PartsSolver m_parts_solver;
};

#endif // PARTPOSEOPTTHREAD_H
