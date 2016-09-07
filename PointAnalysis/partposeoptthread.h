#ifndef PARTPOSEOPTTHREAD_H
#define PARTPOSEOPTTHREAD_H

#include <QThread>
#include <QSharedPointer>
#include "PartsStructure.h"
#include "partssolver.h"
#include "CuboidPredictor.h"

class PartPoseOptThread : public QThread
{
	Q_OBJECT

public:
	PartPoseOptThread(PartsStructure *parts_structure, QSharedPointer<CuboidPredictor> predictor, QObject *parent = 0);
	~PartPoseOptThread();

protected:
	void run();

private:
	PartsStructure *m_parts_structure;
	QSharedPointer<CuboidPredictor> m_predictor;
	PartsSolver m_parts_solver;
};

#endif // PARTPOSEOPTTHREAD_H
