#ifndef DEBUGRELATIONTHREAD_H
#define DEBUGRELATIONTHREAD_H

#include <QThread>
#include <fstream>
#include "loadthread.h"
#include "papart.h"
#include "pcathread.h"
#include "papartrelation.h"
#include "pcmodel.h"
#include "utils.h"

class DebugRelationThread : public QThread
{
	Q_OBJECT

public:
	DebugRelationThread(QObject *parent = 0);
	~DebugRelationThread();

	void execute();

	public slots:
	void receiveModel(PCModel *model);
	void receiveParts(QVector<PAPart> parts);

protected:
	void run();

private:
	LoadThread loadThread;
	PCAThread pcaThread;
	QVector<PAPart> m_parts;
};

#endif // DEBUGRELATIONTHREAD_H
