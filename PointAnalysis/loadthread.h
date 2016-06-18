#ifndef LOADTHREAD_H
#define LOADTHREAD_H

#include <QThread>
#include <QDebug>
#include <QString>
#include "pcmodel.h"
#include "utils.h"

class LoadThread : public QThread
{
	Q_OBJECT

public:
	enum PHASE{
		TRAINING,
		TESTING
	};

	LoadThread(QObject *parent = 0);
	LoadThread(std::string filename, PHASE phase, QObject *parent = 0);
	~LoadThread();

	void setLoadFileName(std::string filename);
	void setPhase(PHASE phase);

signals:
	void loadPointsCompleted(PCModel *model);
	void computeFeaturesCompleted();
	void reportStatus(QString stat);
	void addDebugText(QString text);

protected:
	void run();

private:
	std::string filename;
	PHASE m_phase;

	void loadPointCloud();
};

#endif // LOADTHREAD_H
