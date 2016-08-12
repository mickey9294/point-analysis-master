#ifndef LOADTHREAD_H
#define LOADTHREAD_H

#include <QThread>
#include <QDebug>
#include <QString>
#include <fstream>
#include "pcmodel.h"
#include "model.h"
#include "meshmodel.h"
#include "utils.h"

class LoadThread : public QThread
{
	Q_OBJECT

public:
	LoadThread(QObject *parent = 0);
	LoadThread(std::string filename, PHASE phase, QObject *parent = 0);
	~LoadThread();

	void setLoadFileName(std::string filename);
	void setPhase(PHASE phase);
	void setGapTime(int time);

signals:
	void loadPointsCompleted(Model *model);
	void computeFeaturesCompleted();
	void reportStatus(QString stat);
	void addDebugText(QString text);

protected:
	void run();

private:
	std::string filename;
	PHASE m_phase;
	int m_gap_time;

	void loadPointCloud();
};

#endif // LOADTHREAD_H
