#ifndef LOADTHREAD_H
#define LOADTHREAD_H

#include <QThread>
#include <QDebug>
#include <QString>
#include <fstream>
#include "pcmodel.h"
#include "model.h"
#include "meshmodel.h"
#include "meshpcmodel.h"
#include <boost/filesystem.hpp>
#include "utils.h"
#include "obb.h"

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
	void sendOBBs(QVector<OBB *> obbs);
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
	void loadOBBs(const char *file_path, QVector<OBB *> & obbs);
};

#endif // LOADTHREAD_H
