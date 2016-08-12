#ifndef PCATHREAD_H
#define PCATHREAD_H

#include <QThread>
#include <qdebug.h>
#include <qpair.h>
#include <qpair.h>
#include <qmap.h>
#include <QVector>
#include <fstream>
#include <QList>
#include <Eigen\Core>
#include <string>
#include <assert.h>
#include <pcl/point_types.h>
#include "pcmodel.h"
#include "obb.h"
#include "obbestimator.h"
#include "papart.h"
#include "papartrelation.h"
#include "utils.h"
#include "ICP.h"
#include "meshmodel.h"

class PCAThread : public QThread
{
	Q_OBJECT

public:
	PCAThread(QObject *parent = 0);
	PCAThread(Model *model, PHASE phase, QObject *parent = 0);
	~PCAThread();

	void setPointCloud(Model * model);

signals:
	void estimateOBBsCompleted(QVector<OBB*> obbs);
	void addDebugText(QString text);
	void estimatePartsDone(QVector<PAPart> parts);
	void sendSamples(Samples_Vec samples);

protected:
	void run();

private:
	QMap<int, pcl::PointCloud<pcl::PointXYZ>::Ptr> m_parts;
	Model * m_model;
	QVector<OBB *> m_OBBs;
	PHASE m_phase;

	void loadParts(Model *model);
};

#endif // PCATHREAD_H
