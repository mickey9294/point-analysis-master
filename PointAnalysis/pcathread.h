#ifndef PCATHREAD_H
#define PCATHREAD_H

#include <QThread>
#include <qdebug.h>
#include <qpair.h>
#include <qpair.h>
#include <qmap.h>
#include <QVector>
#include <QList>
#include <pcl/point_types.h>
#include "pcmodel.h"
#include "obb.h"
#include "obbestimator.h"
#include "papart.h"
#include "papartrelation.h"
#include "utils.h"

class PCAThread : public QThread
{
	Q_OBJECT

public:
	PCAThread(QObject *parent = 0);
	PCAThread(PCModel *pcModel, QObject *parent = 0);
	~PCAThread();

	void setPointCloud(PCModel * pcModel);

signals:
	void estimateOBBsCompleted(QVector<OBB*> obbs);
	void addDebugText(QString text);
	void estimatePartsDone(QVector<PAPart> parts);

protected:
	void run();

private:
	QMap<int, pcl::PointCloud<pcl::PointXYZ>::Ptr> m_parts;
	PCModel * m_pcModel;

	void loadParts(PCModel *pcModel);
	QVector<OBB *> m_OBBs;
};

#endif // PCATHREAD_H
