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
#include "utils_sampling.hpp"
#include "ICP.h"

class PCAThread : public QThread
{
	Q_OBJECT

public:
	enum PHASE{
		TRAINING,
		TESTING
	};

	PCAThread(QObject *parent = 0);
	PCAThread(PCModel *pcModel, PHASE phase, QObject *parent = 0);
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
	QVector<OBB *> m_OBBs;
	PHASE m_phase;

	void loadParts(PCModel *pcModel);
	void ICP_procedure();
	/* Load the mesh from file for ICP */
	void loadMesh(std::vector<Utils_sampling::Vec3> &points_list, std::vector<Eigen::Vector3i> &faces_list, QMap<int, QList<int>> &label_faces_map);
	void sampleOnMesh(QMap<int, std::vector<Eigen::Vector3f>> &parts_samples);
	void sampleOnOBBs(QMap<int, std::vector<Eigen::Vector3f>> &obbs_samples);
};

#endif // PCATHREAD_H
