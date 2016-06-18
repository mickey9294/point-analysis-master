#include "pcathread.h"

PCAThread::PCAThread(QObject *parent) : QThread(parent)
{
	
}

PCAThread::PCAThread(PCModel *pcModel, QObject *parent)
	: QThread(parent)
{
	m_pcModel = pcModel;
}

void PCAThread::setPointCloud(PCModel *pcModel)
{
	m_parts.clear();
	m_OBBs.clear();

	m_pcModel = pcModel;
}

PCAThread::~PCAThread()
{
	if (isRunning())
		terminate();
}

using namespace pcl;
void PCAThread::loadParts(PCModel * pcModel)
{
	emit addDebugText("Load parts from the point cloud.");

	int size = pcModel->vertexCount();
	int classSize = pcModel->numOfClasses();
	QVector<int> labels = pcModel->getLabels();
	QList<int> label_names = pcModel->getLabelNames();

	for (int i = 0; i < classSize; i++)
	{
		PointCloud<PointXYZ>::Ptr part(new PointCloud<PointXYZ>);
		int label_name = label_names[i];
		m_parts.insert(label_name, part);
	}
	
	for (int i = 0; i < size; i++)
	{
		GLfloat *point = pcModel->data() + i * 9;
		int label = labels[i];
		PointXYZ pointxyz(point[0], point[1], point[2]);
		m_parts[label]->push_back(pointxyz);
	}
}

void PCAThread::run()
{
	emit addDebugText("Computing oriented bounding boxes...");
	if (m_pcModel->vertexCount() > 0)
	{
		loadParts(m_pcModel);
		QVector<PAPart> parts;
		QVector<int> labels(m_parts.size());

		m_OBBs.resize(m_parts.size());

		QMap<int, PointCloud<PointXYZ>::Ptr>::iterator it;
		int i = 0;
		for (it = m_parts.begin(); it != m_parts.end(); it++)
		{
			int label = it.key();
			labels[i] = i;
			emit addDebugText("Compute OBB of part " + QString::number(label));

			OBBEstimator obbe(label, it.value());
			OBB * obb = obbe.computeOBB();
			obb->triangulate();
			parts.push_back(PAPart(obb));
			m_OBBs[i++] = obb;	
		}
		
		/*QVector<QPair<int, int>> part_pairs = Utils::getCombinations(labels);
		QVector<PAPartRelation> part_relations(part_pairs.size());
		for (int i = 0; i < part_pairs.size(); i++)
		{
			QPair<int, int> part_pair = part_pairs[i];
			PAPartRelation relation(parts[part_pair.first], parts[part_pair.second]);
			part_relations[i] = relation;
		}*/
		emit addDebugText("Computing OBB done.");
		emit estimateOBBsCompleted(m_OBBs);
		emit estimatePartsDone(parts);
	}
}