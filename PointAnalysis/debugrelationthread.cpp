#include "debugrelationthread.h"

DebugRelationThread::DebugRelationThread(QObject *parent)
	: QThread(parent)
{
	qRegisterMetaType<QVector<PAPart>>("QVector<PAPart>");
	loadThread.setPhase(PHASE::TRAINING);

	connect(&loadThread, SIGNAL(loadPointsCompleted(PCModel *)), this, SLOT(receiveModel(PCModel *)));
	connect(&pcaThread, SIGNAL(estimatePartsDone(QVector<PAPart>)), this, SLOT(receiveParts(QVector<PAPart>)));
}

DebugRelationThread::~DebugRelationThread()
{

}

using namespace std;

void DebugRelationThread::execute()
{
	string model_path = "D:\\Projects\\Data\\ground_truth_datasets\\coseg_chairs\\off\\222.off";
	loadThread.setLoadFileName(model_path);
	loadThread.start();
}

void DebugRelationThread::receiveModel(PCModel *pcModel)
{
	qDebug() << "Receive point cloud from" << QString::fromStdString(pcModel->getInputFilepath());

	qDebug() << "Start compute OBBs of parts.";
	pcaThread.setPointCloud(pcModel);
	pcaThread.start();
}

void DebugRelationThread::receiveParts(QVector<PAPart> parts)
{
	qDebug() << "Receive parts vector.";

	m_parts = parts;
	start();
}

void DebugRelationThread::run()
{
	QVector<int> labels(m_parts.size());    /* the i-th component of the vector represents the label of the i-th part */
	QMap<int, int> labels_indices;    /* the label-index map showing the index of the part with the label */
	for (int i = 0; i < m_parts.size(); i++)
	{
		labels[i] = m_parts[i].getLabel();
		labels_indices.insert(labels[i], i);
	}
	
	ofstream mean_out("../data/parts_relations/mean_debug.txt");

	for (QMap<int, int>::iterator label_it_1 = labels_indices.begin(); label_it_1 != labels_indices.end(); label_it_1++)
	{
		int label1 = label_it_1.key();
		int part_index1 = label_it_1.value();
		for (QMap<int, int>::iterator label_it_2 = labels_indices.begin(); label_it_2 != labels_indices.end(); label_it_2++)
		{
			int label2 = label_it_2.key();
			if (label1 != label2)
			{
				int part_index2 = label_it_2.value();
				PAPartRelation relation(m_parts[part_index1], m_parts[part_index2]);
				mean_out << label1 << " " << label2 << endl;

				vector<float> features = relation.getFeatureVector_Float();
				for (int i = 0; i < features.size(); i++)
				{
					if (i < features.size() - 1)
						mean_out << features[i] << " ";
					else
						mean_out << features[i] << endl;
				}
			}
		}
	}

	mean_out.close();

	qDebug() << "done.";
}