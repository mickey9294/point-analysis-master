#include "pcathread.h"

PCAThread::PCAThread(QObject *parent) : QThread(parent)
{
	qRegisterMetaType<Samples_Vec>("SamplePoints");
	qRegisterMetaType<Parts_Vector>("Parts_Vector");
	m_phase = PHASE::TESTING;
}

PCAThread::PCAThread(Model *model, PHASE phase, QObject *parent)
	: QThread(parent)
{
	qRegisterMetaType<Samples_Vec>("SamplePoints");
	qRegisterMetaType<Parts_Vector>("Parts_Vector");
	m_model = model;
	m_phase = phase;
}

void PCAThread::setPointCloud(Model *model)
{
	m_part_clouds.clear();
	m_OBBs.clear();

	m_model = model;
}

PCAThread::~PCAThread()
{
	if (isRunning())
		terminate();
}

using namespace pcl;
void PCAThread::loadParts(Model * model)
{
	emit addDebugText("Load parts from the point cloud.");

	int size = model->vertexCount();
	int classSize = model->numOfClasses();
	QVector<int> label_names = model->getLabelNames();
	if (label_names.empty())
	{
		label_names.resize(1);
		label_names[0] = 10;
	}

	for (int i = 0; i < classSize; i++)
	{
		PointCloud<PointXYZ>::Ptr part(new PointCloud<PointXYZ>);
		int label_name = label_names[i];
		m_part_clouds.insert(label_name, part);

		PAPart *label_part = new PAPart();
		label_part->setLabel(label_name);
		m_parts.insert(label_name, label_part);
	}
	
	if (model->getType() == Model::ModelType::Mesh)  /* If it is a training mesh model */
	{
		MeshModel *mesh = (MeshModel *)model;
		if (mesh->sampleCount() <= 0)
			mesh->samplePoints();

		int idx = 0;
		for (Parts_Samples::iterator part_it = mesh->samples_begin(); part_it != mesh->samples_end(); ++part_it)
		{
			int label = part_it.key();

			for (QVector<SamplePoint>::iterator sample_it = part_it->begin(); sample_it != part_it->end(); ++sample_it)
			{
				PointXYZ point(sample_it->x(), sample_it->y(), sample_it->z());
				m_part_clouds[label]->push_back(point);

				m_parts[label]->addVertex(idx, sample_it->getVertex(), sample_it->getNormal());

				idx++;
			}
		}
	}
	else  /* If it is a testing point cloud model */
	{
		PCModel * pc = (PCModel *)model;

		QVector<int> labels = model->getVerticesLabels();

		int point_idx = 0;
		for (QVector<Eigen::Vector3f>::iterator point_it = pc->vertices_begin(); point_it != pc->vertices_end(); ++point_it)
		{
			int label = labels[point_idx];
			PointXYZ point(point_it->x(), point_it->y(), point_it->z());
			m_part_clouds[label]->push_back(point);

			m_parts[label]->addVertex(point_idx, *point_it, pc->getVertexNormal(point_idx));
			point_idx++;
		}
	}

	/* Remove part clouds with too few points(less than 15) */
	for (QVector<int>::iterator label_it = label_names.begin(); label_it != label_names.end(); ++label_it)
	{
		if (m_part_clouds[*label_it]->size() < 15)
		{
			m_part_clouds.remove(*label_it);
			m_parts.remove(*label_it);
		}
	}
}

void PCAThread::run()
{
	emit addDebugText("Computing oriented bounding boxes...");
	if (m_model->vertexCount() > 0)
	{
		loadParts(m_model);
		Parts_Vector parts;
		QVector<int> labels(m_part_clouds.size());

		m_OBBs.clear();

		/* Compute the oriented bounding box for each distinct part */
		QMap<int, PointCloud<PointXYZ>::Ptr>::iterator it;
		int i = 0;
		for (it = m_part_clouds.begin(); it != m_part_clouds.end(); it++)
		{
			if ((*it)->size() < 15)
				continue;

			int label = it.key();
			labels[i] = i;
			emit addDebugText("Compute OBB of part " + QString::number(label));

			OBBEstimator obbe(label, it.value());
			obbe.setPhase(m_phase);
			OBB *obb = obbe.computeOBB();
			//obb->setLabel(label);

			m_parts[label]->setOBB(obb);
			
			if (m_phase == PHASE::TESTING)
				m_parts[label]->ICP_adjust_OBB();

			parts.push_back(m_parts[label]);
			m_OBBs.push_back(new OBB(obb));	
		} 
		
		emit addDebugText("Computing OBB done.");
		emit estimateOBBsCompleted(m_OBBs);
		emit estimatePartsDone(parts);
	}
}

void PCAThread::setPhase(PHASE phase)
{
	m_phase = phase;
}