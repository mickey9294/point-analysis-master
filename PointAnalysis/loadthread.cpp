#include "loadthread.h"

LoadThread::LoadThread(QObject *parent)
	: QThread(parent)
{
	m_gap_time = 0;
}

LoadThread::LoadThread(std::string name, PHASE phase, QObject *parent)
	: QThread(parent)
{
	filename = name;
	m_phase = phase;
	m_gap_time = 0;
}

LoadThread::~LoadThread()
{
	if (isRunning())
		terminate();
}

void LoadThread::setLoadFileName(std::string name)
{
	filename = name;
}

void LoadThread::run()
{
	QString msg = "Loading points from " + QString::fromStdString(filename) + "...";
	emit addDebugText(msg);
	loadPointCloud();
	qDebug() << "Load points done.";
	emit addDebugText("Load points done.");
	sleep(m_gap_time);
}

using namespace std;
void LoadThread::loadPointCloud()
{
	Model *model;

	/* Decide whether the model is a training mesh or a testing point cloud */
	string segment_filepath = Utils::getSegFilename(filename);
	ifstream seg_in(segment_filepath.c_str());
	if (seg_in.is_open())  /* If there exists a segmentation file for the model, then the model is a training mesh */
	{
		seg_in.close();
		model = new MeshModel(filename);
	}
	else  /* If there not exists a segmentation file, then the model is a testing point cloud or a mesh pointcloud */
	{
		/* Decide whether the model is a testing point cloud or a mesh pointcloud */
		ifstream off_in(filename.c_str());
		if (off_in.is_open())
		{
			char buffer[64];
			off_in.getline(buffer, 64);
			off_in.getline(buffer, 64);
			QString line(buffer);

			int nfaces = line.section(' ', 1, 1).toInt();

			if (nfaces > 0)
				model = new MeshPcModel(filename);
			else
				model = new PCModel(filename, 0);

			off_in.close();
		}
	}
	
	emit loadPointsCompleted(model);
}

void LoadThread::setPhase(PHASE phase)
{
	m_phase = phase;
}

void LoadThread::setGapTime(int time)
{
	m_gap_time = time;
}