#include "loadthread.h"

LoadThread::LoadThread(QObject *parent)
	: QThread(parent)
{

}

LoadThread::LoadThread(std::string name, PHASE phase, QObject *parent)
	: QThread(parent)
{
	filename = name;
	m_phase = phase;
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
	else  /* If there not exists a segmentation file, then the model is a testing point cloud */
	{
		model = new PCModel(filename, 0);
	}
	
	emit loadPointsCompleted(model);
}

void LoadThread::setPhase(PHASE phase)
{
	m_phase = phase;
}