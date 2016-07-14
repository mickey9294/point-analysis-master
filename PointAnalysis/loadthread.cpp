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
	PCModel *model;
	if (m_phase == PHASE::TRAINING)
		model = Utils::loadPointCloud_CGAL_SDF(filename.c_str());
	else
	{
		model = Utils::loadPointCloud_CGAL(filename.c_str());
		//model = Utils::loadPointCloud(filename.c_str());
	}
	emit loadPointsCompleted(model);
}

void LoadThread::setPhase(PHASE phase)
{
	m_phase = phase;
}