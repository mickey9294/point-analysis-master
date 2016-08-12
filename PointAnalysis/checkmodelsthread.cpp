#include "checkmodelsthread.h"

CheckModelsThread::CheckModelsThread(QObject *parent)
	: QObject(parent), loadThread(NULL), current_no(0)
{

}

CheckModelsThread::~CheckModelsThread()
{
	clear();
}

using namespace std;

void CheckModelsThread::execute()
{
	clear();

	char file_list_path[] = "../data/coseg_chairs_8_list.txt";

	ifstream list_in(file_list_path);

	if (list_in.is_open())
	{
		char buffer[64];

		while (!list_in.eof())
		{
			list_in.getline(buffer, 64);
			if (strlen(buffer) > 0)
			{
				model_paths.push_back(string(buffer));
			}
		}
	}

	loadThread = new LoadThread(model_paths[current_no], PHASE::TRAINING, this);
	connect(loadThread, SIGNAL(loadPointsCompleted(Model *)), this, SLOT(receiveModel(Model *)));
	connect(loadThread, SIGNAL(addDebugText(QString)), this, SLOT(onDebugTextAdded(QString)));
	loadThread->setGapTime(5000);
	loadThread->start();
}

void CheckModelsThread::receiveModel(Model *model)
{
	emit showModel(model);
	current_no++;

	if (current_no < model_paths.size())
	{
		if (loadThread->isRunning())
			loadThread->terminate();

		loadThread->setLoadFileName(model_paths[current_no]);
		loadThread->start();
	}
	else
	{
		clear();
		emit finish();
	}
}

void CheckModelsThread::onDebugTextAdded(QString text)
{
	emit addDebugText(text);
}

void CheckModelsThread::clear()
{
	if (loadThread != NULL)
	{
		if (loadThread->isRunning())
			loadThread->terminate();
		delete(loadThread);
		loadThread = NULL;
	}

	model_paths.clear();

	current_no = 0;
}