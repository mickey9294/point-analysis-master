#include "outputthread.h"

OutputThread::OutputThread(QObject *parent)
	: QThread(parent)
{

}

OutputThread::~OutputThread()
{
	if (isRunning())
		terminate();
}

void OutputThread::run()
{
	qDebug() << "Outputing the original point cloud...";
	pcModel->output(outputFileName.c_str());
	emit(outputCompleted());
}

void OutputThread::setOutputModel(PCModel *outmodel, std::string filen)
{
	pcModel = outmodel;
	outputFileName = filen;
}
