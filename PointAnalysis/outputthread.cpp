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
	model->output(outputFileName.c_str());
	emit(outputCompleted());
}

void OutputThread::setOutputModel(Model *outmodel, std::string filen)
{
	model = outmodel;
	outputFileName = filen;
}
