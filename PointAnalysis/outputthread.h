#ifndef OUTPUTTHREAD_H
#define OUTPUTTHREAD_H

#include <QThread>
#include <QDebug>
#include "pcmodel.h"
#include <string>

class OutputThread : public QThread
{
	Q_OBJECT

public:
	OutputThread(QObject *parent = 0);
	~OutputThread();
	void setOutputModel(PCModel *model, std::string filename);

signals:
	void outputCompleted();

protected:
	void run();

private:
	std::string outputFileName;
	PCModel *pcModel;
	
};

#endif // OUTPUTTHREAD_H
