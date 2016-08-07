#ifndef OUTPUTTHREAD_H
#define OUTPUTTHREAD_H

#include <QThread>
#include <QDebug>
#include "model.h"
#include <string>

class OutputThread : public QThread
{
	Q_OBJECT

public:
	OutputThread(QObject *parent = 0);
	~OutputThread();
	void setOutputModel(Model *model, std::string filename);

signals:
	void outputCompleted();

protected:
	void run();

private:
	std::string outputFileName;
	Model *model;
	
};

#endif // OUTPUTTHREAD_H
