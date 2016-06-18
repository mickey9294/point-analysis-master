#ifndef SDFTHREAD_H
#define SDFTHREAD_H

#include <QThread>
#include "utils.h"
#include <fstream>
#include <string>
#include <QStringList>
#include "sdfsubthread.h"

#define NUM_OF_SUBTHREADS  1

class SdfThread : public QObject
{
	Q_OBJECT

public:
	SdfThread(QObject *parent = 0);
	SdfThread(std::string modelClassName, QObject *parent = 0);
	~SdfThread();
	void execute();

	public slots:
	void onDebugTextAdded(QString text); 
	void onSubthreadFinished(int id);

signals:
	void addDebugText(QString text);
	void computeSdfCompleted();

private:
	std::string m_modelClassName;
	QStringList m_filelist;
	QVector<SdfSubThread *> subthreads;
	int finish_count;

	void clean();
};

#endif // SDFTHREAD_H
