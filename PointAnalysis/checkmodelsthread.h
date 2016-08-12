#ifndef CHECKMODELSTHREAD_H
#define CHECKMODELSTHREAD_H

#include <QObject>
#include "loadthread.h"
#include <iostream>
#include <fstream>
#include <QVector>
#include "model.h"
#include "meshmodel.h"
#include <string>
#include "utils.h"

class CheckModelsThread : public QObject
{
	Q_OBJECT

public:
	CheckModelsThread(QObject *parent = 0);
	~CheckModelsThread();

	public slots:
	void receiveModel(Model * model);
	void onDebugTextAdded(QString text);

	void execute();

signals:
	void showModel(Model *model);
	void addDebugText(QString text);
	void finish();

private:
	LoadThread *loadThread;
	QVector<std::string> model_paths;
	int current_no;

	void clear();
};

#endif // CHECKMODELSTHREAD_H
