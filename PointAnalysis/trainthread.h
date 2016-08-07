#ifndef TRAINTHREAD_H
#define TRAINTHREAD_H

#include <QThread>
#include <QDebug>
#include <qstring.h>
#include <qstringlist.h>
#include <shark/Data/Csv.h>
#include <shark/Data/Dataset.h> //importing the file
#include <shark/Algorithms/Trainers/RFTrainer.h> //the random forest trainer
#include <shark/ObjectiveFunctions/Loss/ZeroOneLoss.h> //zero one loss for evaluation
#include <fstream>
#include <string>

class TrainThread : public QThread
{
	Q_OBJECT

public:
	TrainThread(QObject *parent = 0);
	TrainThread(std::string modelClassName, QObject *parent = 0);
	~TrainThread();

signals:
	void reportStatus(QString stat);
	void addDebugText(QString text);

protected:
	void run();


private:
	shark::ClassificationDataset data;
	std::string m_modelClassName;

	void loadPoints();
	void readFeatures(QString featFilename, int mode);
	void train();
};

#endif // TRAINTHREAD_H
