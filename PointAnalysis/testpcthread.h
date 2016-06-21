#ifndef TESTPCTHREAD_H
#define TESTPCTHREAD_H

#include <QThread>
#include <QVector>
#include <QStringList>
#include <QList>
#include <QMap>
#include <fstream>
#include <string>
#include <shark/Data/Csv.h>
#include <shark/Data/Dataset.h> //importing the file
#include <shark/Algorithms/Trainers/RFTrainer.h> //the random forest trainer

typedef shark::Data<shark::RealVector>::element_range PREDICT_Elements;
typedef shark::Data<shark::RealVector>::const_element_reference PREDICT_ElementRef;

class TestPCThread : public QThread
{
	Q_OBJECT

public:
	TestPCThread(QObject *parent = 0);
	TestPCThread(QString name, QObject *parent = 0);
	TestPCThread(QString name, std::string modelClassName, QObject *parent = 0);
	~TestPCThread();

	void setPcName(QString name);

signals:
	void addDebugText(QString text);
	void reportStatus(QString msg);
	void setPCLabels(QVector<int> labels);
	void classifyProbabilityDistribution(QVector<QMap<int, float>> distribution);

protected:
	void run();

private:
	shark::RFClassifier rfmodel;
	shark::Data<shark::RealVector> dataTest;
	QString pcname;
	bool modelLoaded;
	std::string m_modelClassName;

	void test();
	int loadTestPoints();
	void initializeClassifier();
	void loadLabelNames(QList<int> &label_names);
};

#endif // TESTPCTHREAD_H
