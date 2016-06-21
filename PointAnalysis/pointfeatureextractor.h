#ifndef POINTFEATUREEXTRACTOR_H
#define POINTFEATUREEXTRACTOR_H

#include <QThread>
#include <QDebug>
#include <qlist.h>
#include <QtAlgorithms>
#include <string>
#include <fstream>
#include "featureestimator.h"
#include "utils.h"
#include "PAPointCloud.h"
#include "loadthread.h"

class PointFeatureExtractor : public QObject
{
	Q_OBJECT

public:
	PointFeatureExtractor(QObject *parent = 0);
	PointFeatureExtractor(std::string modelClassName, QObject *parent = 0);
	~PointFeatureExtractor();
	void execute();

	public slots:
	void receiveModel(PCModel *model);
	void oneEstimateCompleted(PAPointCloud *cloud);
	void onDebugTextAdded(QString text);

signals:
	void reportStatus(QString stat);
	void showModel(PCModel *);
	void addDebugText(QString text);

private:
	std::vector<std::string> fileList;
	FeatureEstimator fe;
	int currentId;
	LoadThread loadThread;
	std::string m_modelClassName;
	QList<int> m_label_names;

	void estimateFeatures();
	std::string getOutFilename(int index);
	QString getModelName(int index);
};

#endif // POINTFEATUREEXTRACTOR_H
