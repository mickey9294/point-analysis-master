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
#include "model.h"
#include "meshmodel.h"

class PointFeatureExtractor : public QObject
{
	Q_OBJECT

public:
	PointFeatureExtractor(QObject *parent = 0);
	PointFeatureExtractor(std::string modelClassName, QObject *parent = 0);
	~PointFeatureExtractor();
	void execute();

	public slots:
	void receiveModel(Model *model);
	void oneEstimateCompleted(PAPointCloud *cloud);
	void onDebugTextAdded(QString text);

signals:
	void reportStatus(QString stat);
	void showModel(Model *);
	void addDebugText(QString text);

private:
	std::vector<std::string> fileList;
	FeatureEstimator fe;
	int currentId;
	LoadThread loadThread;
	std::string m_modelClassName;
	QList<int> m_label_names;
	QList<std::string> m_features_filepaths;

	void estimateFeatures();
	std::string getOutFilename(int index);
	QString getModelName(int index);
};

#endif // POINTFEATUREEXTRACTOR_H
