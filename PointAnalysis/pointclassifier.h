#ifndef POINTCLASSIFIER_H
#define POINTCLASSIFIER_H

#include <QObject>
#include <vector>
#include <QVector>
#include <string>
#include <iostream>
#include <qmap.h>
#include <shark/Data/Csv.h>
#include <shark/Data/Dataset.h> //importing the file
#include <shark/Algorithms/Trainers/RFTrainer.h> //the random forest trainer

typedef shark::Data<shark::RealVector>::element_range PREDICT_Elements;
typedef shark::Data<shark::RealVector>::const_element_reference PREDICT_ElementRef;

class PointClassifier : public QObject
{
	Q_OBJECT

public:
	PointClassifier(QObject *parent = 0);
	~PointClassifier();

	void setLabelNames(const std::vector<int> & label_names);
	void setModelClassName(const std::string modelClassName);
	void testPointCloud(const std::string & pc_name, QVector<int> & pc_labels, QVector<QMap<int, float>> &distribution);

signals:
	void setPCLabels(QVector<int> labels);

private:
	bool m_classifier_loaded;
	std::string m_modelClassName;
	shark::RFClassifier m_rfmodel;
	std::vector<int> m_label_names;

	void initClassifier();
	void loadPredictionFromFile(std::string prediction_path, QVector<int> & pc_labels, QVector<QMap<int, float>> &distribution);
	void savePredictions(std::string path, const QVector<QMap<int, float>> & distributions);
	void loadTestPoints(std::string pc_name, shark::Data<shark::RealVector> &dataTest);
};

#endif // POINTCLASSIFIER_H
