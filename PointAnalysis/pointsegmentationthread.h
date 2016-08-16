#ifndef POINTSEGMENTATIONTHREAD_H
#define POINTSEGMENTATIONTHREAD_H

#include <QThread>
#include <iostream>
#include <MRFEnergy.h>
#include "PAPointCloud.h"
#include "energyfunctions.h"

class PointSegmentationThread : public QThread
{
	Q_OBJECT

public:
	PointSegmentationThread(PAPointCloud *pointcloud, QVector<int> label_names, EnergyFunctions *energy_functions,
		QObject *parent = 0);
	~PointSegmentationThread();

signals:
	void pointSegmentationDone(QVector<int> new_point_assignments);

protected:
	void run();

private:
	EnergyFunctions *m_energy_functions;
	PAPointCloud *m_pointcloud;
	QVector<int> m_label_names;

	int optimize();    /* Return the number of labels that have changed */
};

#endif // POINTSEGMENTATIONTHREAD_H
