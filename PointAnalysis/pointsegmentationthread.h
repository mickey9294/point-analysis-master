#ifndef POINTSEGMENTATIONTHREAD_H
#define POINTSEGMENTATIONTHREAD_H

#include <QThread>
#include <iostream>
#include <MRFEnergy.h>
#include "PartsStructure.h"

class PointSegmentationThread : public QThread
{
	Q_OBJECT

public:
	PointSegmentationThread(PartsStructure *parts_structure, QVector<int> label_names, QObject *parent = 0);
	~PointSegmentationThread();

signals:
	void pointSegmentationDone(QVector<int> new_point_assignments);

protected:
	void run();

private:
	QVector<int> m_label_names;
	PartsStructure * m_parts_structure;

	//int optimize();    /* Return the number of labels that have changed */

	int segment_sample_points();
};

#endif // POINTSEGMENTATIONTHREAD_H
