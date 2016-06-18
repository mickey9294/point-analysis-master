#ifndef NORMALIZETHREAD_H
#define NORMALIZETHREAD_H

#include <QThread>
#include <fstream>
#include <iostream>
#include <string>
#include <qdebug.h>
#include <QVector3D>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include "Seb.h"
#include "utils.h"

typedef Seb::Point<float> MiniPoint;
typedef std::vector<MiniPoint> PointVector;
typedef Seb::Smallest_enclosing_ball<float> Miniball;

class NormalizeThread : public QThread
{
	Q_OBJECT

public:
	NormalizeThread(QObject *parent);
	NormalizeThread(std::string modelClassName, QObject *parent);
	~NormalizeThread();

signals:
	void addDebugText(QString text);

protected:
	void run();

private:
	std::string m_modelClassName;

	void normalize(const char *filename);
};

#endif // NORMALIZETHREAD_H
