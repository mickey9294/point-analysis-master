#ifndef OPENNIPROCESSOR_H
#define OPENNIPROCESSOR_H

#include <QThread>
#include "kinect2_grabber.h"
#include "pcmodel.h"

typedef pcl::PointXYZRGBA PointType;

class OpenNIProcessor : public QThread
{
	Q_OBJECT

public:
	OpenNIProcessor(QObject *parent);
	~OpenNIProcessor();

signals:
	void sendModel(Model *model);

protected:
	void run();

private:
	bool is_stopped;

	void createModel(pcl::PointCloud<PointType>::ConstPtr cloud);
};

#endif // OPENNIPROCESSOR_H
