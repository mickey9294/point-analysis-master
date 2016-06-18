#ifndef SDFSUBTHREAD_H
#define SDFSUBTHREAD_H

#include <QThread>
#include <QStringList>
#include <fstream>
#include <string>
#include "utils.h"
class SdfSubThread : public QThread
{
	Q_OBJECT

public:
	SdfSubThread(int id, QStringList list, int start, int end, std::string modelClassName, QObject *parent);
	~SdfSubThread();
	
signals:
	void addDebugText(QString text);
	void computeSdfCompleted(int id);

protected:
	void run();

private:
	QStringList m_filelist;
	int m_start;
	int m_end;
	int m_id;
	std::string m_modelClassName;

	void compute_sdf(const char *filename);
};

#endif // SDFSUBTHREAD_H
