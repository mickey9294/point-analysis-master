#ifndef CHECKMODELSTHREAD_H
#define CHECKMODELSTHREAD_H

#include <QObject>
#include "loadthread.h"
#include <iostream>
#include <fstream>
#include <QVector>
#include "model.h"
#include "meshmodel.h"
#include <string>
#include "utils.h"
#include <time.h>

struct FacetProb
{
	int index;		//the no. of triangle in the tri_coordIndex;
	double area;	//area of the triangle;
	FacetProb(void) {};
	FacetProb(const FacetProb &info)
	{
		this->index = info.index;
		this->area = info.area;
	}
	void SetInfo(const int &index, const double &area)
	{
		this->index = index;
		this->area = area;
	}
};

class CheckModelsThread : public QObject
{
	Q_OBJECT

public:
	CheckModelsThread(QObject *parent = 0);
	~CheckModelsThread();

	public slots:
	void receiveModel(Model * model);
	void onDebugTextAdded(QString text);

	void execute();

signals:
	void showModel(Model *model);
	void addDebugText(QString text);
	void finish();

private:
	LoadThread *loadThread;
	QVector<std::string> model_paths;
	int current_no;

	void clear();
	void sampleOnMesh(MeshModel * mesh);
	void setFaceProb(MeshModel *m, vector<FacetProb> &faceProbs);
	bool getRandomPoint(MeshModel *m, const vector<FacetProb> &faceProbs, 
		Eigen::Vector3f &v, Eigen::Vector3f & bary_coord, int & face_idx);
	double TriangleArea(const Eigen::Vector3f &pA, const Eigen::Vector3f &pB, const Eigen::Vector3f &pC);
};

#endif // CHECKMODELSTHREAD_H
