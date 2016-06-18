#include "predictionthread.h"

PredictionThread::PredictionThread(QObject *parent)
	: QThread(parent)
{

}

PredictionThread::PredictionThread(int ncandidates, Part_Candidates part_candidates, QVector<QMap<int, float>> distribution, QObject *parent)
	: QThread(parent)
{
	m_ncandidates = ncandidates;
	m_part_candidates = Part_Candidates(part_candidates);
	m_distribution = QVector<QMap<int, float>>(distribution);
}

PredictionThread::~PredictionThread()
{

}

void PredictionThread::run()
{

}

void PredictionThread::predictLabelsAndOrientations()
{
	MRFEnergy<TypePotts>* mrf;
	MRFEnergy<TypePotts>::NodeId* nodes;
	MRFEnergy<TypePotts>::Options options;
	TypePotts::REAL energy, lowerBound;

	const int nodeNum = m_ncandidates;   /* the number of nodes */
	const int labelNum = 9;   /* the number of labels */

	TypePotts::REAL D[labelNum];
	
	mrf = new MRFEnergy<TypePotts>(TypePotts::GlobalSize(labelNum));
	nodes = new MRFEnergy<TypePotts>::NodeId[nodeNum];

	/* Construct energy */

}