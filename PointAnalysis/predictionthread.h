#ifndef PREDICTIONTHREAD_H
#define PREDICTIONTHREAD_H

#include <QThread>
#include <QMap>
#include <qvector.h>
#include <qsharedpointer.h>
#include <assert.h>
#include "definitions.h"
#include <MRFEnergy.h>
#include "energyfunctions.h"
#include <QtAlgorithms>

/*
 * The sub thread used to do  part lebels and orientations prediction. 
 */
class PredictionThread : public QThread
{
	Q_OBJECT

public:
	PredictionThread(QObject *parent = 0);
	PredictionThread(QSharedPointer<EnergyFunctions> energy_functions, Part_Candidates part_candidates,
		QVector<int> label_names, std::string modelClassName, bool use_symmetry, QObject *parent = 0);
	~PredictionThread();

signals:
	void predictionDone(QMap<int, int> parts_picked, std::vector<int> candidate_labels);
	//void predictionDone();
	//void testSignal();

protected:
	void run();

private:
	std::string m_modelClassName;
	Part_Candidates m_part_candidates;
	QVector<int> m_label_names;    /* The label set. Note that the label with the largest number is null label */
	int m_ncandidates;
	QSharedPointer<EnergyFunctions> m_energy_functions;
	bool m_is_clean;
	long start_time;
	long end_time;
	bool m_use_symmetry;

	void clean();
	void singleThreadOptimize();

	void generateCombinations(const QMap<int, QList<int>> & winners,
		const QList<int> & labels_with_multi_winners_set, QList<QMap<int, int>> & combinations);
	void recurGenerateCombinations(const QVector<QList<int>> & multi_winners, const QList<int> & current, int level, QList<QList<int>> & output);
	void selectMostOptimized(const QList<QMap<int, int>> & combinations, QMap<int, int> & most_optimized);
	double computeEnergy(const QMap<int, int> & combination);
};

#endif // PREDICTIONTHREAD_H
