#ifndef PARTSPREDICTOR_H
#define PARTSPREDICTOR_H

#include <QObject>
#include <qlist.h>
#include <qvector.h>
#include <qset.h>
#include <qmap.h>
#include <QSharedPointer>
#include <MRFEnergy.h>
#include <QtAlgorithms>
#include <assert.h>
#include "utils.h"
#include "energyfunctions.h"

class PartsPredictor : public QObject
{
	Q_OBJECT

public:
	PartsPredictor(QObject *parent = 0);
	~PartsPredictor();

	void setUseSymmetry(bool use_symmetry);
	void setModelClassName(std::string modelClassName);

	void predictLabelsAndOrientations(const Part_Candidates & part_candidates, const QVector<int> & label_names, 
		QMap<int, int> &parts_picked, std::vector<int> &candidate_labels, QSharedPointer<EnergyFunctions> energy_functions);

private:
	bool m_use_symmetry;
	std::string m_modelClassName;

	void generateCombinations(const QMap<int, QList<int>> & winners,
		const QList<int> & labels_with_multi_winners_set, QList<QMap<int, int>> & combinations);
	void recurGenerateCombinations(const QVector<QList<int>> & multi_winners, const QList<int> & current, 
		int level, QList<QList<int>> & output);
	void selectMostOptimized(const QList<QMap<int, int>> & combinations, QMap<int, int> & most_optimized, 
		const Part_Candidates & part_candidates, QSharedPointer<EnergyFunctions> energy_functions);
	double computeEnergy(const QMap<int, int> & combination, const Part_Candidates & part_candidates, QSharedPointer<EnergyFunctions> energy_functions);
};

#endif // PARTSPREDICTOR_H
