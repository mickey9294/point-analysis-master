#ifndef ENERGYFUNCTIONS_H
#define ENERGYFUNCTIONS_H

#include <qvector.h>
#include <qmap.h>
#include <string>
#include <cmath>
#include "gencandidatesthread.h"

class EnergyFunctions
{
public:
	EnergyFunctions();
	~EnergyFunctions();

	/* Epnt
	 * The function computing the point classification energy.
	 * Input: distributions - points classifier probability distribution.
	 * Output: parts_energies - a map storing label-energy value pairs.
	 */
	static void Epnt(QVector<QMap<int, float>> distributions, QMap<int, double> &parts_energies);
	/* Epair
	 * The function computing the part relations energy.
	 * Input: 
	 */
	static double Epair(Part_Candidates part_candidates, std::string modelClassName);
};

#endif
