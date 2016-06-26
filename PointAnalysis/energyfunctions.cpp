#include "energyfunctions.h"


EnergyFunctions::EnergyFunctions()
{
}


EnergyFunctions::~EnergyFunctions()
{
}

void EnergyFunctions::Epnt(QVector<QMap<int, float>> distributions,QMap<int, double> &parts_energies)
{
	QMap<int, int> npointsInPart;   /* A map storing the number of points in each particular part with the form of label-count pairs */

	/* Initialize npointInPart and parts_energies with all the parts */
	QMap<int, float> distribution0 = distributions.first();
	QMap<int, float>::iterator it;
	for (it = distribution0.begin(); it != distribution0.end(); ++it)
	{
		parts_energies.insert(it.key(), 0);
		npointsInPart.insert(it.key(), 0);
	}

	/* Deal with each point one by one */
	QVector<QMap<int, float>>::iterator point_it;
	for (point_it = distributions.begin(); point_it != distributions.end(); point_it++)
	{
		QMap<int, float> distribution = *point_it;
		/* Find the max part probability */
		float max_prob = 0;
		int max_label = 0;
		QMap<int, float>::iterator part_it;
		for (part_it = distribution.begin(); part_it != distribution.end(); ++part_it)
		{
			int label = part_it.key();
			float prob = part_it.value();
			if (prob > max_prob)
			{
				max_prob = prob;
				max_label = label;
			}
		}

		npointsInPart[max_label]++;
		double e = -std::log(max_prob);    /* -log w(p, lb) */
		parts_energies[max_label] += e;
	}

	/* Divide the energy of each part by the number of points in this part */
	QMap<int, double>::iterator energy_it;
	for (energy_it = parts_energies.begin(); energy_it != parts_energies.end(); ++energy_it)
	{
		int label = energy_it.key();
		parts_energies[label] /= (double)npointsInPart[label];
	}
}