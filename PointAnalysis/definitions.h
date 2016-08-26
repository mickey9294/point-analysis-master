#ifndef DEFINITIONS_H
#define DEFINITIONS_H

#include <QVector>
#include <QMap>
#include "papart.h"

typedef QVector<PAPart *> Parts_Vector;
Q_DECLARE_METATYPE(Parts_Vector)

typedef QVector<PAPart> Part_Candidates;
Q_DECLARE_METATYPE(Part_Candidates)

typedef unsigned int LabelIndex;
typedef unsigned int SamplePointIndex;
typedef double Real;
typedef int Label;

#endif