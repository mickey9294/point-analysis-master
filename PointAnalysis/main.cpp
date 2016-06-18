#include "pointanalysis.h"
#include <QtWidgets/QApplication>
#include "gencandidatesthread.h"

int main(int argc, char *argv[])
{
	qRegisterMetaType<Part_Candidates>("PartCandidates");
	QCoreApplication::addLibraryPath("./");
	QApplication a(argc, argv);
	PointAnalysis w;
	w.show();
	return a.exec();
}
