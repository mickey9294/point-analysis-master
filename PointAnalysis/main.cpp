#include "pointanalysis.h"
#include <QtWidgets/QApplication>

int main(int argc, char *argv[])
{
	QCoreApplication::addLibraryPath("./");
	QApplication a(argc, argv);
	PointAnalysis w;
	w.show();
	return a.exec();
}