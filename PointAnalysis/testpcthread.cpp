#include "testpcthread.h"

TestPCThread::TestPCThread(QString name, QObject *parent)
	: QThread(parent), modelLoaded(false)
{
	pcname = name;
}

TestPCThread::TestPCThread(QObject *parent)
	: QThread(parent), modelLoaded(false)
{

}

TestPCThread::~TestPCThread()
{
	if (isRunning())
		terminate();
}

void TestPCThread::run()
{
	initializeClassifier();
	test();
}

using namespace std;
using namespace shark;

void TestPCThread::test()
{
	/* Test each point cloud with the random forest model */
	emit addDebugText("Testing the point cloud...");
	QVector<QMap<int, float>> predictions;
	
	/* Load point cloud features from file*/
	int numOfPoints = loadTestPoints();
	predictions.resize(numOfPoints);

	if (dataTest.numberOfBatches() > 0){
		emit reportStatus("Testing " + pcname + "...");
		emit addDebugText("Testing " + pcname + "...");

		Data<RealVector> prediction = rfmodel(dataTest);
		QString predict_file = "../data/predictions/" + pcname + ".csv";
		exportCSV(prediction, predict_file.toStdString());
		emit addDebugText("Testing done.");

		/* Extract labels for each test data point */
		emit reportStatus("Extracting labels for data points in " + pcname + "...");
		emit addDebugText("Extracting labels for data points in " + pcname + "...");

		QVector<int> labels(dataTest.numberOfElements());
		Data<RealVector>::element_range elements = prediction.elements();
		int li = 0;
		for (PREDICT_Elements::iterator pos = elements.begin(); pos != elements.end(); ++pos)
		{
			RealVector distribution = *pos;
			/* Find the index with biggest probability value as the label of the point */
			int idx = 0;
			double max = -1;
			for (int j = 0; j < distribution.size(); j++)
			{
				predictions[li].insert(j, distribution[j]);    /* Set the probability of part label j for point-li */
				if (distribution[j] > max)
				{
					idx = j;
					max = distribution[j];
				}
			}
			/* Set the label to labels vector */
			labels[li++] = idx;
		}
		emit setPCLabels(labels);
		emit classifyProbabilityDistribution(predictions);
		emit addDebugText("Testing point clouds done.");
	}
	else
	{
		emit reportStatus("Test Point cloud is empty.");
		emit addDebugText("Test Point cloud is empty.");
	}
}

int TestPCThread::loadTestPoints()
{
	QString featpath = "../data/features_test/" + pcname + ".csv";
	emit reportStatus("Import " + featpath + " as test dataset.");
	emit addDebugText("Import test data from " + featpath + ".");
	importCSV(dataTest, featpath.toStdString());
	emit addDebugText("Test points loading done.");
	return dataTest.numberOfElements();
}

//QString TestPCThread::getModelName(int index)
//{
//	QStringList namelist = filenamelist.at(index).split("\\");
//	int size = namelist.size();
//	QString modelname = namelist[size - 1].section('.', 0, 0);
//	return modelname;
//}

void TestPCThread::setPcName(QString name)
{
	pcname = name;
}

void TestPCThread::initializeClassifier()
{
	if (!modelLoaded)
	{
	/* Load random forest model from model file */
		emit addDebugText("Loading random forest model from model file...");
		emit reportStatus("Loading random forest model from model file...");

		ifstream ifs("../data/classifier/original_coseg_chairs_rfmodel.model");
		boost::archive::polymorphic_text_iarchive ia(ifs);
		rfmodel.read(ia);
		ifs.close();

		modelLoaded = true;

		emit addDebugText("Random forest load done.");
	}
}
	