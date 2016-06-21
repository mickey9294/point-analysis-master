#include "trainthread.h"

TrainThread::TrainThread(QObject *parent)
	: QThread(parent)
{
	m_modelClassName = "coseg_chairs_3";
}

TrainThread::TrainThread(std::string modelClassName, QObject *parent)
	: QThread(parent), m_modelClassName(modelClassName)
{

}

TrainThread::~TrainThread()
{
	if (isRunning())
		terminate();
}

void TrainThread::run()
{
	loadPoints();
	//loadTestPoints();
	train();
}

using namespace std;
using namespace shark;
void TrainThread::loadPoints()
{
	qDebug() << "Loading points features from features files...";
	emit addDebugText("Loading points features from features files...");
	emit reportStatus("Loading points features from features files...");

	string list_file_path = "../data/" + m_modelClassName + "_featslist.txt";
	ifstream in(list_file_path.c_str());
	if (in.is_open())
	{
		const int BUFFER_SIZE = 256;
		char buffer[BUFFER_SIZE];
		
		while (!in.eof())
		{
			in.getline(buffer, BUFFER_SIZE);
			QString feat_str(buffer);
			
			if (feat_str.length() > 10)
				readFeatures(feat_str, 0);
		}

		in.close();
	}

	QString stat_msg = "Loading training points done. The training dataset has "
		+ QString::number(data.numberOfElements()) + " elements, " + QString::number(data.numberOfBatches()) + " batches, "
		+ QString::number(numberOfClasses(data)) + " classes.";
	emit reportStatus(stat_msg);
	emit addDebugText(stat_msg);
	//sleep(3000);
}

void TrainThread::loadTestPoints()
{
	qDebug() << "Loading test points...";
	emit reportStatus("Loading test points...");
	emit addDebugText("Loading test points...");

	ifstream in("../data/TestList.txt");
	if (in.is_open())
	{
		const int BUFFER_SIZE = 256;
		char buffer[BUFFER_SIZE];

		while (!in.eof())
		{
			in.getline(buffer, BUFFER_SIZE);
			QString feat_str(buffer);

			if (feat_str.length() > 10)
				readFeatures(feat_str, 1);
		}

		in.close();
	}

	emit addDebugText("Loading test points done.");
}

void TrainThread::readFeatures(QString featFilename, int mode)
{ 
	QString stat_msg = "Reading features from " + featFilename + "...";
	qDebug() << stat_msg;
	emit reportStatus(stat_msg);
	emit addDebugText(stat_msg);

	const int DIMEN = 27;

	ClassificationDataset set;
 	importCSV(set, featFilename.toStdString().c_str(), LAST_COLUMN);

	if (mode == 0){    /* Load the training data */
		data.append(set);
		stat_msg = "Reading done.";
		qDebug() << stat_msg;
		emit addDebugText(stat_msg);
	}
	else    /* Load the test data */
	{
		dataTest.append(set);
		stat_msg = "Reading done.";
		qDebug() << stat_msg;
		emit addDebugText(stat_msg);
	}
}

std::string TrainThread::getModelName(QString filename)
{
	return "ab";
}

void TrainThread::train()
{
	/* Train the random forest model */
	RFTrainer trainer;
	RFClassifier model;
	qDebug() << "Training the random forest model...";
	emit reportStatus("Training the random forest model...");
	emit addDebugText("Training the random forest model...");
	//trainer.setNTrees(200);
	trainer.train(model, data);
	qDebug() << "Training done.";
	emit addDebugText("Training done.");

	
	/* Save the model into file */
	qDebug() << "Saving the classifier into file...";
	emit reportStatus("Saving the classifier into file...");
	emit addDebugText("Saving the classifier into file...");

	string rfmodel_path = "../data/classifier/" + m_modelClassName + "_rfmodel.model";
	ofstream fout(rfmodel_path.c_str());
	boost::archive::polymorphic_text_oarchive oa(fout);
	model.write(oa);
	fout.close();
	qDebug() << "Saving done.";
	emit reportStatus("Saving done.");
	emit addDebugText("Saving done.");
}