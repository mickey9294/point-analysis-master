#include "testpcthread.h"

TestPCThread::TestPCThread(QString name, QObject *parent)
	: QThread(parent), modelLoaded(false)
{
	pcname = name;
	m_modelClassName = "coseg_chairs_8";
}

TestPCThread::TestPCThread(QObject *parent)
	: QThread(parent), modelLoaded(false)
{
	m_modelClassName = "coseg_chairs_8";
}

TestPCThread::TestPCThread(QString name, std::string modelClassName, QObject *parent)
	: QThread(parent), modelLoaded(false), m_modelClassName(modelClassName)
{
	pcname = name;
}

TestPCThread::TestPCThread(int flag, std::string prediction_path, std::string model_class_name, QObject *parent)
	: QThread(parent), m_prediction_path(prediction_path), modelLoaded(false), m_modelClassName(model_class_name)
{

}

TestPCThread::~TestPCThread()
{
	if (isRunning())
		terminate();
}

void TestPCThread::run()
{
	if (pcname.length() > 0)
	{
		initializeClassifier();
		test();
	}
	else
		loadPredictionFromFile();
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

	/* Load the label names from file */
	QList<int> label_names;
	loadLabelNames(label_names);

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
				predictions[li].insert(label_names[j], distribution[j]);    /* Set the probability of part label j for point-li */
				if (distribution[j] > max)
				{
					idx = j;
					max = distribution[j];
				}
			}

			/* Add a null part and set its the probability to 0.1 */
			predictions[li].insert(label_names.last() + 1, 0.1);

			/* Set the label to labels vector */
			labels[li++] = label_names[idx];
		}
		emit setPCLabels(labels);
		emit classifyProbabilityDistribution(predictions);
		std::string out_path = "D:\\Projects\\point-analysis-master\\data\\predictions\\" + pcname.toStdString() + ".txt";
		saveClassification(out_path, predictions);
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

void TestPCThread::loadLabelNames(QList<int> &label_names)
{
	string label_names_path = "../data/label_names/" + m_modelClassName + "_labelnames.txt";
	ifstream label_names_in(label_names_path.c_str());

	if (label_names_in.is_open())
	{
		char buffer[8];

		while (!label_names_in.eof())
		{
			label_names_in.getline(buffer, 8);

			if (strlen(buffer) > 0)
				label_names.push_back(atoi(buffer));
		}

		label_names_in.close();
	}
}

void TestPCThread::setPcName(QString name)
{
	if (isRunning())
		terminate();
	pcname = name;
}

void TestPCThread::setPredictionFilePath(std::string prediction_path)
{
	if (isRunning())
		terminate();

	m_prediction_path = prediction_path;
	pcname.clear();
}

void TestPCThread::initializeClassifier()
{
	if (!modelLoaded)
	{
	/* Load random forest model from model file */
		emit addDebugText("Loading random forest model from model file...");
		emit reportStatus("Loading random forest model from model file...");

		std::string rfmodel_path = "../data/classifier/" + m_modelClassName + "_rfmodel.model"; 
		ifstream ifs(rfmodel_path.c_str());
		boost::archive::polymorphic_text_iarchive ia(ifs);
		rfmodel.read(ia);
		ifs.close();

		modelLoaded = true;

		emit addDebugText("Random forest load done.");
	}
}
	
void TestPCThread::saveClassification(std::string path, QVector<QMap<int, float>> distributions)
{
	std::ofstream out(path.c_str());

	if (out.is_open())
	{
		for (QVector<QMap<int, float>>::iterator point_it = distributions.begin(); point_it != distributions.end(); ++point_it)
		{
			QMap<int, float> distribution = *point_it;
			QList<int> label_names = distribution.keys();
			int numLabels = label_names.size();
			int i = 0;
			for (QList<int>::iterator prob_it = label_names.begin(); prob_it != label_names.end() && i < numLabels - 1; ++prob_it, i++)
			{
				int label = *prob_it;
				float prob = distribution.value(label);
				out << label << "_" << prob << " ";
			}
			out << label_names.last() << "_" << distribution.last() << std::endl;
		}

		out.close();
	}
}

void TestPCThread::loadPredictionFromFile()
{
	std::ifstream in(m_prediction_path.c_str());

	if (in.is_open())
	{
		QVector<QMap<int, float>> predictions;
		QVector<int> labels;

		char buffer[128];

		while (!in.eof())
		{
			in.getline(buffer, 128);

			if (strlen(buffer) > 0)
			{
				QMap<int, float> prediction;
				
				float max_prob = 0;
				int max_label = 0;

				QStringList distribution_str = QString(buffer).split(' ');
				for (QStringList::iterator dis_it = distribution_str.begin(); dis_it != distribution_str.end(); ++dis_it)
				{
					QString prob_str = *dis_it;
					int label = prob_str.section('_', 0, 0).toInt();
					float prob = prob_str.section('_', 1, 1).toFloat();
					prediction.insert(label, prob);

					if (prob > max_prob)
					{
						max_prob = prob;
						max_label = label;
					}
				}
				labels.push_back(max_label);
				predictions.push_back(prediction);
			}
		}

		emit setPCLabels(labels);
		emit classifyProbabilityDistribution(predictions);
		emit addDebugText("Load prediction from file done.");

		in.close();
	}
}