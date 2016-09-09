#include "pointclassifier.h"

PointClassifier::PointClassifier(QObject *parent)
	: QObject(parent), m_classifier_loaded(false)
{
	m_modelClassName = "coseg_chairs_8";
}

PointClassifier::~PointClassifier()
{

}

void PointClassifier::setLabelNames(const std::vector<int> & label_names)
{
	m_label_names.resize(label_names.size());
	int idx = 0;
	for (std::vector<int>::const_iterator it = label_names.begin(); it != label_names.end(); ++it)
		m_label_names[idx++] = *it;
}

using namespace std;

void PointClassifier::initClassifier()
{
	if (!m_classifier_loaded)
	{
		/* Load random forest model from model file */
		std::cout << "Loading random forest model from model file..." << std::endl;

		std::string m_rfmodel_path = "../data/classifier/" + m_modelClassName + "_rfmodel.model";
		ifstream ifs(m_rfmodel_path.c_str());
		boost::archive::polymorphic_text_iarchive ia(ifs);
		m_rfmodel.read(ia);
		ifs.close();

		m_classifier_loaded = true;

		cout << "Random forest load done." << endl;
	}
}

void PointClassifier::setModelClassName(const std::string modelClassName)
{
	m_modelClassName = modelClassName;
}

void PointClassifier::testPointCloud(const std::string & pc_name, QVector<int> & pc_labels, QVector<QMap<int, float>> &distributions)
{
	using namespace shark;

	std::string prediction_path = "../data/predictions/" + pc_name + ".txt";

	/* Check whether the point cloud has been tested */
	ifstream pred_in(prediction_path.c_str());
	if (pred_in.is_open())  /* The point cloud has been tested */
	{
		cout << "Load predictions from file." << endl;
		loadPredictionFromFile(prediction_path, pc_labels, distributions);
	}
	else
	{
		if (!m_classifier_loaded)
			initClassifier();

		shark::Data<shark::RealVector> dataTest;
		loadTestPoints(pc_name, dataTest);

		if (dataTest.numberOfBatches() > 0){
			cout << "Testing point cloud " << pc_name << "..." << endl;

			Data<RealVector> prediction = m_rfmodel(dataTest);
			std::string predict_file = "../data/predictions/" + pc_name + ".csv";
			exportCSV(prediction, predict_file);
			cout << "Testing done." << endl;

			/* Extract labels for each test data point */
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
					distributions[li].insert(m_label_names[j], distribution[j]);    /* Set the probability of part label j for point-li */
					if (distribution[j] > max)
					{
						idx = j;
						max = distribution[j];
					}
				}

				/* Add a null part and set its the probability to 0.1 */
				distributions[li].insert(m_label_names.back() + 1, 0.1);

				/* Set the label to labels vector */
				labels[li++] = m_label_names[idx];
			}

			std::string out_path = "D:\\Projects\\point-analysis-master\\data\\predictions\\" + pc_name + ".txt";
			savePredictions(out_path, distributions);
			cout << "Testing point clouds done." << endl;
		}
		else
			cout << "Test Point cloud is empty." << endl;

	}

}

void PointClassifier::loadTestPoints(std::string pc_name, shark::Data<shark::RealVector> &dataTest)
{
	std::string featpath = "../data/features_test/" + pc_name + ".csv";
	cout << "Import test data from " << featpath << "." << endl;
	importCSV(dataTest, featpath);
	cout << "Test points loading done." << endl;
}

void PointClassifier::loadPredictionFromFile(std::string prediction_path, QVector<int> & labels, QVector<QMap<int, float>> &predictions)
{
	ifstream in(prediction_path.c_str());

	if (in.is_open())
	{
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

		in.close();
	}
}

void PointClassifier::savePredictions(std::string path, const QVector<QMap<int, float>> & distributions)
{
	std::ofstream out(path.c_str());

	if (out.is_open())
	{
		for (QVector<QMap<int, float>>::const_iterator point_it = distributions.begin(); point_it != distributions.end(); ++point_it)
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