#include "sdfthread.h"

SdfThread::SdfThread(QObject *parent)
	: QObject(parent)
{
	m_modelClassName = "coseg_chairs_3";
}

SdfThread::SdfThread(std::string modelClassName, QObject *parent)
	: QObject(parent), m_modelClassName(modelClassName)
{

}

SdfThread::~SdfThread()
{
	clean();
}

void SdfThread::clean()
{
	for (int i = 0; i < subthreads.size(); i++)
	{
		if (subthreads[i] != NULL)
		{
			if (subthreads[i]->isRunning())
				subthreads[i]->terminate();
			delete(subthreads[i]);
			subthreads[i] = NULL;
		}
	}

	subthreads.clear();
	m_filelist.clear();
	finish_count = 0;
}

using namespace std;
void SdfThread::execute()
{
	clean();

	emit addDebugText("Computing sdf values...");
	ifstream in;
	string sdf_filelist_path;
	ofstream sdf_filelist_out;
	if (m_modelClassName.length() < 1)
	{
		in.open("../data/coseg_chairs_3_normalized.txt");
		sdf_filelist_path = "../data/coseg_chairs_3_sdflist.txt";
		sdf_filelist_out.open(sdf_filelist_path.c_str());
	}
	else
	{
		string file_list_path = "../data/" + m_modelClassName + "_normalized.txt";
		in.open(file_list_path.c_str());
		sdf_filelist_path = "../data/" + m_modelClassName + "_sdflist.txt";
		sdf_filelist_out.open(sdf_filelist_path.c_str());
	}
	if (in.is_open())
	{
		char buffer[128];
		int count = 0;
		while (!in.eof())
		{
			in.getline(buffer, 128);
			if (strlen(buffer) > 0)
			{
				QString filepath(buffer);
				QString modelname = Utils::getModelName(filepath);
				string sdf_file_path = "../data/sdf/" + m_modelClassName + "/" + modelname.toStdString() + ".sdff";
				m_filelist.append(filepath);
				sdf_filelist_out << sdf_file_path << endl;

				count++;
			}
		}

		/* Create 8 subthreads to compute sdf values of all training models */
		int one_part = count / NUM_OF_SUBTHREADS;
		int rounds = NUM_OF_SUBTHREADS < count ? NUM_OF_SUBTHREADS : count;
		emit addDebugText("Create " + QString::number(rounds) + " threads to compute sdf values.");
		subthreads.resize(rounds);
		finish_count = rounds;
		for (int i = 0; i < rounds; i++)
		{
			int start = i * one_part;
			int end = i == (rounds - 1) ? (count - 1) : ((i + 1) * one_part - 1);

			SdfSubThread *thread = new SdfSubThread(i, m_filelist, start, end, m_modelClassName, this);
			connect(thread, SIGNAL(addDebugText(QString)), this, SLOT(onDebugTextAdded(QString)));
			connect(thread, SIGNAL(computeSdfCompleted(int)), this, SLOT(onSubthreadFinished(int)));
			subthreads[i] = thread;
			thread->start();
		}

		in.close();
		sdf_filelist_out.close();
	}
}

void SdfThread::onDebugTextAdded(QString text)
{
	emit addDebugText(text);
}

void SdfThread::onSubthreadFinished(int id)
{
	emit addDebugText("SdfSubThread-" + QString::number(id) + " has finished.");
	qDebug("SdfSubThread-%d has finished.", id);
	if (subthreads[id]->isRunning())
		subthreads[id]->terminate();
	delete(subthreads[id]);
	subthreads[id] = NULL;

	finish_count--;
	if (finish_count == 0)
		emit computeSdfCompleted();
}