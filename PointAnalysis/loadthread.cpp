#include "loadthread.h"

LoadThread::LoadThread(QObject *parent)
	: QThread(parent)
{
	m_gap_time = 0;
}

LoadThread::LoadThread(std::string name, PHASE phase, QObject *parent)
	: QThread(parent)
{
	filename = name;
	m_phase = phase;
	m_gap_time = 0;
}

LoadThread::~LoadThread()
{
	if (isRunning())
		terminate();
}

void LoadThread::setLoadFileName(std::string name)
{
	filename = name;
}

void LoadThread::run()
{
	QString msg = "Loading points from " + QString::fromStdString(filename) + "...";
	emit addDebugText(msg);
	loadPointCloud();
	qDebug() << "Load points done.";
	emit addDebugText("Load points done.");
	sleep(m_gap_time);
}

using namespace std;
void LoadThread::loadPointCloud()
{
	Model *model;
	QVector<OBB *> obbs;

	/* Decide whether the model is a training mesh or a testing point cloud */
	boost::filesystem::path path(filename);
	std::string format = path.extension().string();

	if (format.compare(".pts") != 0)
	{
		string segment_filepath = Utils::getSegFilename(filename);
		ifstream seg_in(segment_filepath.c_str());
		if (seg_in.is_open())  /* If there exists a segmentation file for the model, then the model is a training mesh */
		{
			seg_in.close();
			model = new MeshModel(filename);
		}
		else  /* If there not exists a segmentation file, then the model is a testing point cloud or a mesh pointcloud */
		{
			/* Decide whether the model is a testing point cloud or a mesh pointcloud */
			ifstream off_in(filename.c_str());
			if (off_in.is_open())
			{
				char buffer[64];
				off_in.getline(buffer, 64);
				off_in.getline(buffer, 64);
				QString line(buffer);

				int nfaces = line.section(' ', 1, 1).toInt();

				if (nfaces > 0)
				{
					model = new MeshPcModel(filename);
				}
				else
				{
					model = new PCModel(filename, 0);

					/* Decide whether there exists a bounding boxexs file */
					QString q_file_path = QString::fromStdString(filename);
					char seperator = q_file_path.contains('/') ? '/' : '\\';
					QString q_file_dir = q_file_path.section(seperator, 0, -2);
					QString q_model_name = q_file_path.section(seperator, -1, -1);
					QString q_obbs_name = q_model_name.section('_', 0, 1) + ".arff";
					std::string obbs_path = q_file_dir.toStdString() + "/" + q_obbs_name.toStdString();


					loadOBBs(obbs_path.c_str(), obbs);

					Eigen::Vector3f pc_centroid = model->getCentroid();
					double pc_radius = model->getRadius();

					/*model->normalize();

					for (QVector<OBB *>::iterator obb_it = obbs.begin(); obb_it != obbs.end(); ++obb_it)
						(*obb_it)->normalize(pc_centroid, pc_radius);*/
				}

				off_in.close();
			}
		}
	}
	else
	{
		model = new PCModel(filename, 0);
	}
	
	emit loadPointsCompleted(model);
	if (obbs.size() > 0)
		emit sendOBBs(obbs);
}

void LoadThread::setPhase(PHASE phase)
{
	m_phase = phase;
}

void LoadThread::setGapTime(int time)
{
	m_gap_time = time;
}

void LoadThread::loadOBBs(const char *file_path, QVector<OBB *> & obbs)
{
	std::ifstream in(file_path);

	if (in.is_open())
	{
		char buffer[256];
		obbs.clear();
		obbs.reserve(6);

		while (!in.eof())
		{
			in.getline(buffer, 256);
			if (strlen(buffer) <= 0)
				break;
			else if (buffer[0] == '@')
				continue;

			QStringList line_list = QString(buffer).split(',');

			QStringList::iterator line_it = line_list.begin();

			int label = line_it->toInt();
			++line_it;

			Eigen::Matrix3f axes;
			for (int i = 0; i < 9 && line_it != line_list.end(); i++, line_it++)
				axes(i) = line_it->toFloat();

			Eigen::Vector3f centroid, scale;
			for (int i = 0; i < 3 && line_it != line_list.end(); i++, line_it++)
				centroid[i] = line_it->toFloat();
			for (int i = 0; i < 3 && line_it != line_list.end(); i++, line_it++)
				scale[i] = line_it->toFloat();

			OBB *obb = new OBB(axes.col(0), axes.col(1), axes.col(2), centroid, scale[0], scale[1], scale[2], label);
			obbs.push_back(obb);
		}

		in.close();
	}
}