#include "normalizethread.h"

NormalizeThread::NormalizeThread(QObject *parent)
	: QThread(parent)
{
	m_modelClassName = "coseg_chairs_3";
}

NormalizeThread::NormalizeThread(std::string modelClassName, QObject *parent)
	: QThread(parent), m_modelClassName(modelClassName)
{

}

NormalizeThread::~NormalizeThread()
{
	if (isRunning())
		terminate();
}

using namespace std;

void NormalizeThread::run()
{
	ifstream in;
	std::string normalized_filelist_path;
	ofstream normalized_filelist_out;

	std::string path = "../data/" + m_modelClassName + "_list.txt";
	in.open(path.c_str());
	normalized_filelist_path = "../data/" + m_modelClassName + "_normalized.txt";
	normalized_filelist_out.open(normalized_filelist_path.c_str());

	if (in.is_open())
	{
		char buffer[128];
		while (!in.eof())
		{
			in.getline(buffer, 128);
			if (strlen(buffer) > 0)
			{
				normalize(buffer);
				string modelname = Utils::getModelName(QString(buffer)).toStdString();
				string normalized_file = "../off_normalized/" + m_modelClassName + "/" + modelname + ".off";
				normalized_filelist_out << normalized_file << endl;
			}
		}

		in.close();
		normalized_filelist_out.close();
	}

}

void NormalizeThread::normalize(const char *filename)
{
	emit addDebugText("Normalizing " + QString(filename) + "...");

	ifstream off_in(filename);
	if (off_in.is_open())
	{
		const int d = 3;
		char buffer[128];
		PointVector S;
		vector<double> coords(d);

		off_in.getline(buffer, 128);
		off_in.getline(buffer, 128);
		QString line(buffer);
		int nvertices = line.section(' ', 0, 0).toInt();
		int nfaces = line.section(' ', 1, 1).toInt();

		for (int i = 0; i < nvertices; i++)
		{
			off_in.getline(buffer, 128);
			line = QString(buffer);
			float x = line.section(' ', 0, 0).toFloat();
			float y = line.section(' ', 1, 1).toFloat();
			float z = line.section(' ', 2, 2).toFloat();

			coords[0] = x;
			coords[1] = y;
			coords[2] = z;
			S.push_back(MiniPoint(d, coords.begin()));
		}

		/* Compute the Miniball of point cloud */
		Miniball mb(d, S);
		double rad = mb.radius();
		Miniball::Coordinate_iterator center_it = mb.center_begin();
		QVector3D mbs_center(center_it[0], center_it[1], center_it[2]);
		qDebug() << "Radius =" << rad
			<< "; Center :(" << mbs_center.x() << "," << mbs_center.y() << "," << mbs_center.z() << ")";

		/* Output vertices after normalization */
		string classname = m_modelClassName.length() < 1 ? "coseg_chairs_3" : m_modelClassName;
		QString output_filename = "../data/off_normalized/" + QString::fromStdString(classname) + "/" + QString(filename).section('\\', -1, -1);
		ofstream off_out(output_filename.toStdString().c_str());

		off_out << "OFF" << endl;
		off_out << to_string(nvertices) << " " << to_string(nfaces) << " 0" << endl;

		for (int i = 0; i < nvertices; i++)
		{
			MiniPoint point = S[i];
			float x = (point[0] - mbs_center.x()) / rad;
			float y = (point[1] - mbs_center.y()) / rad;
			float z = (point[2] - mbs_center.z()) / rad;

			off_out << to_string(x) << " " << to_string(y) << " " << to_string(z) << endl;
		}

		for (int i = 0; i < nfaces; i++)
		{
			off_in.getline(buffer, 128);
			off_out << buffer << endl;
		}

		off_out.close();

		off_in.close();

		emit addDebugText("Normalize done.");
		return;
	}
	emit addDebugText("Normalize failed. Could not open " + QString(filename) + ".");
}