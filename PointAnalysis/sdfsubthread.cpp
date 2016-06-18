#include "sdfsubthread.h"

SdfSubThread::SdfSubThread(int id, QStringList filelist, int start, int end, std::string modelClassName, QObject *parent)
	: QThread(parent)
{
	emit addDebugText("SdfSubThread-" + QString::number(id) + " is created.");
	qDebug("SdfSubThread-%d is created.", id);
	m_filelist = filelist;
	m_start = start;
	m_end = end;
	m_id = id;
	m_modelClassName = modelClassName;
}

SdfSubThread::~SdfSubThread()
{
	if (isRunning())
		terminate();
}

void SdfSubThread::run()
{
	for (int i = m_start; i <= m_end; i++)
	{
		emit addDebugText("SdfSubThread-" + QString::number(m_id) + ": computiong sdf of " + m_filelist[i] + "...");
		qDebug() << "SdfSubThread-" + QString::number(m_id) + ": computiong sdf of " + m_filelist[i] + "...";

		compute_sdf(m_filelist[i].toStdString().c_str());
	}

	emit computeSdfCompleted(m_id);
}

using namespace std;
void SdfSubThread::compute_sdf(const char *filename)
{
	/* create and read Polyhedron */
	Polyhedron mesh;
	std::ifstream input(filename);
	if (!input || !(input >> mesh) || mesh.empty())
	{
		std::cerr << "Not a valid off file." << std::endl;
		return;
	}
	/* create a property-map */
	typedef std::map<Polyhedron::Facet_const_handle, double> Facet_double_map;
	Facet_double_map internal_map;
	boost::associative_property_map<Facet_double_map> sdf_property_map(internal_map);

	const std::size_t number_of_rays = 100;    /* cast 100 rays per facet */
	const double cone_angle = 45.0 / 180.0 * CGAL_PI;   /* set cone opening-angle to 45 degrees */
	CGAL::sdf_values(mesh, sdf_property_map, cone_angle, number_of_rays, false);
	std::pair<double, double> min_max_sdf = CGAL::sdf_values_postprocessing(mesh, sdf_property_map);

	ifstream off_in(filename);  /* Used to read indices of vertices of each face */
	const int BUFFER_SIZE = 128;
	char buffer[BUFFER_SIZE];
	off_in.getline(buffer, BUFFER_SIZE);
	off_in.getline(buffer, BUFFER_SIZE);
	/* Read the number of vertices and faces from the header */
	QString line = QString(buffer);
	int nvertices = line.section(' ', 0, 0).toInt();
	int nfaces = line.section(' ', 1, 1).toInt();

	QVector<int> sdf_count(nvertices);    /* Each component represents the nubmer of sdf values added into the corresponding vertex */
	QVector<double> vertices_sdf(nvertices);    /* Sdf values vector, each of which stores the sdf value of corresponding vertex */

	for (int i = 0; i < nvertices; i++)
		off_in.getline(buffer, BUFFER_SIZE);    /* Skip the vertices part to read the faces part */

	/* Get the sdf value of each facet in the mesh */
	int count = 0;
	for (Polyhedron::Facet_const_iterator facet_it = mesh.facets_begin();
		facet_it != mesh.facets_end(); ++facet_it)
	{
		/* Read a face containing the indices of 3 vertieces */
		off_in.getline(buffer, BUFFER_SIZE);
		line = QString(buffer);
		int v0 = line.section(' ', 1, 1).toInt();
		int v1 = line.section(' ', 2, 2).toInt();
		int v2 = line.section(' ', 3, 3).toInt();

		sdf_count[v0]++;
		sdf_count[v1]++;
		sdf_count[v2]++;

		double sdf = sdf_property_map[facet_it];    /* The sdf value of the current face triangle */
		/* Add the sdf value to all three vertices of the face triangle */
		vertices_sdf[v0] += sdf;
		vertices_sdf[v1] += sdf;
		vertices_sdf[v2] += sdf;

		count++;
	}

	/* Compute the average sdf value of each vertex */
	/* The sdf value of each vertex equals to the arithmetic mean of the sdf values of all faces containing it */
	/* Meanwhile save the sdf values into file */
	QString outfile_str = "../data/sdf/" + QString::fromStdString(m_modelClassName) + "/" + Utils::getModelName(QString(filename)) + ".sdff";
	ofstream out(outfile_str.toStdString().c_str());
	for (int i = 0; i < nvertices; i++){
		vertices_sdf[i] /= (double)sdf_count[i];
		out << vertices_sdf[i] << endl;
	}

	input.close();
	off_in.close();
	out.close();

	emit addDebugText("SdfSubThread-" + QString::number(m_id) + ": compute sdf of " + QString(filename) 
		+ " done.\nSave the result to " + outfile_str + ".");
	qDebug() << "SdfSubThread-" + QString::number(m_id) + ": compute sdf of " + QString(filename)
		+ " done.\nSave the result to " + outfile_str + ".";
}