 #include "utils.h"

int Utils::mat_no = 19;

Utils::Utils(QObject *parent)
	: QObject(parent)
{

}

Utils::~Utils()
{

}

using namespace std;
using namespace pcl;
using namespace Eigen;

double Utils::sdf(pcl::PointCloud<pcl::PointXYZ>::Ptr points, pcl::PointCloud<pcl::Normal>::Ptr normals, int searchPointIdx)
{
	double raysAngle = 30.0 / 180.0 * PI;  /* angle of the cone */
	PointXYZ searchPoint = points->at(searchPointIdx);
	Normal searchPointNormal = normals->at(searchPointIdx);

	/* Convert the point cloud to points matrix */
	Eigen::Vector3d searchPointVec(searchPoint.x, searchPoint.y, searchPoint.z);
	Eigen::Vector3d normalVec(searchPointNormal.normal_x, searchPointNormal.normal_y, searchPointNormal.normal_z); 
	normalVec.normalize();
	Vector3d cone_axis = -normalVec;
	cone_axis.normalize();

	QList<double> dists; 
	QList<double> weights;
	for (int i = 0; i < points->size(); i++)
	{
		Vector3d p(points->at(i).x, points->at(i).y, points->at(i).z);

		Vector3d vec = p - searchPointVec;
		Vector3d unit_vec = vec.normalized();
		
		/* filter out the points with a vector that isn't within the designated cone */
		double costheta = unit_vec.dot(cone_axis);
		double angle = acos(costheta);
		if (angle > 0 && angle <= raysAngle){
			Vector3d pNorm(normals->at(i).normal_x, normals->at(i).normal_y, normals->at(i).normal_z);
			pNorm.normalize();
			if (pNorm.dot(normalVec) <= 0)
			{
				dists.push_back(vec.norm());
				weights.push_back(1.0 / angle);
			}
		}
	}

	/* Sort the lengths of the rays */
	int size = dists.size();
	if (size > 0){
		QList<int> indices;  /* The original index of each element in dists after sorting */
		for (int i = 0; i < size; i++)
			indices.push_back(i);
		sort(dists, indices, 0, size - 1);

		/* Compute the standard deviation of lengths of the rays */
		boost::accumulators::accumulator_set<double, boost::accumulators::stats<boost::accumulators::tag::variance>> acc;
		acc = std::for_each(dists.begin(), dists.end(), acc);
		double sd = sqrt(boost::accumulators::variance(acc));
		/* Find the median of lengths of the rays */
		double med = 0;
		if (size % 2 == 0){
			med = (dists[size / 2 - 1] + dists[size / 2]) / 2.0;
		}
		else
			med = dists[size / 2];

		/* Take the weighted average of all rays lenghs which fall within one standard deviation from the median of all lenghts as SDF value */
		double sdf = 0;
		double sum_weights = 0;
		for (int i = 0; i < size; i++)
		{
			if (double_equal(dists[i], med - sd) || double_equal(dists[i], med + sd) ||  dists[i] > (med - sd) && dists[i] < (med + sd))
			{
				double w = weights[indices[i]];
				sdf += dists[i] * w;
				sum_weights += w;
			}
		}
		sdf /= sum_weights;

		return sdf;
	}
	else
		return 0;
}

QVector<double> Utils::sdf_mesh(QString off_mesh_filename)
{
	/* create and read Polyhedron */
	Polyhedron mesh;
	std::ifstream input(off_mesh_filename.toStdString().c_str());
	if (!input || !(input >> mesh) || mesh.empty())
	{
		std::cerr << "Not a valid off file." << std::endl;
	}
	/* create a property-map */
	typedef std::map<Polyhedron::Facet_const_handle, double> Facet_double_map;
	Facet_double_map internal_map;
	boost::associative_property_map<Facet_double_map> sdf_property_map(internal_map);

	const std::size_t number_of_rays = 100;    /* cast 100 rays per facet */
	const double cone_angle = 45.0 / 180.0 * CGAL_PI;   /* set cone opening-angle to 45 degrees */
	CGAL::sdf_values(mesh, sdf_property_map, cone_angle, number_of_rays, false);
	std::pair<double, double> min_max_sdf = CGAL::sdf_values_postprocessing(mesh, sdf_property_map);

	ifstream off_in(off_mesh_filename.toStdString().c_str());  /* Used to read indices of vertices of each face */
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

	qDebug() << "Facets count =" << count;
	/* Compute the average sdf value of each vertex */
	/* The sdf value of each vertex equals to the arithmetic mean of the sdf values of all faces containing it */
	for (int i = 0; i < nvertices; i++)
		vertices_sdf[i] /= (double)sdf_count[i];

	input.close();
	off_in.close();

	return vertices_sdf;
}
void Utils::sort(QList<double> &lens, QList<int> &indices, int low, int high)
{
	if (low < high)
	{
		int nIdx = partition(lens, indices, low, high);
		sort(lens, indices, low, nIdx - 1);
		sort(lens, indices, nIdx + 1, high);
	}
}

int Utils::partition(QList<double> &subs, QList<int> &indices, int low, int high)
{
	double nTemp = subs[high];
	int i = low, j = low - 1;
	for (; i < high; i++)
	{
		if (subs[i] <= nTemp)
		{
			j++;
			if (i != j)
			{
				double temp = subs[i];
				int itemp = indices[i];
				subs[i] = subs[j];
				indices[i] = indices[j];
				subs[j] = temp;
				indices[j] = itemp;
			}
		}
	}

	double temp = subs[j + 1];
	int itemp = indices[j + 1];
	subs[j + 1] = subs[high];
	indices[j + 1] = indices[high];
	subs[high] = temp;
	indices[high] = itemp;

	return j + 1;
}

bool Utils::double_equal(double a, double b)
{
	if (std::fabs(a - b) < 1e-8)
		return true;
	else
		return false;
}

bool Utils::float_equal(double a, double b)
{
	if (std::fabs(a - b) < 1e-6)
		return true;
	else
		return false;
}

QString Utils::getSegFilename(QString modelFilename)
{
	bool slash_seperator = modelFilename.contains('\\');
	QStringList namelist;
	if (slash_seperator)
		namelist = modelFilename.split("\\");
	else
		namelist = modelFilename.split("/");
	int size = namelist.size();
	if (size > 0)
	{
		namelist[size - 2] = "gt";
		QString name = namelist[size - 1];
		int namelen = name.length();
		QChar * namedata = name.data();
		namedata[namelen - 3] = 's';
		namedata[namelen - 2] = 'e';
		namedata[namelen - 1] = 'g';
		namelist[size - 1] = name;
		QString segName = namelist.join("\\");

		return segName;
	}
	return "";
}

std::string Utils::getSegFilename(std::string model_filename)
{
	return getSegFilename(QString::fromStdString(model_filename)).toStdString();
}

std::string Utils::getSegFilename(const char *model_file_name)
{
	return getSegFilename(QString(model_file_name)).toStdString();
}

int Utils::searchPoint(Point3 searchPoint, PointList points)
{
	for (int i = 0; i < points.size(); i++)
	{
		if (float_equal(searchPoint.x(), points.at(i).first.x())
			&& float_equal(searchPoint.y(), points.at(i).first.y())
			&& float_equal(searchPoint.z(), points.at(i).first.z()))
			return i;
	}
	return -1;
}

QVector<int> Utils::searchPoints(Point3 searchPoints[3], PointList points)
{
	bool found0 = false, found1 = false, found2 = false;
	QVector<int> idx(3);

	for (int i = 0; i < points.size(); i++)
	{

		if (!found0 && float_equal(searchPoints[0].x(), points.at(i).first.x())
			&& float_equal(searchPoints[0].y(), points.at(i).first.y())
			&& float_equal(searchPoints[0].z(), points.at(i).first.z()))
		{
			found0 = true;
			idx[0] = i; 
			continue;
		}

		if (!found1 && float_equal(searchPoints[1].x(), points.at(i).first.x())
			&& float_equal(searchPoints[1].y(), points.at(i).first.y())
			&& float_equal(searchPoints[1].z(), points.at(i).first.z()))
		{
			found1 = true;
			idx[1] = i;
			continue;
		}

		if (!found2 && float_equal(searchPoints[2].x(), points.at(i).first.x())
			&& float_equal(searchPoints[2].y(), points.at(i).first.y())
			&& float_equal(searchPoints[2].z(), points.at(i).first.z()))
		{
			found2 = true;
			idx[2] = i;
			continue;
		}
		
		if (found0 && found1 && found2)
			break;
	}

	//qDebug("searchPoints(): %d, %d, %d", idx[0], idx[1], idx[2]);

	return idx;
}

QString Utils::getModelName(QString filepath)
{
	QStringList file_list;
	bool has_splash = filepath.contains('\\');
	if (has_splash)
		file_list = filepath.split('\\');
	else
		file_list = filepath.split('/');

	int size = file_list.size();
	return file_list.at(size - 1).section('.', 0, 0);
}

std::string Utils::getModelName(std::string filepath)
{
	return getModelName(QString::fromStdString(filepath)).toStdString();
}

int Utils::comb(int n, int i)
{
	int numerator = 1;
	for (int j = 0; j < i; j++)
		numerator *= n - j;
	int denominator = 1;
	for (int j = i; j >= 1; j--)
		denominator *= j;

	return numerator / denominator;
}

QVector<QPair<int, int>> & Utils::getCombinations(QVector<int> nums)
{
	int size = nums.size();
	int count = 0;
	QVector<QPair<int, int>> combs(comb(size, 2));
	for (int i = 0; i < size; i++)
	{
		for (int j = i + 1; j < size; j++)
			combs[count++] = QPair<int, int>(nums[i], nums[j]);

	}
	for (int i = 0; i < count; i++)
		qDebug() << combs[i];
	qDebug() << "done.";

	return combs;
}

std::string Utils::vectorToString(Eigen::VectorXf vec)
{
	std::string vec_str;
	for (int i = 0; i < vec.size() - 1; i++)
		vec_str.append(std::to_string(vec(i)) + " ");
	vec_str.append(std::to_string(vec(vec.size() - 1)));
	return vec_str;
}

std::string Utils::matrixToString(Eigen::MatrixXf mat)
{
	std::string mat_str;
	for (int i = 0; i < mat.rows(); i++)
	{
		for (int j = 0; j < mat.cols() - 1; j++)
			mat_str.append(std::to_string(mat(i, j)) + " ");
		mat_str.append(std::to_string(mat(i, mat.cols() - 1)) + "\n");
	}
	return mat_str;
}

void Utils::saveMatrixToFile(Eigen::MatrixXf mat, Eigen::VectorXf relation, Eigen::VectorXf mean, Eigen::VectorXf vec)
{
	std::string output_path = "../data/covariance_" + std::to_string(mat_no) + ".txt";
	std::ofstream mat_out(output_path.c_str());
	if (mat_out.is_open())
	{
		mat_out << mat << std::endl << std::endl;
		mat_out << relation << std::endl << std::endl;
		mat_out << mean << std::endl << std::endl;
		mat_out << vec << std::endl;
		mat_out.close();
	}

	mat_no++;
}

//void Utils::savePartsPairToFile(PAPart part1, PAPart part2)
//{
//	using namespace std;
//	string out_path1 = "../data/debug/part1_" + std::to_string(mat_no) + ".txt";
//	ofstream out1(out_path1.c_str());
//	string out_path2 = "../data/debug/part2_" + std::to_string(mat_no) + ".txt";
//	ofstream out2(out_path2.c_str());
//
//	if (out1.is_open() && out2.is_open())
//	{
//		out1 << part1.getRotMat() << endl << endl;
//		out1 << part1.getTransVec() << endl << endl;
//		out1 << part1.getScale() << endl;
//
//		out2 << part2.getRotMat() << endl << endl;
//		out2 << part2.getTransVec() << endl << endl;
//		out2 << part2.getScale() << endl;
//
//		out1.close();
//		out2.close();
//	}
//}

void Utils::saveRelationToFile(Eigen::Matrix<float, 3, 4> T12, Eigen::Vector4f h1, Eigen::Matrix<float, 3, 4> T21, Eigen::Vector4f h2)
{
	using namespace std;
	using namespace Eigen;
	string out_path = "../data/debug/relation_" + to_string(mat_no) + ".txt";
	ofstream out(out_path.c_str());
	
	if (out.is_open())
	{
		out << T12 << endl << endl;
		out << h1 << endl << endl;
		out << T21 << endl << endl;
		out << h2 << endl;

		out.close();
	}
}

void Utils::saveFeatureToFile(float feature[32])
{
	using namespace std;
	string out_path = "../data/debug/feature_" + to_string(mat_no) + ".txt";
	ofstream out(out_path.c_str());

	if (out.is_open())
	{
		for (int i = 0; i < 32; i++)
			out << feature[i] << endl;

		out.close();
	}
}

using namespace std;
void Utils::saveFeatureToFile(vector<float> feature)
{
	string out_path = "../data/debug/feature_" + to_string(mat_no) + ".txt";
	ofstream out(out_path.c_str());

	if (out.is_open())
	{
		for (vector<float>::iterator it = feature.begin(); it != feature.end(); ++it)
		{
			out << *it << endl;
		}

		out.close();
	}
}

long Utils::getCurrentTime()
{
	boost::posix_time::ptime time = boost::posix_time::microsec_clock::local_time();
	boost::posix_time::time_duration duration(time.time_of_day());
	long int time_ms = duration.total_milliseconds();
	return time_ms;
}

QVector3D Utils::eigen_vector3f_to_qvector3d(Eigen::Vector3f vec)
{
	QVector3D qvec(vec.x(), vec.y(), vec.z());
	return qvec;
}

std::string Utils::getFileFormat(std::string file_path)
{
	QString q_file_path = QString::fromStdString(file_path);
	QString format = q_file_path.section('.', -1, -1);
	return format.toStdString();
}

std::string Utils::getFileFormat(const char *file_path)
{
	QString q_file_path(file_path);
	QString format = q_file_path.section('.', -1, -1);
	return format.toStdString();
}

Eigen::Matrix3f Utils::rotation_matrix(Eigen::Vector3f v0, Eigen::Vector3f v1)
{
	using namespace Eigen;

	v0.normalize();
	v1.normalize();

	Vector3f v = v0.cross(v1);
	float s = v.norm();
	float c = v0.dot(v1);

	Matrix3f ssc_v = skew_symmetric_cross(v);
	Matrix3f R = Matrix3f::Identity() + ssc_v + ssc_v * ssc_v * (1 - c) / (s * s);
	return R;
}

Eigen::Matrix3f Utils::skew_symmetric_cross(Eigen::Vector3f v)
{
	Eigen::Matrix3f ssc;
	ssc << 0, -v.z(), v.y(),
		v.z(), 0, -v.x(),
		-v.y(), v.x(), 0;

	return ssc;
}

void Utils::CHECK_NUMERICAL_ERROR(const std::string& _desc, const double& _error)
{
	if (std::abs(_error) > NUMERIAL_ERROR_THRESHOLD)
	{
		std::cerr << "(" << _desc << "): Numerical Error (" << _error << ")" << std::endl;
		do {
			std::cout << '\n' << "Press the Enter key to continue.";
		} while (std::cin.get() != '\n');
	}
}

void Utils::CHECK_NUMERICAL_ERROR(const std::string & _desc, const double & _value_1, const double & _value_2)
{
	CHECK_NUMERICAL_ERROR(_desc, _value_1 - _value_2);
}

Eigen::MatrixXd Utils::regularized_inverse(const Eigen::MatrixXd& _mat)
{
	return (_mat + 1.0E-3 * Eigen::MatrixXd::Identity(_mat.rows(), _mat.cols())).inverse();
}

void Utils::savePredictionResult(const QMap<int, int> & parts_picked, const std::string & file_path)
{
	std::ofstream out(file_path.c_str());
	if (out.is_open())
	{
		for (QMap<int, int>::const_iterator it = parts_picked.begin(); it != parts_picked.end(); ++it)
			out << it.key() << " " << it.value() << endl;

		out.close();
	}
}

QMap<int, int> Utils::loadPredictionResult(const std::string & file_path)
{
	QMap<int, int> parts_picked;

	std::ifstream in(file_path.c_str());
	
	if (in.is_open())
	{
		char buffer[8];

		while (!in.eof())
		{
			in.getline(buffer, 8);
			if (strlen(buffer) > 0)
			{
				QString pair(buffer);
				int label = pair.section(' ', 0, 0).toInt();
				int cand_index = pair.section(' ', 1, 1).toInt();
				parts_picked.insert(label, cand_index);
			}
		}
	}

	return parts_picked;
}

float Utils::euclideanDistance(pcl::PointXYZ point1, pcl::PointXYZ point2)
{
	float dist = 0;
	dist = std::sqrt((point1.x - point2.x) * (point1.x - point2.x) +
		(point1.y - point2.y) * (point1.y - point2.y) +
		(point1.z - point2.z) * (point1.z - point2.z));
	return dist;
}

void Utils::computePlane(const Eigen::Vector3f &n, double d, std::vector<Eigen::Vector3f> & triangles_vertices)
{
	Eigen::Vector3f v1(n[1], -n[0], 0);
	Eigen::Vector3f v2(-n[2], 0, n[0]);
	if (!v1.isZero())
	{
		v1.normalize();
		v2 = (v1.cross(n)).normalized();
	}
	else if (!v2.isZero())
	{
		v2.normalize();
		v1 = (v2.cross(n)).normalized();
	}

	Eigen::Vector3f unit_n = n.normalized();

	Eigen::Vector3f center = Eigen::Vector3f::Zero() - (float)d * unit_n;

	std::vector<Eigen::Vector3f> corners(4);
	corners[0] = center + v1 + v2;
	corners[1] = center + v1 - v2;
	corners[2] = center - v1 - v2;
	corners[3] = center - v1 + v2;

	triangles_vertices.resize(6);
	triangles_vertices[0] = corners[0];
	triangles_vertices[1] = corners[1];
	triangles_vertices[2] = corners[2];
	triangles_vertices[3] = corners[0];
	triangles_vertices[4] = corners[2];
	triangles_vertices[5] = corners[3];
}