 #include "utils.h"

Utils::Utils(QObject *parent)
	: QObject(parent)
{

}

Utils::~Utils()
{

}

using namespace std;
PCModel * Utils::loadPointCloud(const char *filename)
{
	PCModel *outModel = NULL;
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

	int name_len = strlen(filename);
	char suffix[4] = { filename[name_len - 3], filename[name_len - 2], filename[name_len - 1], filename[name_len] };
		
	if (strcmp(suffix, "off") == 0)
	{
		ifstream in(filename);
		if (in.is_open())
		{
			char buffer[256];
			in.getline(buffer, 256);
			in.getline(buffer, 256);
			QString *line = new QString(buffer);
			QStringList slist = line->split(" ");
			int nvertices = slist[0].toInt();
			delete(line);

			/* Data structure for bounding sphere computation */
			const int n = nvertices;                        // number of points
			const int d = 3;                         // dimension of points
			Point *points = new Point[n]();                 // n points
			double        coord[d];

			qDebug() << "Loading points coordinates...";
			for (int i = 0; i < nvertices; i++)
			{
				in.getline(buffer, 256);
				line = new QString(buffer);
				QStringList vertex_str = line->split(" ");
				float x = vertex_str.at(0).toFloat();
				float y = vertex_str.at(1).toFloat();
				float z = vertex_str.at(2).toFloat();
				//qDebug() << x << " " << y << " " << z;
				cloud->push_back(pcl::PointXYZ(x, y, z));

				coord[0] = x;
				coord[1] = y;
				coord[2] = z;
				Point newp(d, coord, coord + d);
				points[i] = newp;
				delete line;
			}
			qDebug() << "Loading done.";
			in.close();

			qDebug() << "Computing points normals...";
			// Create the normal estimation class, and pass the input dataset to it
			pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
			ne.setInputCloud(cloud);

			// Create an empty kdtree representation, and pass it to the normal estimation object.
			// Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
			pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
			ne.setSearchMethod(tree);

			// Output datasets
			pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);

			// Use all neighbors in a sphere of radius 3cm
			ne.setRadiusSearch(0.1);
			
			// Compute the features
			ne.compute(*cloud_normals);

			qDebug() << "Normals calculation done.";
			outModel = new PCModel(nvertices, cloud, cloud_normals);
		}
	}
	else if (strcmp(suffix, "xyz") == 0)
	{
		ifstream in(filename);
		if (in.is_open())
		{
			char buffer[256];
			QVector<float> points_data;
			int nvertices = 0;
			while (!in.eof())
			{
				in.getline(buffer, 256);
				QString *line = new QString(buffer);
				QStringList vertex_str = line->split(" ");
				if (vertex_str.size() != 6)
				{
					delete(line);
					break;
				}
				float x = vertex_str.at(0).toFloat();
				float y = vertex_str.at(1).toFloat();
				float z = vertex_str.at(2).toFloat();
				float nx = vertex_str.at(3).toFloat();
				float ny = vertex_str.at(4).toFloat();
				float nz = vertex_str.at(5).toFloat();

				points_data.push_back(x);
				points_data.push_back(y);
				points_data.push_back(z);
				points_data.push_back(-nx);
				points_data.push_back(-ny);
				points_data.push_back(-nz);

				nvertices++;
				delete(line);
			}
			outModel = new PCModel(nvertices, points_data);
		}

	}
	return outModel;
}

PCModel * Utils::loadPointCloud_CGAL(const char *filename)
{
	PCModel *outModel = new PCModel();
	/* The output filename for mesh modified by CGAL algorithms */
	QString meshFilename = "../data/off_modified/" + getModelName(QString(filename)) + "_modified.off";

	int name_len = strlen(filename);
	char suffix[4] = { filename[name_len - 3], filename[name_len - 2], filename[name_len - 1], filename[name_len] };

	/* Read point cloud from off file */
	if (strcmp(suffix, "off") == 0){
		PointList points;    /* The container holding all the points and their normal vectors */
		QVector<float> points_data;    /* The points information (coordinates & normal vectors) to be used to create PCModel object */
		QVector<Point3> origin_points;

		std::ifstream off_in(filename);    /* Input file stream to read model from off file */
		char off_buffer[128];     /* Line buffer to store a line in off file */
		

		/* Get the number of faces */
		off_in.getline(off_buffer, 128);    /* Read "OFF" header */
		off_in.getline(off_buffer, 128);    /* Read the number of vertices and faces */
		QString nvertices_str = QString(off_buffer).section(' ', 0, 0);
		QString nfaces_str = QString(off_buffer).section(' ', 1, 1);
		int nvertices = nvertices_str.toInt();    /* The number of vertices */
		int nfaces = nfaces_str.toInt();    /* The number of triangle faces */

		/* Read points coordinates from off file and store them in origin_points list */
		for (int i = 0; i < nvertices; i++)
		{
			off_in.getline(off_buffer, 128);
			QString line(off_buffer);

			double x = line.section(' ', 0, 0).toDouble();
			double y = line.section(' ', 1, 1).toDouble();
			double z = line.section(' ', 2, 2).toDouble();
			Point3 point(x, y, z);
			Vector nullVector;
			PointVectorPair point_vector(point, nullVector);
			points.push_back(point_vector);    /* Add the point to points list which will be sent to normal estimation process */
			origin_points.push_back(point);    /* Add the point to origin_points list which keep the origin order of the points */
		}

		// Estimates normals direction.
		// Note: pca_estimate_normals() requires an iterator over points
		// as well as property maps to access each point's position and normal.
		qDebug() << "Estimating normals direction...";
		const int nb_neighbors = 18; // K-nearest neighbors = 3 rings
		CGAL::jet_estimate_normals<Concurrency_tag>(points.begin(), points.end(),
			CGAL::First_of_pair_property_map<PointVectorPair>(),
			CGAL::Second_of_pair_property_map<PointVectorPair>(),
			18);
		qDebug() << "Normals estimation done.";

		// Orients normals.
		// Note: mst_orient_normals() requires an iterator over points
		// as well as property maps to access each point's position and normal.
		qDebug() << "Orienting normals...";
		PointList::iterator unoriented_points_begin =
			CGAL::mst_orient_normals(points.begin(), points.end(),
			CGAL::First_of_pair_property_map<PointVectorPair>(),
			CGAL::Second_of_pair_property_map<PointVectorPair>(),
			16);
		qDebug() << "Normals orientation done.";

		std::ofstream off_out(meshFilename.toStdString().c_str());    /* Output file stream to save the model modified by CGAL algorithm to a new off file */
		/* Write the header to new off file storing the model modified */
		off_out << "OFF" << std::endl;
		off_out << nvertices_str.toStdString() + " " + nfaces_str.toStdString() + " 0" << std::endl;

		/* Add points after normals estimation into points_data which is to be sent to create PCModel object */
		for (std::vector<PointVectorPair>::iterator it = points.begin(); it != points.end(); ++it)
		{
			PointVectorPair pair = *it;
			Point3 point = pair.first;
			Vector normal = pair.second;

			points_data.push_back(point.x());
			points_data.push_back(point.y());
			points_data.push_back(point.z());
			points_data.push_back(normal.x());
			points_data.push_back(normal.y());
			points_data.push_back(normal.z());

			/* Write the point to the new off file storing the model modified by CGAL algorithms */
			off_out << QString::number(point.x()).toStdString() << " " << QString::number(point.y()).toStdString()
				<< " " << QString::number(point.z()).toStdString() << std::endl;
		}

		/* Read labels of points from gt files */
		QVector<int> points_labels(nvertices, 9294);

		std::ifstream seg_in(Utils::getSegFilename(filename).toStdString().c_str());    /* Input file stream to read segmentation files */
		if (seg_in.is_open()){    /* If the current model has label information */
			qDebug() << "Setting the labels of points...";			
			char seg_buffer[3];    /* Line buffer to store a line in seg file */

			/* Read face label and points attached to the face */
			for (int i = 0; i < nfaces; i++)
			{
				seg_in.getline(seg_buffer, 3);
				int label = std::atoi(seg_buffer);    /* The label of face i */

				off_in.getline(off_buffer, 128);    /* Since off_in has already read all the points coordinates above, it currently points to the faces infomation */
				QString face_str(off_buffer);
				QStringList face_list = face_str.split(' ');

				/* Indices of points in orgin_points of face i */
				int v1 = face_list.at(1).toInt();
				int v2 = face_list.at(2).toInt();
				int v3 = face_list.at(3).toInt();
				//qDebug("Iteration-%d, origin indices of 3 points: %d, %d, %d.", i, v1, v2, v3);

				/* Find the current indices of three points in the face i*/
				Point3 sps[3] = { origin_points[v1], origin_points[v2], origin_points[v3] };
				QVector<int> indices = searchPoints(sps, points);
				//qDebug("Iteration-%d, indices of 3 points in current list: %d, %d, %d", i, indices[0], indices[1], indices[2]);

				/* Set the point label for each point in labels array */
				if (indices[0] >= 0)
					points_labels[indices[0]] = label;
				else
					throw "Didn't find the point in origin_points list!";    /* If not find the point in origin points list, there is an error */
				if (indices[1] >= 0)
					points_labels[indices[1]] = label;
				else
					throw "Didn't find the point in origin_points list!";
				if (indices[2] >= 0)
					points_labels[indices[2]] = label;
				else
					throw "Didn't find the point in origin_points list!";

				/* Write the facet information to the new off file */
				off_out << "3 " << std::to_string(indices[0]) << " " + std::to_string(indices[1])
 					<< " " + std::to_string(indices[2]) << std::endl;
			}

			seg_in.close();
			
			qDebug() << "Labels setting done.";
		}

		off_in.close();
		off_out.close();
		delete(outModel);    /* Delete the current empty model object */
		outModel = new PCModel(nvertices, points_data, points_labels);
	}
	/* Read point cloud from xyz file */
	else if (strcmp(suffix, "xyz") == 0)
	{
		ifstream in(filename);
		if (in.is_open())
		{
			char buffer[256];
			QVector<float> points_data;
			int nvertices = 0;
			while (!in.eof())
			{
				in.getline(buffer, 256);
				QString *line = new QString(buffer);
				QStringList vertex_str = line->split(" ");
				if (vertex_str.size() < 6)
				{
					delete(line);
					break;
				}
				float x = vertex_str.at(0).toFloat();
				float y = vertex_str.at(1).toFloat();
				float z = vertex_str.at(2).toFloat();
				float nx = vertex_str.at(3).toFloat();
				float ny = vertex_str.at(4).toFloat();
				float nz = vertex_str.at(5).toFloat();

				points_data.push_back(x);
				points_data.push_back(y);
				points_data.push_back(z);
				points_data.push_back(-nx);
				points_data.push_back(-ny);
				points_data.push_back(-nz);

				nvertices++;
				delete(line);
			}

			delete(outModel);
			outModel = new PCModel(nvertices, points_data);
		}
	}
	outModel->setInputFilename(filename);
	return outModel;
}

PCModel * Utils::loadPointCloud_CGAL_SDF(const char *filename)
{
	PCModel *outModel = new PCModel();

	int name_len = strlen(filename);
	char suffix[4] = { filename[name_len - 3], filename[name_len - 2], filename[name_len - 1], filename[name_len] };

	/* Read point cloud from off file */
	if (strcmp(suffix, "off") == 0){
		PointList points;    /* The container holding all the points and their normal vectors */
		QVector<float> points_data;    /* The points information (coordinates & normal vectors) to be used to create PCModel object */
		QVector<Point3> origin_points;

		QVector<double> sdfs_origin;    /* The sdf values in the order of origin points list */
		QVector<double> sdfs_current;    /* The sdf values in the order of current points list */

		std::ifstream off_in(filename);    /* Input file stream to read model from off file */
		char off_buffer[128];     /* Line buffer to store a line in off file */

		QString sdf_filename = "../data/sdf/coseg_chairs_3/" + Utils::getModelName(QString(filename)) + ".sdff";
		std::ifstream sdf_in(sdf_filename.toStdString().c_str());    /* Input file stream to read sdf value of each vertex */
		char sdf_buffer[50];    /* Line buffer to store a sdf value */

		/* Get the number of faces */
		off_in.getline(off_buffer, 128);    /* Read "OFF" header */
		off_in.getline(off_buffer, 128);    /* Read the number of vertices and faces */
		QString nvertices_str = QString(off_buffer).section(' ', 0, 0);
		QString nfaces_str = QString(off_buffer).section(' ', 1, 1);
		int nvertices = nvertices_str.toInt();    /* The number of vertices */
		int nfaces = nfaces_str.toInt();    /* The number of triangle faces */

		sdfs_origin.resize(nvertices);
		sdfs_current.resize(nvertices);

		/* Read points coordinates from off file and store them in origin_points list */
		for (int i = 0; i < nvertices; i++)
		{
			off_in.getline(off_buffer, 128);
			QString line(off_buffer);
			sdf_in.getline(sdf_buffer, 50);

			double x = line.section(' ', 0, 0).toDouble();
			double y = line.section(' ', 1, 1).toDouble();
			double z = line.section(' ', 2, 2).toDouble();
			Point3 point(x, y, z);
			Vector nullVector;
			PointVectorPair point_vector(point, nullVector);
			points.push_back(point_vector);    /* Add the point to points list which will be sent to normal estimation process */
			origin_points.push_back(point);    /* Add the point to origin_points list which keep the origin order of the points */

			sdfs_origin[i] = std::atof(sdf_buffer);
		}

		// Estimates normals direction.
		// Note: pca_estimate_normals() requires an iterator over points
		// as well as property maps to access each point's position and normal.
		qDebug() << "Estimating normals direction...";
		const int nb_neighbors = 18; // K-nearest neighbors = 3 rings
		CGAL::jet_estimate_normals<Concurrency_tag>(points.begin(), points.end(),
			CGAL::First_of_pair_property_map<PointVectorPair>(),
			CGAL::Second_of_pair_property_map<PointVectorPair>(),
			18);
		qDebug() << "Normals estimation done.";

		// Orients normals.
		// Note: mst_orient_normals() requires an iterator over points
		// as well as property maps to access each point's position and normal.
		qDebug() << "Orienting normals...";
		PointList::iterator unoriented_points_begin =
			CGAL::mst_orient_normals(points.begin(), points.end(),
			CGAL::First_of_pair_property_map<PointVectorPair>(),
			CGAL::Second_of_pair_property_map<PointVectorPair>(),
			16);
		qDebug() << "Normals orientation done.";

		/* Add points after normals estimation into points_data which is to be sent to create PCModel object */
		for (std::vector<PointVectorPair>::iterator it = points.begin(); it != points.end(); ++it)
		{
			PointVectorPair pair = *it;
			Point3 point = pair.first;
			Vector normal = pair.second;

			points_data.push_back(point.x());
			points_data.push_back(point.y());
			points_data.push_back(point.z());
			points_data.push_back(normal.x());
			points_data.push_back(normal.y());
			points_data.push_back(normal.z());
		}

		/* Read labels of points from gt files */
		QVector<int> points_labels(nvertices, 9294);

		std::ifstream seg_in(Utils::getSegFilename(filename).toStdString().c_str());    /* Input file stream to read segmentation files */
		if (seg_in.is_open()){    /* If the current model has label information */
			qDebug() << "Setting the labels of points...";
			char seg_buffer[3];    /* Line buffer to store a line in seg file */

			/* Read face label and points attached to the face */
			for (int i = 0; i < nfaces; i++)
			{
				seg_in.getline(seg_buffer, 3);
				int label = std::atoi(seg_buffer);    /* The label of face i */

				off_in.getline(off_buffer, 128);    /* Since off_in has already read all the points coordinates above, it currently points to the faces infomation */
				QString face_str(off_buffer);
				QStringList face_list = face_str.split(' ');

				/* Indices of points in orgin_points of face i */
				int v1 = face_list.at(1).toInt();
				int v2 = face_list.at(2).toInt();
				int v3 = face_list.at(3).toInt();
				//qDebug("Iteration-%d, origin indices of 3 points: %d, %d, %d.", i, v1, v2, v3);

				/* Find the current indices of three points in the face i*/
				Point3 sps[3] = { origin_points[v1], origin_points[v2], origin_points[v3] };
				QVector<int> indices = searchPoints(sps, points);
				//qDebug("Iteration-%d, indices of 3 points in current list: %d, %d, %d", i, indices[0], indices[1], indices[2]);

				/* Set the point label for each point in labels array */
				if (indices[0] >= 0){
					points_labels[indices[0]] = label;
					sdfs_current[indices[0]] = sdfs_origin[v1];
				}
				else
					throw "Didn't find the point in origin_points list!";    /* If not find the point in origin points list, there is an error */
				if (indices[1] >= 0){
					points_labels[indices[1]] = label;
					sdfs_current[indices[1]] = sdfs_origin[v2];
				}
				else
					throw "Didn't find the point in origin_points list!";
				if (indices[2] >= 0){
					points_labels[indices[2]] = label;
					sdfs_current[indices[2]] = sdfs_origin[v3];
				}
				else
					throw "Didn't find the point in origin_points list!";
			}

			seg_in.close();
			
			qDebug() << "Labels setting done.";
		}

		off_in.close();
		sdf_in.close();
		delete(outModel);    /* Delete the current empty model object */
		outModel = new PCModel(nvertices, points_data, points_labels);
		outModel->setSdf(sdfs_current);
	}
	return outModel;
}
using namespace pcl;
using namespace Eigen;
double Utils::sdf(pcl::PointCloud<pcl::PointXYZ>::Ptr points, pcl::PointCloud<pcl::Normal>::Ptr normals, int searchPointIdx)
{
	const double pi = 3.14159265359;
	double raysAngle = 30.0 / 180.0 * pi;  /* angle of the cone */
	PointXYZ searchPoint = points->at(searchPointIdx);
	Normal searchPointNormal = normals->at(searchPointIdx);

	/* Convert the point cloud to points matrix */
	Eigen::Vector3d searchPointVec(searchPoint.x, searchPoint.y, searchPoint.z);
	Eigen::Vector3d normalVec(searchPointNormal.normal_x, searchPointNormal.normal_y, searchPointNormal.normal_z); 
	normalVec.normalize();
	Vector3d cone_axis = -normalVec;

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

QVector<QPair<int, int>> Utils::getCombinations(QVector<int> nums)
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