#include "checkmodelsthread.h"

CheckModelsThread::CheckModelsThread(QObject *parent)
	: QObject(parent), loadThread(NULL), current_no(0)
{

}

CheckModelsThread::~CheckModelsThread()
{
	clear();
}

using namespace std;

void CheckModelsThread::execute()
{
	clear();

	char file_list_path[] = "../data/coseg_chairs_8_list.txt";

	ifstream list_in(file_list_path);

	if (list_in.is_open())
	{
		char buffer[64];

		while (!list_in.eof())
		{
			list_in.getline(buffer, 64);
			if (strlen(buffer) > 0)
			{
				model_paths.push_back(string(buffer));
			}
		}
	}

	loadThread = new LoadThread(model_paths[current_no], PHASE::TRAINING, this);
	connect(loadThread, SIGNAL(loadPointsCompleted(Model *)), this, SLOT(receiveModel(Model *)));
	connect(loadThread, SIGNAL(addDebugText(QString)), this, SLOT(onDebugTextAdded(QString)));
	loadThread->setGapTime(5000);
	loadThread->start();
}

void CheckModelsThread::receiveModel(Model *model)
{
	emit showModel(model);
	current_no++;

	sampleOnMesh((MeshModel *)model);
	
	if (current_no < model_paths.size())
	{
		if (loadThread->isRunning())
			loadThread->terminate();

		loadThread->setLoadFileName(model_paths[current_no]);
		loadThread->start();
	}
	else
	{
		clear();
		emit finish();
	}
}

void CheckModelsThread::onDebugTextAdded(QString text)
{
	emit addDebugText(text);
}

void CheckModelsThread::clear()
{
	if (loadThread != NULL)
	{
		if (loadThread->isRunning())
			loadThread->terminate();
		delete(loadThread);
		loadThread = NULL;
	}

	model_paths.clear();

	current_no = 0;
}

void CheckModelsThread::sampleOnMesh(MeshModel * mesh)
{
	std::vector<FacetProb> faceProbs;
	setFaceProb(mesh, faceProbs);

	srand(time(NULL));

	std::string model_name = Utils::getModelName(mesh->getInputFilepath());
	std::string out_path = "D:\\Projects\\PointCloudAnalysis\\Data\\shape2pose\\data\\2_analysis\\coseg_chairs\\points\\even1000\\"
		+ model_name + ".pts";
	std::string mesh_out_path = "D:\\Projects\\PointCloudAnalysis\\Data\\shape2pose\\data\\1_input\\coseg_chairs\\off\\"
		+ model_name + ".off";

	//mesh->output(mesh_out_path.c_str());

	std::ofstream out(out_path.c_str());

	if (out.is_open())
	{
		int num_samples = std::max(1000, mesh->faceCount());
		for (int i = 0; i < num_samples; i++)
		{
			Eigen::Vector3f v;
			Eigen::Vector3f bary_coord;
			int face_index;
			getRandomPoint(mesh, faceProbs, v, bary_coord, face_index);

			out << face_index << " " << bary_coord.x() << " " << bary_coord.y() << " " << bary_coord.z()
				<< " " << v.x() << " " << v.y() << " " << v.z() << std::endl;
		}

		out.close();
	}
}

void CheckModelsThread::setFaceProb(MeshModel *m, vector<FacetProb> &faceProbs)
{
	//calculate the areas of triangle meshes
	faceProbs.clear();
	int i = 0;
	for (QVector<Eigen::Vector3i>::iterator face_it = m->faces_begin(); face_it != m->faces_end(); ++face_it, i++)
	{
		FacetProb facetP;
		facetP.index = i; // t the index of the triangle
		int indexA = face_it->operator[](0);
		int indexB = face_it->operator[](1);
		int indexC = face_it->operator[](2);
		Eigen::Vector3f vA(m->getVertex(indexA));
		Eigen::Vector3f vB(m->getVertex(indexB));
		Eigen::Vector3f vC(m->getVertex(indexC));
		facetP.area = TriangleArea(vA, vB, vC);
		//insert area into areaArray and ensure the ascending order
		int k;
		for (k = 0; k<faceProbs.size(); k++)
		{
			if (facetP.area<faceProbs[k].area) break;
		}
		faceProbs.insert(faceProbs.begin() + k, facetP);
	}
	//area normalization
	double total_area = 0;
	for (int i = 0; i < faceProbs.size(); i++)
	{
		total_area += faceProbs[i].area;
	}
	if (total_area != 0)
	{
		for (int i = 0; i< faceProbs.size(); i++)
		{
			faceProbs[i].area /= total_area;
		}
	}
}

bool CheckModelsThread::getRandomPoint(MeshModel *m, const vector<FacetProb> &faceProbs, 
	Eigen::Vector3f &v, Eigen::Vector3f & bary_coord, int & face_index)
{
	double d = (double)(rand() / (double)RAND_MAX);
	int i = 0;	/* "i" indicates the triangle in the ordered area list */
	while (d >= 0 && i < faceProbs.size())
	{
		d = d - faceProbs[i].area;
		i++;
	}
	if (d<0 || fabs(d) < 0.000001) i--;
	else
	{
		printf("d=%lf\nindex=%d\tarea_size=%d\n", d, i, faceProbs.size());
		return false;
	}
	
	face_index = faceProbs[i].index;	/* "index" indicates the selected triangle */
	Eigen::Vector3f p1 = m->getVertex(m->getFace(face_index)[0]);
	Eigen::Vector3f p2 = m->getVertex(m->getFace(face_index)[1]);
	Eigen::Vector3f p3 = m->getVertex(m->getFace(face_index)[2]);

	bary_coord = (p1 + p2 + p3) / 3.0;

	//	srand((unsigned)time(NULL)); 
	double r1 = (double)(rand() / (double)RAND_MAX);
	double r2 = (double)(rand() / (double)RAND_MAX);
	v[0] = (1 - sqrt(r1)) * p1.x() + sqrt(r1)*(1 - r2) * p2.x() + sqrt(r1) * r2 * p3.x();
	v[1] = (1 - sqrt(r1)) * p1.y() + sqrt(r1)*(1 - r2) * p2.y() + sqrt(r1) * r2 * p3.y();
	v[2] = (1 - sqrt(r1)) * p1.z() + sqrt(r1)*(1 - r2) * p2.z() + sqrt(r1) * r2 * p3.z();

	return true;
}

double CheckModelsThread::TriangleArea(const Eigen::Vector3f &pA, const Eigen::Vector3f &pB, const Eigen::Vector3f &pC)
{
	//calculate the lengths of sides pA-pB and pA-pC
	Eigen::Vector3f v1 = pB - pA;
	double a = v1.norm();
	Eigen::Vector3f v2 = pC - pA;
	double b = v2.norm();
	//calculate the angle between sides pA-pB and pA-pC
	if (a == 0 || b == 0)
		return 0;
	double cos_angle = v1.dot(v2) / (a * b);

	//avoid accumulative error
	if (cos_angle>1)
		cos_angle = 1;
	else if (cos_angle<-1)
		cos_angle = -1;

	double angle = acos(cos_angle);
	double area = a*b*sin(angle) / 2;

	return area;
}