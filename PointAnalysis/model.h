#ifndef MODEL_H
#define MODEL_H

#include <QVector>
#include <Eigen/Core>
#include "constants.h"
#include "Seb.h"

/* Define 11 different colors */
const float COLORS[11][3] = {
	{ 1.0, 0.0, 0.0 },    /* ��ɫ */
	{ 0.0, 1.0, 0.0 },    /* ��ɫ */
	{ 0.0, 0.0, 1.0 },    /* ��ɫ */
	{ 1.0, 1.0, 0.0 },    /* ��ɫ */
	{ 0.0, 1.0, 1.0 },    /* ����ɫ */
	{ 1.0, 0.0, 1.0 },    /* ����ɫ*/
	{ 0.5, 0.0, 0.5 },    /* ��ɫ */
	{ 1.0, 0.5, 0.25 },   /* �ٻ�ɫ */
	{ 0.5, 0.5, 0.0 },
	{ 0.0, 0.5, 0.5 },
	{ 0.5, 0.5, 0.5 }     /* ��ɫ */
};

typedef Seb::Point<float> MiniPoint;
typedef std::vector<MiniPoint> PointVector;
typedef Seb::Smallest_enclosing_ball<float> Miniball;

class Model
{
public:
	enum ModelType{
		Mesh,
		PointCloud,
		MeshPointCloud
	};

	Model();
	virtual ~Model();
	Model(ModelType type);
	ModelType getType();
	virtual void draw(float scale) = 0;
	virtual void drawSymmetry(float scale);
	virtual void rotate(float angle, float x, float y, float z) = 0;
	virtual std::string getInputFilepath() const = 0;
	virtual QVector<Eigen::Vector3f> getVertices() const = 0;
	virtual Eigen::Vector3f getVertexNormal(int index) = 0;
	virtual int vertexCount() const = 0;
	virtual QVector<int> getVerticesLabels() const = 0;
	virtual QVector<int> getLabelNames() const = 0;
	virtual void output(const char *file_path) = 0;
	virtual void splitOutput(const std::string output_dir);
	virtual Eigen::Vector3f getCentroid() const = 0;
	virtual int numOfClasses() = 0;
	virtual double getRadius() const = 0;
	virtual void setLabels(QVector<int> labels);
	virtual void normalize();
	virtual void outputVerticesLabels(const char *file_path);
	virtual void downSample();

protected:
	ModelType m_type;
};

#endif