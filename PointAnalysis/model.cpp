#include "model.h"

Model::Model()
{
}

Model::Model(Model::ModelType type)
{
	m_type = type;
}

Model::~Model()
{
}

Model::ModelType Model::getType()
{
	return m_type;
}

void Model::setLabels(QVector<int> labels)
{

}