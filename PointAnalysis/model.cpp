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

void Model::normalize()
{
	std::cout << "This model does not have normalize() function." << std::endl;
}

void Model::outputVerticesLabels(const char *file_path)
{
	std::cout << "This model does not implement outputVerticesLabels() function." << std::endl;
}

void Model::downSample()
{
	std::cout << "This model does not implement downSample() function." << std::endl;
}

void Model::drawSymmetry(float scale)
{
	std::cout << "This model does not implement drawSymmetry() function." << std::endl;
}

void Model::splitOutput(const std::string output_dir)
{
	std::cout << "This model does not implement splitOutput() function." << std::endl;
}