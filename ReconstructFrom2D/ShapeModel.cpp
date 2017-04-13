#include "ShapeModel.h"



ShapeModel::ShapeModel()
{
}

ShapeModel::ShapeModel(ModelType type)
{
	type_ = type;
}
ShapeModel::~ShapeModel()
{
}

ShapeModel::ModelType ShapeModel::get_type()
{
	return type_;
}