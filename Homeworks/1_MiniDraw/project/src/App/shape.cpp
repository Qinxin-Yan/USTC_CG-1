#include "shape.h"

Shape::Shape()
{
}

Shape::~Shape()
{
}

void Shape::set_start(QPoint s)
{
	start = s;
}

void Shape::set_end(QPoint e)
{
	end = e;
}

void Shape::add_point(QPoint p)
{
	Point_List_.push_back(p);
}

int Shape::point_num()
{
	return Point_List_.size();
}