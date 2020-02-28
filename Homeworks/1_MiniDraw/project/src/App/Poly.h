#pragma once

#include "shape.h"
#include <vector>
#include <iostream>

class Poly :public Shape
{
public:
	Poly();
	~Poly();

	void Draw(QPainter& painter);

//protected:
	//std::vector<QPoint> Point_List_;
};

