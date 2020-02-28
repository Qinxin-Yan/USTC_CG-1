#include "Poly.h"
#include <iostream>

Poly::Poly()
{
}

Poly::~Poly()
{
}

void Poly::Draw(QPainter& painter)
{
	int i;
	//using namespace std;
	//cout << "Poly Draw!" << endl;
	//painter.drawPolygon(Point_List_, Point_List_->size());
	if(Point_List_.size())
	{
		for (i = 0; i < Point_List_.size() - 1; i++)
		{
			painter.drawLine(Point_List_[i], Point_List_[i + 1]);
		}
		painter.drawLine(Point_List_[Point_List_.size() - 1], Point_List_[0]);
		//using namespace std;
		//cout << "Poly Draw!" << endl;
	}
}
