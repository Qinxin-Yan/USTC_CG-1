#include "Rect.h"
//#include <iostream>

Rect::Rect()
{
}

Rect::~Rect()
{
}

void Rect::Draw(QPainter& painter)
{
	painter.drawRect(start.x(), start.y(),
		end.x() - start.x(), end.y() - start.y());
	//using namespace std;
	//cout << "Rect Draw!" << endl;
}
