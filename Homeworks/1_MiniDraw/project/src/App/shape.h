#pragma once

#include <QtGui>
#include <vector>

class Shape
{
public:
	Shape();
	virtual ~Shape();
	virtual void Draw(QPainter &paint)=0;	void set_start(QPoint s);	void set_end(QPoint e);
	void add_point(QPoint p);
	int point_num();

public:
	enum Type
	{
		kDefault = 0,
		kLine = 1,
		kRect = 2,
		kElli = 3,
		kPoly = 4,
	};
	
protected:
	QPoint start;
	QPoint end;
	std::vector<QPoint> Point_List_;
};

