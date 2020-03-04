#pragma once
#include <vector>
#include <QPoint>
#include <QImage>
#include <math.h>
#include <map>

struct map_pair
{
	QPoint start;
	QPoint end;
};

class Warp
{
public:
	Warp();
	~Warp();

public:
	//void add_static_point(QPoint P);
	void add_map_point(map_pair Pair);
	int map_pair_length();
	map_pair get_map_pair(int i);

public:
	virtual void warp_image(QImage* ptr_image_) = 0;


public:
	enum WarpType
	{
		kDefault = 0,
		kIDW = 1,
		kRBF = 2,
	};

private:
	//std::vector<QPoint> static_point_list;
	std::vector<map_pair> map_pair_list;
};