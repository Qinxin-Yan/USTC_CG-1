#include "Warp.h"

Warp::Warp()
{
}

Warp::~Warp()
{
}

/*void Warp::add_static_point(QPoint P)
{
	static_point_list.push_back(P);
}*/

void Warp::add_map_point(map_pair Pair)
{
	map_pair_list.push_back(Pair);
}

int Warp::map_pair_length()
{
	return map_pair_list.size();
}

map_pair Warp::get_map_pair(int i)
{
	return map_pair_list[i];
}