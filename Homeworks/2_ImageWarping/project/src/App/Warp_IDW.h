#pragma once
#include "Warp.h"
#include <math.h>
#include <map>

class Warp_IDW:public Warp
{
public:
	Warp_IDW();
	~Warp_IDW();

public:
	void warp_image(QImage* ptr_image_);   //use IDW to warp the image 

	double get_distance(QPoint P1, QPoint P2);              //get the distance from two points
	std::map<int, double> get_weight_list(const QPoint input_point); //get the weight for the imput point
	//delete const
	QPoint get_output_point(QPoint input_point);      //get the output point

	void set_weight_parameter(double weighted_parameter_);//set weight parameter
	double get_weight_parameter();                    //get weight parameter

private:
	double weight_parameter_;
};
