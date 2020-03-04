#include "Warp_IDW.h"
#include <iostream>

Warp_IDW::Warp_IDW()
{
	weight_parameter_ = 2.0;   //set the weight parameter miu to 2
}

Warp_IDW::~Warp_IDW()
{
}

double Warp_IDW::get_distance(QPoint P1, QPoint P2)
{
	return sqrt((P1.x() - P2.x()) * (P1.x() - P2.x()) + (P1.y() - P2.y()) * (P1.y() - P2.y()));
}

std::map<int, double> Warp_IDW::get_weight_list(const QPoint input_point)
{
	std::map<int, double> point_correspond_control_point_weight_map;  //the map_list for the imput point
	double sum_weight_ = 0;    //initiate the sum weight

	for (int i = 0; i < map_pair_length(); i++)
	{
		double temp_ = 0;
		temp_ = 1 / get_distance(get_map_pair(i).start, input_point);
		temp_ = pow(temp_, weight_parameter_);    
		sum_weight_ = sum_weight_ + temp_;
	}

	for (int i = 0; i < map_pair_length(); i++)
	{
		double temp_ = 0;
		temp_ = 1 / this->get_distance(get_map_pair(i).start, input_point);
		temp_ = pow(temp_, weight_parameter_);
		temp_ = temp_ / sum_weight_;
		point_correspond_control_point_weight_map.insert(std::make_pair(i, temp_));//insert the weight to the map
	}
	return point_correspond_control_point_weight_map;
}

QPoint Warp_IDW::get_output_point(QPoint input_point)
{
	double offset_x_ = 0;   
	double offset_y_ = 0;   
	double x = 0, y = 0;   
	std::map<int, double> tem_map_ = get_weight_list(input_point);
	for (int i = 0; i < map_pair_length(); i++)
	{
		offset_x_ = get_map_pair(i).end.x() - get_map_pair(i).start.x(); 
		offset_y_ = get_map_pair(i).end.y() - get_map_pair(i).start.y();
		x += offset_x_ * tem_map_[i];//multiply the change in x with weight
		y += offset_y_ * tem_map_[i];

	}
	input_point.setX(input_point.x() + x);	
	input_point.setY(input_point.y() + y);
	return input_point;
}

void Warp_IDW::warp_image(QImage* ptr_image)
{
	QImage image_tmp(*(ptr_image));
	QPoint output_point_;
	int** is_filled_;       //if the pixel has been changed
	is_filled_ = new int* [ptr_image->height()];
	for (int j = 0; j < ptr_image->height(); j++)
	{
		is_filled_[j] = new int[ptr_image->width()];
	}
	for (int i = 0; i < ptr_image->height(); i++)//initiate
	{
		for (int j = 0; j < ptr_image->width(); j++)
		{
			is_filled_[i][j] = 0;
		}
	}

	std::cout << ptr_image->height() << "," << ptr_image->width() << std::endl;

	for (int i = 0; i < ptr_image->width(); i++)
	{
		for (int j = 0; j < ptr_image->height(); j++)
		{
			output_point_ = this->get_output_point(QPoint(i, j));//get the output point
			if (output_point_.x() > 0 && output_point_.y() > 0 && output_point_.x() < ptr_image->width() && output_point_.y() < ptr_image->height()) 
			{
				is_filled_[output_point_.y()][output_point_.x()] = 1;
				ptr_image->setPixelColor(output_point_.x(), output_point_.y(), image_tmp.pixelColor(QPoint(i, j)));
			}
		}
	}

	//for pixels with no color
	QRgb rgb;
	for (int i = 1; i < ptr_image->height() - 1; i++)
	{
		for (int j = 1; j < ptr_image->width() - 1; j++)
		{
			if (is_filled_[i][j] == 0)//if the pixel has not been colored
			{
				if (is_filled_[i + 1][j] != 0)
				{
					ptr_image->setPixelColor(j, i, ptr_image->pixelColor(QPoint(j, i + 1)));
				}
				else if (is_filled_[i - 1][j] != 0)
				{
					ptr_image->setPixelColor(j, i, ptr_image->pixelColor(QPoint(j, i - 1)));
				}
				else if (is_filled_[i][j + 1] != 0)
				{
					ptr_image->setPixelColor(j, i, ptr_image->pixelColor(QPoint(j + 1, i)));
				}
				else if (is_filled_[i][j - 1] != 0)
				{
					ptr_image->setPixelColor(j, i, ptr_image->pixelColor(QPoint(j - 1, i)));
				}


				if (is_filled_[i + 1][j + 1] != 0)
				{
					ptr_image->setPixelColor(j, i, ptr_image->pixelColor(QPoint(j + 1, i + 1)));
				}
				else if (is_filled_[i - 1][j - 1] != 0)
				{
					ptr_image->setPixelColor(j, i, ptr_image->pixelColor(QPoint(j - 1, i - 1)));
				}
				else if (is_filled_[i + 1][j - 1] != 0)
				{
					ptr_image->setPixelColor(j, i, ptr_image->pixelColor(QPoint(j - 1, i + 1)));
				}
				else if (is_filled_[i - 1][j + 1] != 0)
				{
					ptr_image->setPixelColor(j, i, ptr_image->pixelColor(QPoint(j + 1, i - 1)));
				}
			}

		}
	}
	for (int i = 0; i < ptr_image->height(); i++) delete is_filled_[i];
}

void Warp_IDW::set_weight_parameter(double weighted_parameter)
{
	weight_parameter_ = weighted_parameter;
}

double Warp_IDW::get_weight_parameter()
{
	return weight_parameter_;
}