#include "Warp_RBF.h"
Warp_RBF::Warp_RBF()
{
	u_parameter_ = 1.0;
	r = 10;
}

Warp_RBF::~Warp_RBF()
{
}

double Warp_RBF::get_distance(const QPoint P1, const QPoint P2)
{
	return sqrt((P1.x() - P2.x()) * (P1.x() - P2.x()) + (P1.y() - P2.y()) * (P1.y() - P2.y()));
}

double Warp_RBF::get_x_distance(const QPoint P1, const QPoint P2)
{
	return P2.x() - P1.x();
}

double Warp_RBF::get_y_distance(const QPoint P1, const QPoint P2)
{
	return P2.y() - P1.y();
}

double Warp_RBF::get_min_distance_(int input_start_point_position_)
{
	double min = 1000000000000;
	double temp=10000000;
	for (int i = 0; i < map_pair_length(); i++)
	{
		if (i != input_start_point_position_)
		{
			temp = get_distance(get_map_pair(input_start_point_position_).start, get_map_pair(i).start);
			if (temp < min) min = temp;
		}
	}
	return min;
}

double Warp_RBF::get_function_result(double two_point_distance_, double min_start_control_distance_)
{
	return pow((pow(two_point_distance_, 2) + pow(min_start_control_distance_, 2)), u_parameter_/2);
}

double Warp_RBF::get_function_result(QPoint input_point_, int control_start_point_position_)
{
	double two_point_distance_ = get_distance(input_point_, get_map_pair(control_start_point_position_).start);

	double min_distance_ = get_min_distance_(control_start_point_position_);
	double temp_;
	temp_ = get_function_result(two_point_distance_, min_distance_);
	return temp_;
}

void Warp_RBF::set_QR_matrix()
{
	Eigen::MatrixXd coefficient_matrix;
	coefficient_matrix.resize(map_pair_length(), map_pair_length());
	//double r;
	for (int row = 0; row < map_pair_length(); row++)   //initiate the coeeficient matrix
	{
		//r = get_min_distance_(row);
		
		for (int col = 0; col < map_pair_length(); col++)
		{
			coefficient_matrix(row, col) = get_function_result(get_distance(get_map_pair(row).start, get_map_pair(col).start), r);
		}
	}
	//QR = Eigen::HouseholderQR<Eigen::MatrixXf>(coefficient_matrix);
	QR = coefficient_matrix.colPivHouseholderQr();
}

Eigen::MatrixXd Warp_RBF::get_input_control_point_x_weight()
{
	Eigen::MatrixXd x_distance;
	x_distance.resize(map_pair_length(), 1);
	for (int row = 0; row < map_pair_length(); row++)    //initiate the x_distance vector
	{
		x_distance(row,0) = get_x_distance(get_map_pair(row).start, get_map_pair(row).end);
	}

	Eigen::MatrixXd x_weight= QR.solve(x_distance);  //solve the weight in x
	return x_weight;
}

Eigen::MatrixXd Warp_RBF::get_input_control_point_y_weight()
{
	Eigen::MatrixXd y_distance;
	y_distance.resize(map_pair_length(), 1);
	for (int row = 0; row < map_pair_length(); row++)    //initiate the y_distance vector
	{
		y_distance(row, 0) = get_y_distance(get_map_pair(row).start, get_map_pair(row).end);
	}

	Eigen::MatrixXd y_weight = QR.solve(y_distance);  //solve the weight in y
	return y_weight;
}

QPoint Warp_RBF::get_output_point(QPoint input_point_)
{
	double offset_x = 0;
	double offset_y = 0;
	Eigen::MatrixXd x_weight = get_input_control_point_x_weight();
	Eigen::MatrixXd y_weight = get_input_control_point_y_weight();
	for (int i = 0; i < map_pair_length(); i++)
	{
		offset_x += x_weight(i, 0) * get_function_result(input_point_, i);
		offset_y += y_weight(i, 0) * get_function_result(input_point_, i);
	}
	input_point_.setX(input_point_.x() + offset_x);
	input_point_.setY(input_point_.y() + offset_y);
	return input_point_;
}

void Warp_RBF::warp_image(QImage* ptr_image)
{
	set_QR_matrix();
	QImage image_tmp(*(ptr_image));  //copy the original image
	QPoint output_point_;
	int** is_filled_;//memorize if the pixel has been visited
	is_filled_ = new int* [ptr_image->height()];
	for (int j = 0; j < ptr_image->height(); j++)
	{
		is_filled_[j] = new int[ptr_image->width()];
	}
	for (int i = 0; i < ptr_image->height(); i++)//initiate with 0
	{
		for (int j = 0; j < ptr_image->width(); j++)
		{
			is_filled_[i][j] = 0;
		}
	}
	for (int i = 0; i < ptr_image->width(); i++)//for every pixel in the image
	{
		for (int j = 0; j < ptr_image->height(); j++)
		{
			output_point_ = get_output_point(QPoint(i, j));
			if (output_point_.x() > 0 && output_point_.y() > 0 && output_point_.x() < ptr_image->width() && output_point_.y() < ptr_image->height()) 
			{
				is_filled_[output_point_.y()][output_point_.x()] = 1;
				ptr_image->setPixelColor(output_point_.x(), output_point_.y(), image_tmp.pixelColor(QPoint(i, j)));
			}
        }
	}
	//for pixels with no color(not been visited)
	QRgb rgb;
	for (int i = 1; i < ptr_image->height() - 1; i++)
	{
		for (int j = 1; j < ptr_image->width() - 1; j++)
		{
			if (is_filled_[i][j] == 0)
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