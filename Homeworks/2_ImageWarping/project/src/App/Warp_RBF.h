#pragma once
#include "Warp.h"
#include <Eigen/Dense>

using namespace std;

class Warp_RBF :public Warp
{
public:
	Warp_RBF();
	~Warp_RBF();

public:
	void warp_image(QImage* ptr_image_);

	double get_distance(const QPoint p1, const QPoint p2);//get distance
	double get_x_distance(const QPoint p1, const QPoint p2);//get distance in x
	double get_y_distance(const QPoint p1, const QPoint p2);//get distance in y
	double get_min_distance_(int input_start_point_position_);//get the minimum distance of the iput_point and other anchor points

	double get_function_result(double two_point_distance_, double min_start_control_distance_);//min_start_control_distance=r
	double get_function_result(QPoint input_point_, int control_start_point_positon_);//get the result of the basis radial function

	void set_QR_matrix();    //QR on the coeffienct matrix
	Eigen::MatrixXd get_input_control_point_x_weight();//the weight in x
	Eigen::MatrixXd get_input_control_point_y_weight();//the weight in y

	QPoint get_output_point(QPoint input_point_);//get the output point under RBF

private:
	Eigen::ColPivHouseholderQR<Eigen::MatrixXd> QR;
	double u_parameter_;   //parameter u
	double r;    //parameter r
};
