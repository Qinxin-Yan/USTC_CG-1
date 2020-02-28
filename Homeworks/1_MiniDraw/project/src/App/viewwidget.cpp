#include "viewwidget.h"
#include <iostream>

ViewWidget::ViewWidget(QWidget* parent)
	: QWidget(parent)
{
	ui.setupUi(this);
	draw_status_ = false;
	shape_ = NULL;
	type_ = Shape::kDefault;
}

ViewWidget::~ViewWidget()
{
	for (size_t i = 0; i < shape_list_.size(); i++)
		if (shape_list_[i])
		{
			delete shape_list_[i];
			shape_list_[i] = NULL;
		}
}


void ViewWidget::setLine()
{
	type_ = Shape::kLine;
}

void ViewWidget::setRect()
{
	type_ = Shape::kRect;
}

void ViewWidget::setElli()
{
	type_ = Shape::kElli;
}

void ViewWidget::setPoly()
{
	type_ = Shape::kPoly;
}

void ViewWidget::mousePressEvent(QMouseEvent* event)
{
	QPainter painter(this);
	if (Qt::LeftButton == event->button())
	{
		if (type_ == Shape::kPoly)
		{
			if (shape_ == NULL) shape_ =new Poly();
			draw_status_ = true;
			shape_->add_point(event->pos());
			
		}
		else
		{
			switch (type_)
			{
			case Shape::kLine:
				shape_ = new Line();
				break;

			case Shape::kDefault:
				break;

			case Shape::kRect:
				shape_ = new Rect();
				break;

			case Shape::kElli:
				shape_ = new Elli();
				break;
			}

				if (shape_ != NULL)
				{
					draw_status_ = true;
					start_point_ = end_point_ = event->pos();
					shape_->set_start(start_point_);
					shape_->set_end(end_point_);
				}
		
		}
	}
	else if (Qt::RightButton == event->button())
	{
		if (draw_status_ && type_ == Shape::kPoly)
		{
			shape_->add_point(event->pos());
			draw_status_ = false;
			//using namespace std;
			//cout << shape_->point_num() << endl;
			shape_list_.push_back(shape_);
			update();
			//shape_->Draw(painter);
			//delete shape_;
			shape_ = NULL;
		}
	}
	update();
}


void ViewWidget::mouseMoveEvent(QMouseEvent* event)
{
	if (draw_status_ && shape_ != NULL)
	{
		end_point_ = event->pos();
		shape_->set_end(end_point_);
	}
}

void ViewWidget::mouseReleaseEvent(QMouseEvent* event)
{
	if (shape_ != NULL&&type_ != Shape::kPoly)
	{
		draw_status_ = false;
		shape_list_.push_back(shape_);
		//delete shape_;
		shape_ = NULL;
	}
}

void ViewWidget::paintEvent(QPaintEvent*)
{
	QPainter painter(this);
	//using namespace std;
	//cout << "Paint!" << endl;

	for (int i = 0; i < shape_list_.size(); i++)
	{
		shape_list_[i]->Draw(painter);
	}

	if (shape_ != NULL) {
		shape_->Draw(painter);
	}

	//painter.end();

	update();
}