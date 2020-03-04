#include "ImageWidget.h"
#include <QImage>
#include <QPainter>
#include <QtWidgets> 
#include <iostream>

using std::cout;
using std::endl;

ImageWidget::ImageWidget(void)
{
	ptr_image_ = new QImage();
	ptr_image_backup_ = new QImage();
	warp_type_ = Warp::kDefault;
	is_draw = 0;
	flag = 0;
	warp = NULL;
}


ImageWidget::~ImageWidget(void)
{
	delete (ptr_image_);
	delete(ptr_image_backup_);
}

void ImageWidget::paintEvent(QPaintEvent *paintevent)
{
	//cout << "paint!" << endl;
	QPainter painter;
	painter.begin(this);

	// Draw background
	painter.setBrush(Qt::lightGray);
	QRect back_rect(0, 0, width(), height());
	painter.drawRect(back_rect);

	// Draw image
	QRect rect = QRect(0, 0, ptr_image_->width(), ptr_image_->height());
	//QRect rect = QRect( (width()-ptr_image_->width())/2, (height()-ptr_image_->height())/2, ptr_image_->width(), ptr_image_->height());
	painter.drawImage(rect, *ptr_image_); 

	//Draw Line&Point
	painter.drawPoint(QPoint(0, 0));
	if (is_draw == TRUE&& warp->map_pair_length())
	{
		//painter.setPen(QPen(QColor(0, 160, 230), 16));
		//painter.drawPoint(QPoint(0, 0));
		painter.setPen(QPen(QColor(0, 160, 230), 8));
		for (int i = 0; i < warp->map_pair_length(); i++)
		{
			if (warp->get_map_pair(i).start == warp->get_map_pair(i).end)
			{
				painter.setPen(QPen(QColor(0, 160, 230), 8));
				painter.drawPoint(warp->get_map_pair(i).start);
			}
			else
			{
				painter.setPen(QPen(QColor(0, 160, 230), 8));
				painter.drawPoint(warp->get_map_pair(i).start);
				painter.setPen(QPen(QColor(3, 20, 160), 5));
				painter.drawLine(warp->get_map_pair(i).start, warp->get_map_pair(i).end);
				painter.setPen(QPen(QColor(0, 160, 230), 8));
				painter.drawPoint(warp->get_map_pair(i).end);
			}
		}
	}

	painter.end();
}

void ImageWidget::Open()
{
	// Open file
	QString fileName = QFileDialog::getOpenFileName(this, tr("Read Image"), ".", tr("Images(*.bmp *.png *.jpg)"));

	// Load file
	if (!fileName.isEmpty())
	{
		ptr_image_->load(fileName);
		*(ptr_image_backup_) = *(ptr_image_);
	}

	//ptr_image_->invertPixels(QImage::InvertRgb);
	//*(ptr_image_) = ptr_image_->mirrored(true, true);
	//*(ptr_image_) = ptr_image_->rgbSwapped();
	//cout<<"image size: "<<ptr_image_->width()<<' '<<ptr_image_->height()<<endl;
	update();
}

void ImageWidget::Save()
{
	SaveAs();
}

void ImageWidget::SaveAs()
{
	QString filename = QFileDialog::getSaveFileName(this, tr("Save Image"), ".", tr("Images(*.bmp *.png *.jpg)"));
	if (filename.isNull())
	{
		return;
	}	

	ptr_image_->save(filename);
}

void ImageWidget::Invert()
{
	for (int i=0; i<ptr_image_->width(); i++)
	{
		for (int j=0; j<ptr_image_->height(); j++)
		{
			QRgb color = ptr_image_->pixel(i, j);
			ptr_image_->setPixel(i, j, qRgb(255-qRed(color), 255-qGreen(color), 255-qBlue(color)) );
		}
	}

	// equivalent member function of class QImage
	// ptr_image_->invertPixels(QImage::InvertRgb);
	update();
}

void ImageWidget::Mirror(bool ishorizontal, bool isvertical)
{
	QImage image_tmp(*(ptr_image_));
	int width = ptr_image_->width();
	int height = ptr_image_->height();

	if (ishorizontal)
	{
		if (isvertical)
		{
			for (int i=0; i<width; i++)
			{
				for (int j=0; j<height; j++)
				{
					ptr_image_->setPixel(i, j, image_tmp.pixel(width-1-i, height-1-j));
				}
			}
		} 
		else
		{
			for (int i=0; i<width; i++)
			{
				for (int j=0; j<height; j++)
				{
					ptr_image_->setPixel(i, j, image_tmp.pixel(i, height-1-j));
				}
			}
		}
		
	}
	else
	{
		if (isvertical)
		{
			for (int i=0; i<width; i++)
			{
				for (int j=0; j<height; j++)
				{
					ptr_image_->setPixel(i, j, image_tmp.pixel(width-1-i, j));
				}
			}
		}
	}

	// equivalent member function of class QImage
	//*(ptr_image_) = ptr_image_->mirrored(true, true);
	update();
}

void ImageWidget::TurnGray()
{
	for (int i=0; i<ptr_image_->width(); i++)
	{
		for (int j=0; j<ptr_image_->height(); j++)
		{
			QRgb color = ptr_image_->pixel(i, j);
			int gray_value = (qRed(color)+qGreen(color)+qBlue(color))/3;
			ptr_image_->setPixel(i, j, qRgb(gray_value, gray_value, gray_value) );
		}
	}

	update();
}

void ImageWidget::Restore()
{
	*(ptr_image_) = *(ptr_image_backup_);
	update();
}

void ImageWidget::SetWarpToIDW()
{
	warp_type_ = Warp::kIDW;
}

void ImageWidget::SetWarpToRBF()
{
	warp_type_ = Warp::kRBF;
}


void ImageWidget::mousePressEvent(QMouseEvent* event)
{
	std::cout << "press" << std::endl;
	QPainter painter(this);
	if (Qt::LeftButton == event->button())
	{
		switch (warp_type_)
		{
		case Warp::kDefault:
			break;

		case Warp::kIDW:
			is_draw = TRUE;
			if (warp == NULL)
			{
				warp = new Warp_IDW();
			}
			pair.start = event->pos();
			pair.end = event->pos();
			//warp->add_map_point(pair);
			//painter.drawPoint(event->pos());
			break;

		case Warp::kRBF:
			is_draw = TRUE;
			if (warp == NULL)
			{
				warp = new Warp_RBF();
			}
			pair.start = event->pos();
			pair.end = event->pos();
			warp->add_map_point(pair);
			break;
		}
	}
	/*else if (Qt::LeftButton == event->button())
	{
		//flag = TRUE;
		switch (warp_type_)
		{
		case Warp::kDefault:
			break;

		case Warp::kIDW:
			is_draw = TRUE;
			if (warp == NULL)
			{
				warp = new Warp_IDW();
			}
			pair.start = event->pos();
			pair.end = event->pos();
			break;

		case Warp::kRBF:
			is_draw = TRUE;
			if (warp == NULL)
			{
				warp = new Warp_RBF();
			}
			pair.start = event->pos();
			pair.end = event->pos();
			break;
		}
	}*/
	update();
}

void ImageWidget::mouseMoveEvent(QMouseEvent* event)
{
	if (warp_type_ != Warp::kDefault)
	{
		is_draw = TRUE;
		pair.end = event->pos();
	}
	update();
}

void ImageWidget::mouseReleaseEvent(QMouseEvent* event)
{
	//std::cout << "release!" << std::endl;
	if (warp_type_ != Warp::kDefault)
	{
		std::cout << "release!" << std::endl;
		warp->add_map_point(pair);
	}
	update();
}

void ImageWidget::WarpNow()
{
	if (warp_type_ != Warp::kDefault)
	{
		is_draw = FALSE;
		warp->warp_image(ptr_image_);
		update();
		delete(warp);
		warp = NULL;
	}
}

/*void ImageWidget::mouseDoubleClickEvent(QMouseEvent* event)
{
	if (warp_type_ != Warp::kDefault)
	{
		is_draw = FALSE;
		warp->warp_image(ptr_image_);
		update();
		free(warp);
		warp = NULL;
	}
}*/