#pragma once
#include <QWidget>
#include <Qevent>
#include "Warp_IDW.h"
#include "Warp_RBF.h"
#include "Warp.h"

QT_BEGIN_NAMESPACE
class QImage;
class QPainter;
QT_END_NAMESPACE

class ImageWidget :
	public QWidget
{
	Q_OBJECT

public:
	ImageWidget(void);
	~ImageWidget(void);

public:
	void mousePressEvent(QMouseEvent* event);
	//void mouseDoubleClickEvent(QMouseEvent* event);
	void mouseMoveEvent(QMouseEvent* event);
	void mouseReleaseEvent(QMouseEvent* event);

protected:
	void paintEvent(QPaintEvent *paintevent);

public slots:
	// File IO
	void Open();												// Open an image file, support ".bmp, .png, .jpg" format
	void Save();												// Save image to current file
	void SaveAs();												// Save image to another file

	// Image processing
	void Invert();												// Invert pixel value in image
	void Mirror(bool horizontal=false, bool vertical=true);		// Mirror image vertically or horizontally
	void TurnGray();											// Turn image to gray-scale map
	void Restore();	                                            // Restore image to origin
	void SetWarpToIDW();                                        // Set warp type to IDW 
	void SetWarpToRBF();                                        // Set warp type to RBF
	void WarpNow();                                             //start to warp after choosing the anchor points

public:
	Warp* warp;  //warp

private:
	QImage		*ptr_image_;				// image 
	QImage		*ptr_image_backup_;
	Warp::WarpType warp_type_;
	map_pair     pair;
	bool         is_draw;
	bool         flag;
};

