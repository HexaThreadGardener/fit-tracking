//
// Created by Peter Lu on 2/24/17.
//


#ifndef FIT_TRACKING_CONNECTEDCOMPONENTS_H

#include <opencv2/core.hpp>
using namespace cv;

#define CV_CVX_WHITE    CV_RGB(0xff,0xff,0xff)
#define CV_CVX_BLACK    CV_RGB(0x00,0x00,0x00)
#define CV_CVX_ZIDINGYI  CV_RGB(110,220,180)

void ConnectedComponents(Mat &mask_process, int poly1_hull0, float scale, int &p, int number,
                         Rect * &bounding_box, const Point &contour_centers = Point(-1, -1));

#define FIT_TRACKING_CONNECTEDCOMPONENTS_H

#endif //FIT_TRACKING_CONNECTEDCOMPONENTS_H
