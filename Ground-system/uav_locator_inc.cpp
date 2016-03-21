#pragma once
#include "uav_locator_inc.hpp"
#include <stdlib.h>
#include <stdio.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <string>
#include <iostream>
#include <cmath>
#include <vector>
#include <list>
#include <algorithm>

using std::cout;
using std::endl;
using std::list;
using std::vector;
using cv::Point;
using cv::Point2f;
using cv::Mat;
using cv::imread;
using cv::Size;
using cv::MORPH_ELLIPSE;
using cv::Vec4i;
using cv::Moments;
using cv::RotatedRect;
using cv::Scalar;


vector<vector<Point> > original, received;
vector<Vec4i> shape_hierarchy;

bool set_up_complete = false;

height_point locate_uav(Mat _in_color) {
    Mat _in;
    _in_color.copyTo(_in);
    cvtColor(_in, _in, CV_BGR2GRAY);

    if (set_up_complete == false) {
        init_locate_uav();
    }
    height_point _hp;
    double lowest = INT_MAX, match;
    vector<double> matches;
    int best_match;

    _hp.x = 0;
    _hp.y = 0;
    _hp.orientation = 4;
    received = get_shapes(_in);

    //Scalar color = Scalar(0,255,0);
    //drawContours(_in_color,received,-1,color,4,8,shape_hierarchy,0,Point(0,0));
    //imshow("LORT",_in_color);

    for (int i = 0; i < static_cast<int>(received.size()); i++) {
        matches.push_back(matchShapes(original[ORIGINAL_SHAPE], received[i],
            CV_CONTOURS_MATCH_I1, 0));
    }

    for (int i = 0; i < static_cast<int>(matches.size()); i++) {
        match = matches[i];

        if (lowest > match && match < MATCH_SHAPE_THRESHOLD) {
            lowest = match;
            best_match = i;
        }
    }

    if (lowest != INT_MAX) {
        get_orientation(received, best_match, _hp);
        get_distance(received, best_match, _hp);

        Scalar color = Scalar(0,255,0);
        drawContours(_in_color,received,best_match,color,4,8,shape_hierarchy,0,Point(0,0));
        imshow("Tracking",_in_color);
    }

    matches.clear();
    received.clear();
    return _hp;
}

/*****************************************************************************
 * Initialises some global variables
 * Input:  Mat
 * Output: None
 *****************************************************************************/
void init_locate_uav(void) {
    Mat src_color = imread(SAMPLE_IMAGE);
    Mat src;
    src_color.copyTo(src);
    cvtColor(src,src,CV_BGR2GRAY);
    if (!src.data) {
        exit(-2);
    }

    original = get_shapes(src);
    Scalar color = Scalar(0,255,0);
    drawContours(src_color,original,-1,color,4,8,shape_hierarchy,0,Point(0,0));
    imshow("Shape",src_color);
    set_up_complete = true;
}

/*****************************************************************************
 * Prepares a Mat for thresholding.
 * Input:  address to a Mat
 * Output: none
 *****************************************************************************/
void prepare_mat(Mat &_src) {
    Mat _element;
    blur(_src, _src, Size(5, 5));
    _element = getStructuringElement(MORPH_ELLIPSE,
            Size(2*EROSION_SIZE + 1, 2*EROSION_SIZE+1),
            Point(EROSION_SIZE, EROSION_SIZE));
    erode(_src, _src, _element, Point(-1, -1), ERODE_PREP_ITERATIONS);
}

/*****************************************************************************
 * Performs erosion and dilation on a Mat, to make individual contours.
 * Input:  Address to a Mat
 * Output: None
 *****************************************************************************/
void erode_dilate(Mat &_src) {
    Mat _element = getStructuringElement(MORPH_ELLIPSE,
            Size(2*EROSION_SIZE + 1, 2*EROSION_SIZE+1),
            Point(EROSION_SIZE, EROSION_SIZE));
    erode(_src, _src, _element, Point(-1, -1), ERODE_ITERATIONS);
    dilate(_src, _src, _element, Point(-1, -1), DILATE_ITERATIONS);
}

/*****************************************************************************
 * Calls prepare_mat(), threshold(), erode_dilate(), and findContours() in
 * order to find shapes in the image.
 * Input:  Mat
 * Output: cds (custom struct for storing contour and hierarchy)
 *****************************************************************************/
vector<vector<Point> > get_shapes(Mat _src) {
    vector<vector<Point> > _contours;
    vector<Vec4i> _hierarchy;

    //prepare_mat(_src);
    threshold(_src, _src, THRESHOLD, BINARY_MAX, THRESHOLD_MODE);
    erode_dilate(_src);
    findContours(_src, _contours, _hierarchy, CV_RETR_TREE,
        CV_CHAIN_APPROX_SIMPLE, Point(0, 0));

    shape_hierarchy = _hierarchy;

    return _contours;
}

/*****************************************************************************
 * Calculates the orientation and the mass center of the found shape, based on
 * the found contours. Function is based on
 * http://stackoverflow.com/questions/14720722/binary-image-orientation
 * Input:  Found contours
 *         which contour matches with the original sample
 *         Address to object of type height_point to hold information about contour
 * Output: None
 *****************************************************************************/
void get_orientation(vector<vector<Point> > _contours, int _n, height_point &_hp1) {
    Moments _m = moments(_contours[_n], false);
    _hp1.x = _m.m10/_m.m00;
    _hp1.y = _m.m01/_m.m00;
    _hp1.orientation = 0.5 * atan2(((-2) * _m.mu11), (_m.mu20 - _m.mu02));
}

/*****************************************************************************
 * Calculates the biggest distance of the contour
 * Input:  vector of vector of points (contour)
           Which vector to use
           Address to object of type height_point
   Output: None
 *****************************************************************************/
void get_distance(vector<vector<Point> > _contours, int _n, height_point &_p) {
    double max_dist = -1, temp_dist;
    RotatedRect min_rect;
    Point2f points[4];

    min_rect = minAreaRect(Mat(_contours[_n]));
    min_rect.points(points);

    for (int j = 0; j < 4; j++) {
        for (int k = j; k < 4; k++) {
            temp_dist = calc_dist(points[j], points[k]);
            if (temp_dist > max_dist) {
                max_dist = temp_dist;
            }
        }
    }

    _p.distance = max_dist;
}

/*****************************************************************************
 * Calculates the distance between two points.
 * Input:  Two points
 * Output: The distance between the two points
 *****************************************************************************/
double calc_dist(Point p1, Point p2) {
    return sqrt((p2.x-p1.x)*(p2.x-p1.x)+(p2.y-p1.y)*(p2.y-p1.y));
}

/*****************************************************************************
 * Converts radians to degree
 * Input:  double radian to convert
 * Output: double degree
 *****************************************************************************/
double rtod(double _r) {
    return _r*180/M_PIl;
}

/*****************************************************************************
 * Converts degree to radians
 * Input:  double degree to convert
 * Output: double radians
 *****************************************************************************/
double dtor(double _d) {
    return _d*M_PIl/180;
}
