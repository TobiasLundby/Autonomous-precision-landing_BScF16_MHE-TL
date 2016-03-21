#pragma once
#include <opencv2/core/core.hpp>
#include <stdlib.h>
#include <vector>

using cv::Mat;
using cv::Point3d;
using cv::Point;
using std::vector;

typedef struct height_point {
    double x;
    double y;
    double orientation;
    double distance;
} height_point;

height_point locate_uav(Mat);
void init_locate_uav(void);
void prepare_mat(Mat&);
void erode_dilate(Mat&);
vector<vector<Point> > get_shapes(Mat);
void get_orientation(vector<vector<Point> >, int, height_point&);
void get_distance(vector<vector<Point> >, int, height_point&);
double calc_dist(Point, Point);
double rtod(double r);
double dtor(double d);

#define SAMPLE_IMAGE            "src/shape.jpg"

#define THRESHOLD               60
#define THRESHOLD_MODE          1
#define BINARY_MAX              255

#define MATCH_SHAPE_THRESHOLD   1
#define ORIGINAL_SHAPE          0

#define ERODE_ITERATIONS        2
#define ERODE_PREP_ITERATIONS   5
#define DILATE_ITERATIONS       2
#define EROSION_SIZE            1

#define M_PIl                   3.141592653589793238462643383279502884L /* pi */
