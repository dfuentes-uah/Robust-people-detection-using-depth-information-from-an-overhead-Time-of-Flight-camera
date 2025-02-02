#ifndef __FUNCIONES_H
#define __FUNCIONES_H

#include <stdarg.h>

using namespace std;
using namespace cv;
/* typedef struct */
/* { */
/*     cv::Mat mat_contornos; */
/*     vector<vector<Point> > contours; */
/*     vector<Point2f>center_count; */
/*     vector<float>radius_count; */
/* }struct_count; */



void Calculo_histograma(cv::Mat & mat, cv::Mat & mask, char *nombre);
struct_count Calculo_contorno(cv::Mat mat1, cv::Mat mat2);
double getOrientation(vector < Point > &pts, Mat & img);
Mat Calculo_gradiente(Mat & original);

#endif
