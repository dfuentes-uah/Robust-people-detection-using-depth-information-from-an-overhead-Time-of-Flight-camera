#ifndef __CLASIFICADOR_H
#define __CLASIFICADOR_H

int *heighNumPoints(Mat & original, int *pointsCarac);
float mahalanovisDist(Mat actual, Mat & mediaVector, Mat & covariance,
                      int numCarac);
double PCAclasification(Mat samples, Mat actual);

double minValue(double a, double b);

#endif
