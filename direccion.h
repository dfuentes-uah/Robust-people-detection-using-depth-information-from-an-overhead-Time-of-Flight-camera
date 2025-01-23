

#include <stdio.h>


//#include <cv.h>
#include <opencv2/opencv.hpp>
//#include <highgui.h>
#include <iostream>
using namespace cv;
using namespace std;
int ruido(cv::Mat imgIn, cv::Mat *imgOut);
int ruido_uint16(cv::Mat imgIn, cv::Mat *imgOut);
int pixelesErroneosSuelo(Mat imgIn,Mat *imgOut);
int delecteSecciones(int position,Point coord,std :: vector < Mat > smallImages, double maxVal);
int reconstruccionImage(std :: vector < Mat > smallImages, Mat& imageReconstruct);

int pintanucleomaximos(cv::Mat imgIn, cv::Mat *imgOut,cv::Point minLoc);
int numPointsSeccion(std :: vector < Mat > smallImages,cv::Point minLoc,double minVal,int posicion,int secciones[5]);




///DIRECIONES SEGUN PAPER
int direction1(cv::Mat img, cv::Mat imgOut,cv::Mat *imgOut2,cv::Point minLoc,double minVal,std :: vector < Mat > smallImages,int aceptadas[17],int *contador3);
int direction2(cv::Mat img, cv::Mat imgOut,cv::Mat *imgOut2,cv::Point minLoc,double minVal,std :: vector < Mat > smallImages,int aceptadas[17],int *contador3);
int direction3(cv::Mat img, cv::Mat imgOut,cv::Mat *imgOut2,cv::Point minLoc,double minVal,std :: vector < Mat > smallImages,int aceptadas[17],int *contador3);
int direction4(cv::Mat img, cv::Mat imgOut,cv::Mat *imgOut2,cv::Point minLoc,double minVal,std :: vector < Mat > smallImages,int aceptadas[17],int *contador3);
int diagonal(cv::Mat img, cv::Mat imgOut,cv::Mat *imgOut2,cv::Point minLoc,double minVal,std :: vector < Mat > smallImages,int aceptadas[12],int *contador3);





/*int dir2(cv::Mat imgIn, cv::Mat *imgOut);
int dir3(cv::Mat imgIn, cv::Mat *imgOut);
int dir4(cv::Mat imgIn, cv::Mat *imgOut);
int diagonal(cv::Mat imgIn, cv::Mat *imgOut);*/


