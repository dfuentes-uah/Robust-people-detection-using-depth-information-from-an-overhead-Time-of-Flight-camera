#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>
#include <stdio.h>
#define NUMCARAC 6

using namespace std;
using namespace cv;


/**
 * @function main
 */
int histogramasec( double seccion[6], Mat *histImage,Scalar color)
{
  int pointScal[NUMCARAC];

  //cout <<"vector escalado" << endl;
  for (int i=0; i<NUMCARAC;i++)
  {pointScal[i] = 150*seccion[i];
  // cout << seccion[i] << endl;

  }



  int hist_w = 500; int hist_h = 500;


    line( *histImage,Point(1,hist_h - pointScal[0]) ,Point(100,hist_h - pointScal[1]),color, 1, 8, 0  );
    line( *histImage,Point(100,hist_h - pointScal[1]) ,Point(200,hist_h - pointScal[2]),color, 1, 8, 0  );
    line( *histImage,Point(200,hist_h - pointScal[2]) ,Point(300,hist_h - pointScal[3]),color, 1, 8, 0  );
    line( *histImage,Point(300,hist_h - pointScal[3]) ,Point(400,hist_h - pointScal[4]),color, 1, 8, 0  );
line( *histImage,Point(400,hist_h - pointScal[4]) ,Point(500,hist_h - pointScal[5]),color, 1, 8, 0  );
  //imshow("histograma", *histImage );

  return 0;
}
