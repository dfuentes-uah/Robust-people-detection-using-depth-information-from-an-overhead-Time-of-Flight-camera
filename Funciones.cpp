
#include <stdio.h>
#include <stdlib.h>
//#include <pmdsdk2.h>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/contrib/contrib.hpp>
#include <opencv2/core/core.hpp>

#include <stdarg.h>
  

using namespace cv;
using namespace std;

typedef struct
{   cv::Mat mat_contornos;
    vector<vector<Point> > contours;
    vector<Point2f>center_count;
    vector<float>radius_count;

}struct_count;




//------------------------CALCULO HISTOGRAMA-------------------//

void Calculo_histograma (cv::Mat& mat, cv::Mat& mask, char* nombre)
{

    // Initialize parameters
        int histSize = 256;    // bin size
        float range[] = { 0, 255 };
        const float *ranges[] = { range };

        // Calculate histogram
        MatND hist;
        calcHist( &mat, 1, 0, mask, hist, 1, &histSize, ranges, true, false );

        // Show the calculated histogram in command window
        double total;
        total = mat.rows * mat.cols;

///        for( int h = 0; h < histSize; h++ )
///             {
///                float binVal = hist.at<float>(h);
///                cout<<" "<<binVal;
///             }

        // Plot the histogram
        int hist_w = 512; int hist_h = 400;
        int bin_w = cvRound( (double) hist_w/histSize );

        Mat histImage( hist_h, hist_w, CV_8UC1, Scalar( 0,0,0) );
        normalize(hist, hist, 0, histImage.rows, NORM_MINMAX, -1, Mat() );

        for( int i = 1; i < histSize; i++ )
        {
          line( histImage, Point( bin_w*(i-1), hist_h - cvRound(hist.at<float>(i-1)) ) ,
                           Point( bin_w*(i), hist_h - cvRound(hist.at<float>(i)) ),
                           Scalar( 255, 0, 0), 2, 8, 0  );
        }

        namedWindow( "Result", 1 );    imshow( "Result", histImage );

        waitKey(0);


}


//------------------------CALCULO CONTORNOS-------------------//


struct_count Calculo_contorno (Mat mat1, Mat mat2)
{



Mat grad = mat1.clone();
Mat original = mat2.clone();

 struct_count aux_struct;
 RNG rng(12345);

      vector<vector<Point> > contours;
      vector<Vec4i> hierarchy;



   findContours( grad, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0) );

   cout << "aqui hay contornos = " << contours.size()<< endl;
   // Approximate contours to polygons + get bounding rects and circles
   vector<vector<Point> > contours_poly( contours.size() );
   vector<Rect> boundRect( contours.size() );
   vector<Point2f>center_count( contours.size() );
   vector<float>radius_count( contours.size() );

   for( int i = 0; i < contours.size(); i++ )
       { approxPolyDP( Mat(contours[i]), contours_poly[i], 3, true );
         boundRect[i] = boundingRect( Mat(contours_poly[i]) );
         minEnclosingCircle( (Mat)contours_poly[i], center_count[i], radius_count[i] );
       }



    /// Draw polygonal contour + bonding rects + circles
     Mat drawing = Mat::zeros( grad.size(), CV_8UC3 );
     Mat mask;
     for( int i = 0; i< contours.size(); i++ )
        {
          mask = Mat::zeros (45, 64, CV_8UC3);


           drawContours(original, contours_poly, i, Scalar(0,0,0), CV_FILLED);
          //drawContours( drawing, contours_poly, i, color, 1, 8, vector<Vec4i>(), 0, Point() );
          circle( original, center_count[i], (int)radius_count[i], Scalar(0,0,0), -1, 8, 0 );
         }

   ///convierto la imagen a CV_8U para poder analizar otra vez el contorno
  double minVal, maxVal;
  minMaxLoc(original, &minVal, &maxVal);
  original.convertTo(original, CV_8U, 255.0/(maxVal - minVal), -minVal * 255.0/(maxVal - minVal));

  aux_struct.mat_contornos = original;
  aux_struct.contours = contours;
  aux_struct.center_count = center_count;
  aux_struct.radius_count = radius_count;
  cout << "una pasada " << endl;
  return aux_struct;

}




//---------------------------CÁLCULO PCA-----------------------------------//
double getOrientation(vector<Point> &pts, Mat &img)
{
    //Construct a buffer used by the pca analysis
    Mat data_pts = Mat(pts.size(), 2, CV_64FC1);
    for (int i = 0; i < data_pts.rows; ++i)
    {
        data_pts.at<double>(i, 0) = pts[i].x;
        data_pts.at<double>(i, 1) = pts[i].y;
    }

    //Perform PCA analysis
    PCA pca_analysis(data_pts, Mat(), CV_PCA_DATA_AS_ROW);

    //Store the position of the object
    Point pos = Point(pca_analysis.mean.at<double>(0, 0),
                      pca_analysis.mean.at<double>(0, 1));

    //Store the eigenvalues and eigenvectors
    vector<Point2d> eigen_vecs(2);
    vector<double> eigen_val(2);
    for (int i = 0; i < 2; ++i)
    {
        eigen_vecs[i] = Point2d(pca_analysis.eigenvectors.at<double>(i, 0),
                                pca_analysis.eigenvectors.at<double>(i, 1));

        eigen_val[i] = pca_analysis.eigenvalues.at<double>(0, i);
    }


    Mat color_image;

    cvtColor(img, color_image, CV_GRAY2RGB);

    applyColorMap(color_image, color_image, COLORMAP_BONE);



    // Draw the principal components
    circle(color_image, pos, 1, CV_RGB(255, 0, 255), 2);


    line(color_image, pos, pos + 0.3 * Point(eigen_vecs[0].x * eigen_val[0], eigen_vecs[0].y * eigen_val[0]) , CV_RGB(255, 0, 0));
    line(color_image, pos, pos + 0.3 * Point(eigen_vecs[1].x * eigen_val[1], eigen_vecs[1].y * eigen_val[1]) , CV_RGB(0, 0, 255));

     imshow("PCa", color_image);

    return atan2(eigen_vecs[0].y, eigen_vecs[0].x);
}

//---------------------------CÁLCULO GRADINTE-----------------------------------//

Mat Calculo_gradiente(Mat &original)
{

    int scale = 1;
    int delta = 0;
    int ddepth = CV_16S;


    //________parte relativa al filtro sobel________//

    Mat grad;
    Mat grad_x, grad_y;
    Mat abs_grad_x, abs_grad_y;

    /// Gradient X
    Sobel( original, grad_x, ddepth, 1, 0, 3, scale, delta, BORDER_DEFAULT );
    convertScaleAbs( grad_x, abs_grad_x );

    /// Gradient Y
    Sobel( original, grad_y, ddepth, 0, 1, 3, scale, delta, BORDER_DEFAULT );
    convertScaleAbs( grad_y, abs_grad_y );

    addWeighted( abs_grad_x, 0.5, abs_grad_y, 0.5, 0, grad );

    return grad;

}
