//para operaciones opencv
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
//#include <pmdsdk2.h>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/contrib/contrib.hpp>
#include <opencv2/core/core.hpp>


/*
//para pcl
#include <pcl/ModelCoefficients.h>

#include <pcl/io/vtk_io.h>
#include <pcl/console/parse.h>
#include <pcl/visualization/histogram_visualizer.h>

#include <boost/thread/thread.hpp>
#include <pcl/point_types.h>
#include <pcl/features/pfh.h>

#include <pcl/features/normal_3d.h>

#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>

//segmentacion
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>

*/


//typedef pcl::PointXYZ PointT;
using namespace cv;
using namespace std;
//using namespace pcl;



//pcl::visualization::PCLVisualizer _viewer_mexican ("Mexican_window");



///==========================================================================///
///                         GAUSSIANA VISUAL EN PCL                          ///
///                                                                          ///
///==========================================================================///
/*

pcl::PointCloud<pcl::PointXYZ>::Ptr paint_gaussian(float mu_x, float mu_y, float ro)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr gaussian_cloud (new pcl::PointCloud<pcl::PointXYZ>);


    gaussian_cloud->width = 200;//200
    gaussian_cloud->height = 200; //200
    gaussian_cloud->is_dense = false;
    gaussian_cloud->points.resize (gaussian_cloud->width * gaussian_cloud->height);

    float* x_array= new float[gaussian_cloud->width * gaussian_cloud->height];
    float* y_array= new float[gaussian_cloud->width * gaussian_cloud->height];
    float* z_array= new float[gaussian_cloud->width * gaussian_cloud->height];



    int width = gaussian_cloud->width; // bucle de k (variable x)
    int height = gaussian_cloud->height; //bucle de j (variable y)



    float ro_x = ro;
    float ro_y = ro;


    float ampl = (1/(ro_y*sqrt(2*M_PI)));

    double exp_aux_x, exp_aux_y;
    int j_bucle=0;
    int k_bucle=0;


    for(int j=-width/2; j < width/2; j++)
    {
        for (int k=-height/2; k< height/2; k++)
        {

        x_array[height*j_bucle+k_bucle]=k;
        y_array[height*j_bucle+k_bucle]=j;

        exp_aux_x = (((k-mu_x)/ro_x)*((k-mu_x)/ro_x))*(-0.5);
        exp_aux_y = (((j-mu_y)/ro_y)*((j-mu_y)/ro_y))*(-0.5);

        z_array[height*j_bucle+k_bucle] = ampl* exp(exp_aux_x + exp_aux_y);

        k_bucle++;
        }
        j_bucle++;
        k_bucle=0;
    }


    for ( int i=0; i< gaussian_cloud->points.size();i++)
    {
        gaussian_cloud->points[i].x = x_array[i];
        gaussian_cloud->points[i].y = y_array[i];
        gaussian_cloud->points[i].z = z_array[i]*1000;

        if(x_array[i]==0 && y_array[i]==0 && gaussian_cloud->points[i].z == 0) cout <<"eje de coordenadas?" <<endl;
    }

    return gaussian_cloud;

   // _viewer_gauss.setBackgroundColor (0, 0, 0);
   // _viewer_gauss.addPointCloud(gaussian_cloud, "gaussian");
   // _viewer_gauss.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 0.0, 0.0, 1.0, "gaussian");
   // _viewer_gauss.spin();



}

*/
///==========================================================================///
///                     SOMBRERO MEJICANO VISUAL EN PCL                      ///
///                                                                          ///
///==========================================================================///

/*
void paint_mexican()
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr mexican_cloud (new pcl::PointCloud<pcl::PointXYZ>);


    mexican_cloud->width = 100;//200
    mexican_cloud->height = 100; //200
    mexican_cloud->is_dense = false;
    mexican_cloud->points.resize (mexican_cloud->width * mexican_cloud->height);

    float* x_array= new float[mexican_cloud->width * mexican_cloud->height];
    float* y_array= new float[mexican_cloud->width * mexican_cloud->height];
    float* z_array= new float[mexican_cloud->width * mexican_cloud->height];



    int width = mexican_cloud->width; // bucle de k (variable x)
    int height = mexican_cloud->height; //bucle de j (variable y)



    float ro_x = 10;
    float ro_y = 10;
    float mu_x = 0;
    float mu_y = 0;

    float ampl = (1/(ro_y*sqrt(2*M_PI)));

    double exp_aux_x, exp_aux_y, ampli_comp;
    int j_bucle=0;
    int k_bucle=0;


    for(int j=-width/2; j < width/2; j++)
    {
        for (int k=-height/2; k< height/2; k++)
        {

        x_array[height*j_bucle+k_bucle]=k;
        y_array[height*j_bucle+k_bucle]=j;

        exp_aux_x = (((k-mu_x)/ro_x)*((k-mu_x)/ro_x))*(-0.5);
        exp_aux_y = (((j-mu_y)/ro_y)*((j-mu_y)/ro_y))*(-0.5);
        ampli_comp = 1-(((k-mu_x)/ro_x)*((k-mu_x)/ro_x) + (((j-mu_y)/ro_y)*((j-mu_y)/ro_y)));

        z_array[height*j_bucle+k_bucle] = ampl*ampli_comp*exp(exp_aux_x + exp_aux_y);

        k_bucle++;
        }
        j_bucle++;
        k_bucle=0;
    }


    for ( int i=0; i< mexican_cloud->points.size();i++)
    {
        mexican_cloud->points[i].x = x_array[i];
        mexican_cloud->points[i].y = y_array[i];
        mexican_cloud->points[i].z = z_array[i]*1000;

        if(x_array[i]==0 && y_array[i]==0 && mexican_cloud->points[i].z == 0) cout <<"eje de coordenadas?" <<endl;
    }

    //return mexican_cloud;

    _viewer_mexican.setBackgroundColor (0, 0, 0);
    _viewer_mexican.addPointCloud(mexican_cloud, "mexican");
    _viewer_mexican.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 0.0, 0.0, 1.0, "mexican");
    _viewer_mexican.spin();



}


*/
