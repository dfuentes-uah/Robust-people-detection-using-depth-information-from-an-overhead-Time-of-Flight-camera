//para operaciones opencv
#include <stdio.h>
#include <pmdsdk2.h>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/contrib/contrib.hpp>
#include <opencv2/core/core.hpp>




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

#include "Funciones_opencv.h"
#include "Funciones_pcl.h"


#define USE_VIEWER  1

#define desv_head_mayor 0.05
#define desv_head_menor 0.04
#define max_head_rad 0.17
#define min_head_rad 0.06
//#define radius 0.001
#define rel_head_alt 0.0625 // 1/16 altura

#define Distsphere_th_140 0.015
#define Distsphere_th_170 0.025
#define Distsphere_th_210 0.040

typedef pcl::PointXYZ PointT;
using namespace cv;
using namespace pcl;
using namespace std;




pcl::visualization::PCLVisualizer _viewer ("Simple visualizing head");
pcl::visualization::PCLVisualizer _viewer_head ("HEAD");
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_head (new pcl::PointCloud<pcl::PointXYZ>);

//variables para contornos

int max_thresh = 255;
//RNG rng(12345);
  Mat dst2;
  vector<vector<Point> > contours;
  vector<Vec4i> hierarchy;

Mat threshold_output = Mat::zeros(48, 64, CV_32FC1);

Mat grad_x, grad_y;
Mat abs_grad_x, abs_grad_y;


Point minLoc, maxLoc;

char cadena[200];
char line_name[10];
char numero[2];
char line_max;




///*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
///+		FUNCIÓN DETECCIÓN CABEZA Y HOMBROS POR PCL                      +
///+                                                                        +
///+	Argumentos que recibe:                                              +
///+	- cloud: nube de puntos a analizar                                  +
///+	- pos: posición más alta de la imágen                           	+
///+	- posz: posición del suelo                                        	+
///+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/


bool detection_pcl (pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int pos, int posz, bool true_head)
{
    true_head = false;
    float DistSphere_th;
    bool sphere_condition = false;

    float head_rad, shoul_rad;
    float suelo = cloud->points[posz].z;//la distancia entre la cámara y el suelo
    float altura = cloud->points[pos].z; //la distancia entre la cámara y la cabeza
    float altura_objeto = suelo-altura;
    float altura_hombros = (suelo-altura)*0.18 + altura; //1.5/8*altura
    float z_aument=(suelo-altura)*0.125; // posición de Z que bajar para encontrar hombros

    cout << "---------------------------------------------------------------" << endl;

    cout << "distancia al suelo = " << suelo << endl;
    cout << "altura a secas = " << altura << endl;
    cout << "altura persona = " << suelo - altura << endl;
    cout << "altura hombros = " << altura_hombros << endl;
    cout << "z_aument = " << z_aument <<endl;
    bool head=false;
    bool shoulder=false;
    bool flag = false;


    head_rad = (suelo-altura)*rel_head_alt; //radio estimado de la cabeza
    cout << "radio estimado de la cabeza = " << head_rad << endl;



///----------definición de los planos auxiliares

        pcl::ModelCoefficients circle_coef;
        circle_coef.values.resize (3);
        pcl::ModelCoefficients sphere_coeff;
        sphere_coeff.values.resize (4); // We need 4 values
        pcl::ModelCoefficients line_coeff;
        line_coeff.values.resize (6); // We need 6 values

        PointT line_start, line_end;

 ///----------variables relativas a la búsqueda de vecino de la cabeza



if (altura_objeto < 0.90 || altura_objeto > 2.10)

    cout << "DETECCIÓN DEMASIADO PEQUEÑA O GRANDE " << endl;

else
{


/// =============================================== DETECCIÓN DE LA CABEZA =========================================================///

 ///-----------------------ROI SEGMENTATION------------------------

 /*se utiliza para comprobar si la sección seleccionada se aproxima a una forma redonda */
float x_ROI = 0.25;
float y_ROI = 0.25;


pcl::copyPointCloud( *cloud,*cloud_head);

 pcl::PassThrough<PointT> pass;
 pass.setInputCloud (cloud_head);
 pass.setFilterFieldName ("x");
 pass.setFilterLimits (cloud_head->points[pos].x - x_ROI, cloud_head->points[pos].x + x_ROI);
 pass.setFilterFieldName ("y");
 pass.setFilterLimits (cloud_head->points[pos].y - y_ROI, cloud_head->points[pos].y + y_ROI);
 pass.setFilterFieldName ("z");
 pass.setFilterLimits (0, cloud_head->points[pos].z + z_aument);
 pass.filter (*cloud_head);




//coeficienrtes para representar la esfera, no busco el punto mas alto sino el centro de la cabeza, por eso bajo el radio
    sphere_coeff.values[0] = cloud_head->points[ pos ].x;
    sphere_coeff.values[1] = cloud_head->points[ pos ].y;
    sphere_coeff.values[2] = cloud_head->points[ pos ].z+z_aument/2;
    sphere_coeff.values[3] = z_aument/2;
    _viewer_head.addSphere (sphere_coeff, "sphere_head",0);


    float dist_point_shpere = 0;
    float sphere_distance_mean1 = 0;

    for (int points_head = 0; points_head < cloud_head->size(); points_head++)
    {
        dist_point_shpere = sphera_point_distance(cloud_head->points[ pos ].x,cloud_head->points[ pos ].y, cloud_head->points[ pos ].z + z_aument/2, z_aument/2, cloud_head->points[points_head].x,cloud_head->points[points_head].y,cloud_head->points[points_head].z);
        sphere_distance_mean1 = sphere_distance_mean1+ dist_point_shpere;

    }

    sphere_distance_mean1 = fabs((sphere_distance_mean1/cloud_head->size())-(z_aument/2));

    //cout << "distancia media de los puntos a la esfera = " << sphere_distance_mean1<< endl;

//Selección de la distancia umbral a la esfera en función de la altura
if (altura_objeto > 0 && altura_objeto <= 1.40)
    DistSphere_th = Distsphere_th_140;
if (altura_objeto > 1.40 && altura_objeto <= 1.70)
    DistSphere_th = Distsphere_th_170;
if (altura_objeto > 1.70 && altura_objeto <= 2.20)
    DistSphere_th = Distsphere_th_210;

if (sphere_distance_mean1 < DistSphere_th)
   {
    sphere_condition = true;
    }

///-------------------------------------------------------------------



//operaciones relativas a la visualización
 if (USE_VIEWER)
    {

     cout <<"estoy dentro del if para poner la nube de puntos!!" << endl;
     _viewer.addCoordinateSystem(1.0, 0);//, (const char *)"CS1", 0);
     _viewer.setBackgroundColor (0, 0, 0);
	 _viewer.addPointCloud(cloud, "scene_cloud");
	 _viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 0.0, 0.0, 1.0, "scene_cloud");

     _viewer_head.addCoordinateSystem(1.0, 0);//, (const char *)"CS1", 0);
     _viewer_head.setBackgroundColor (0, 0, 0);
     _viewer_head.addPointCloud(cloud_head, "cloud_head");
     _viewer_head.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 0.0, 0.0, "cloud_head");
    }


//_viewer_aux.spinOnce(1, false);

PointXYZ searchPoint_CB;
PointXYZ searchPoint_HB;

searchPoint_CB.x = cloud->points[pos].x;
searchPoint_CB.y = cloud->points[pos].y;
searchPoint_CB.z = cloud->points[pos].z;

KdTreeFLANN<PointXYZ> kdtree;
kdtree.setInputCloud(cloud);


//------con radio a examinar

std::vector<int> pointIdxRadiusSearch_CB;
std::vector<float> pointRadiusSquaredDistance_CB;
std::vector<int> pointIdxRadiusSearch_HB;
std::vector<float> pointRadiusSquaredDistance_HB;

float Dist_th = 0;
int pos_rad;
float radius = 0.099;




int contador =0;
int contador_max = 0;

float center_pointx = cloud->points[pos].x;
float center_pointy = cloud->points[pos].y;
float actual_pointx, actual_pointy, distance_actual_point, distance_last_point, distance_p2p, distance_p2p_min = 1000;
float next_pointx = center_pointx;
float next_pointy = center_pointy;

if (head_rad > max_head_rad || head_rad < min_head_rad || sphere_condition == false)
  {cout << "ese punto no es cabeza " << endl;}



else{

//cout << "Punto centro " << cloud->points[pos].x << " " << cloud->points[pos].y << " " << cloud->points[pos].z << endl;
bool condition1 = true;
bool condition2 = false;


do{


    contador = 0;

    if ( kdtree.radiusSearch (searchPoint_CB, radius, pointIdxRadiusSearch_CB, pointRadiusSquaredDistance_CB) > 0 )
      {
        ///cout<< "if del kdtree" <<endl;
        ///cout<<"pointIdx...."<< pointIdxRadiusSearch_CB.size () << endl;

         for (size_t h = 0; h < pointIdxRadiusSearch_CB.size (); ++h)
          {
                contador++;
                if (pointRadiusSquaredDistance_CB[h] > Dist_th)
                     {Dist_th = pointRadiusSquaredDistance_CB[h];
                      pos_rad = h;
                     }
                if (contador > contador_max)
                {contador_max = contador;}
           }
      }


    radius = radius + 0.009;

    //cout << "puntos en la cabeza = " << contador_max << endl;
    //cout << "radiusss = " << radius << endl;
    ///cout << "Dist_th = " << Dist_th << endl;
    ///cout << "posicion mas alta  = " << cloud->points[pos].z << endl;
    ///cout << "posicion en z con la que comparamos = " << cloud->points[pointIdxRadiusSearch_CB[pos_rad]].z << endl;

        //ifs para las condiciones del bucle
        if (fabs(cloud->points[pos].x - cloud->points[pointIdxRadiusSearch_CB[pos_rad]].x) > altura_hombros-altura)
              {  condition1 = false;
                cout << "CAMBIA CONDITION1!!!!!!!"<< endl;}


   }while ((fabs(cloud->points[pos].z - cloud->points[pointIdxRadiusSearch_CB[pos_rad]].z) < altura_hombros-altura)); //explicar porque esa distancia

/*
cout << "radio estandimado cabeza segun altura = " << head_rad << "m" << endl;
cout << "Radio máximo localizado = " << Dist_th << endl;
cout << "Desviación permitida del 3%" << endl;*/

if (Dist_th < head_rad + desv_head_mayor && Dist_th > head_rad - desv_head_menor)
{
    cout << "Se encuentra dentro del rango"<< endl;
head = true;
flag = true;
true_head = true;}


if (head)
{
    cout << " =================CABEZA=================== " << endl;
//coeficienrtes para representar la esfera, no busco el punto mas alto sino el centro de la cabeza, por eso bajo el radio
    sphere_coeff.values[0] = cloud->points[ pos ].x;
    sphere_coeff.values[1] = cloud->points[ pos ].y;
    sphere_coeff.values[2] = cloud->points[ pos ].z+Dist_th;
    sphere_coeff.values[3] = Dist_th;
    _viewer.addSphere (sphere_coeff, "sphere1",0);




    /// ===================================BUSCAMOS LOS EXTREMOS DE LA CABEZA
    /*  situamos los 4 posibles puntos donde podrían estar los hombros
        Ampliamos Dist_th*2 para separarnos del radio de la cabeza */
line_data data_line = locateLessPoint( cloud,  pos,  Dist_th,  z_aument);

//cout << "dirección con menos puntos" <<data_line.line_min<<endl;

line_start.x = data_line.point_array[0];
line_start.y = data_line.point_array[1];
line_start.z = data_line.point_array[2] + z_aument;

line_end.x = data_line.point_array[0];
line_end.y = data_line.point_array[1];
line_end.z = data_line.point_array[2] + 0.5;

_viewer.addLine(line_start,line_end,  255,0,0,"lineX",0);

    ///====================================BUSCAMOS LA DIRECCIÓN DE LA PERSONA



   /*
     float shoulder_center[3]; //punto central de los hombros (en la misma x e y que la posición mas alta)
     float shoulder_line[3];
     float less_points[3];
     float vector_shoulder_line[3];

     //punto que marca el centro de los hombros (punto mas alto menos altura aprox hombros)
     shoulder_center[0] = cloud->points[pos].x;
     shoulder_center[1] = cloud->points[pos].y;
     shoulder_center[2] = cloud->points[pos].z + z_aument;

     //punto que marca el lado que se ha seleccionado con menos puntos (frente o espaldas)
     less_points[0] = line_start.x;
     less_points[1] = line_start.y;
     less_points[2] = line_start.z;

    //linea que une la espalda con el centro de la cabeza
     line_start.x = less_points[0];
     line_start.y = less_points[1];
     line_start.z = less_points[2];

     line_end.x = shoulder_center[0];
     line_end.y = shoulder_center[1];
     line_end.z = shoulder_center[2];

    _viewer.addLine(line_start,line_end,  255,0,0,"line head-shoulder",0);

    //vector pendiente entre la perpendicular a la recta de los hombros



    vector_shoulder_line[0] = shoulder_center[1] - less_points[1];
    vector_shoulder_line[1] = -(shoulder_center[0] - less_points[0]);
    vector_shoulder_line[2] = shoulder_center[2] - less_points[2];

    cout << "COEFICIENTES PARA LA RECTA HOMBROS" << endl;
    cout << shoulder_center[0]<< endl;
    cout << shoulder_center[1]<< endl;
    cout << shoulder_center[2] << endl;
    cout << vector_shoulder_line[0] << endl;
    cout << vector_shoulder_line[1] << endl;
    cout << vector_shoulder_line[2] << endl;

    line_coeff.values[0]= 0;
    line_coeff.values[1]= 0;
    line_coeff.values[2]= 0;
    line_coeff.values[3]= vector_shoulder_line[0];
    line_coeff.values[4]= vector_shoulder_line[1];
    line_coeff.values[5]= vector_shoulder_line[2];


*/


  //  _viewer.addLine(line_coeff,"lineline",0);


//CALCULAR BIEN LOS COEFICIENTES PARA LA LINEA PERPENDICULAR!!!!!!!!!





    /// -------------añadir esfera







/// =============================================== DETECCIÓN DE LOS HOMBROS =========================================================///

pos_rad = 0;
searchPoint_HB.x = cloud->points[pos].x;
searchPoint_HB.y = cloud->points[pos].y;
searchPoint_HB.z = cloud->points[pos].z + 2*z_aument;


//cout << "radio con el que empieza a evaluar = " << radius <<endl;

do{


  //  contador = 0;


    if ( kdtree.radiusSearch (searchPoint_HB, radius, pointIdxRadiusSearch_HB, pointRadiusSquaredDistance_HB) > 0 )
      {

        ///cout<<"pointIdx...."<< pointIdxRadiusSearch_CB.size () << endl;

         for (size_t h = 0; h < pointIdxRadiusSearch_HB.size (); ++h)
          {
                contador++;
                if (pointRadiusSquaredDistance_HB[h] > Dist_th)
                     {Dist_th = pointRadiusSquaredDistance_HB[h];
                      pos_rad = h;
                     }
                if (contador > contador_max)
                {contador_max = contador;}
           }
      }


    radius = radius + 0.009;


    //cout << "radius hombros = " << radius << endl;
/*
    cout << "distancia q hay entre   donde ponemos el centro y la actual = " << fabs((cloud->points[pos].z + 2*z_aument) - cloud->points[pointIdxRadiusSearch_HB[pos_rad]].z) << endl;
    cout << "radio que llevamos = " << Dist_th<< endl;
    cout << "posicion en z con la que comparamos = " << (suelo-altura)/3 << endl;
    cout << "" <<endl;
*/



   }while (fabs((cloud->points[pos].z + 2*z_aument) - cloud->points[pointIdxRadiusSearch_HB[pos_rad]].z) < (suelo-altura)/4 );


sphere_coeff.values[0] = searchPoint_HB.x;
sphere_coeff.values[1] = searchPoint_HB.y;
sphere_coeff.values[2] = searchPoint_HB.z;
sphere_coeff.values[3] = Dist_th;
_viewer.addSphere (sphere_coeff, "sphere_hombros",0);


}
}



 if (USE_VIEWER)
  {
    _viewer.spin();
    _viewer_head.spin();
  }



if (head)
{
    cout << "borro circulo 1" << endl;
    if (USE_VIEWER)
    {
        sprintf(line_name, "line%c",line_max);
        //_viewer.removeShape("circle1",0);
       _viewer.removeShape("sphere1",0);
       _viewer.removeShape("sphere_hombros",0);
       // _viewer.removeShape(line_name);
        _viewer.removeShape("sphere_prueba");
 /*       _viewer.removeShape("line head-shoulder");
        _viewer.removeShape("line shoulder");*/
        _viewer.removeShape("lineX");
/*
        _viewer.removeShape("lineA1");
        _viewer.removeShape("lineA2");
        _viewer.removeShape("lineB1");
        _viewer.removeShape("lineB2");
        _viewer.removeShape("lineC1");
        _viewer.removeShape("lineC2");
        _viewer.removeShape("lineD1");
        _viewer.removeShape("lineD2");

*/
    }

}

 if (USE_VIEWER)
    {
     _viewer.removePointCloud("scene_cloud");
     _viewer_head.removePointCloud("cloud_head");
     _viewer_head.removeShape("sphere_head");
    }

//waitKey();


 if (USE_VIEWER)
 {
     _viewer.close();
     _viewer_head.close();
  }

}
return true_head;

}

