//para operaciones opencv
#include <stdio.h>
#include <pmdsdk2.h>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/contrib/contrib.hpp>
#include <opencv2/core/core.hpp>





#include <pcl/ModelCoefficients.h>

#include <pcl/io/vtk_io.h>
#include <pcl/console/parse.h>
#include <pcl/visualization/histogram_visualizer.h>

#include <boost/thread/thread.hpp>
#include <pcl/point_types.h>
#include <pcl/features/vfh.h>

#include <pcl/features/normal_3d.h>

#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>

#include "Funciones_opencv.h"



typedef pcl::PointXYZ PointT;
using namespace cv;
using namespace pcl;
using namespace std;

typedef struct
{
    char *line_min;
    float point_array[3];
}line_data;


typedef struct
{
    float x;
    float y;
    float z;
    int pos;
    int posz;
}posHighest;

typedef struct
{
    float x;
    float y;
}RelationDimCloudData;




///*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
///+		     FUNCIÓN LOCALIZA MÁXIMO EN LA NUBE DE PUNTOS               +
///+                                                                        +
///+	Argumentos que recibe:                                              +
///+	- cloud: nube de puntos a analizar                                  +
///+	Argumentos que devuelve:                                            +
///+    - Highest_point: estructura con las posiciones del punto mas alto  	+
///+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/


posHighest LocateMaxCloud (pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{

    posHighest Highest_point;

    cout << "entra en la funcion LocateMaxCloud" <<endl;

    float width_cl = cloud->width;
    float height_cl = cloud->height;



    float min_z = 10000;
    float max_z = 0;
    float max_x = 0;
    float max_y = 0;
    float diff_z;
    int pos, posx = -1, posy = -1, posz = -1;

    for (int i=0; i<width_cl*height_cl; i++)
    {


       diff_z = cloud->points[i].z;

       if (diff_z < min_z && diff_z != 0) //posicion mas alta (cabeza)
        {min_z = diff_z;
             pos = i;}

       if (diff_z > max_z) //posicion mas baja (suelo)
        {max_z = diff_z;
         posz = i;}
       if (cloud->points[i].x > max_x) //posicion mas alejada en x
        {max_x = cloud->points[i].x;
         posx=i;}
       if (cloud->points[i].y > max_y) //posicion mas alejada en y
        {max_y = cloud->points[i].y;
         posy=i;}
    }



    cout << cloud->points[pos].x << endl;
    cout <<cloud->points[pos].y << endl;
    cout  <<cloud->points[pos].z << endl;
    cout  <<cloud->points[posz].z << endl;

    Highest_point.x = cloud->points[pos].x;
    Highest_point.y = cloud->points[pos].y;
    Highest_point.z = cloud->points[pos].z;
    Highest_point.pos = pos;
    Highest_point.posz = posz;



    cout <<"localiza los puntos"<< endl;


    return Highest_point;
}


RelationDimCloudData relationDimCloud (pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,float *x2D, float *y2D, float *z2D, float size_array)
{
    RelationDimCloudData dataRelDimPix;
    float max_x = 0;
    float min_x = 1000;
    float value_x;

    float max_y = 0;
    float min_y = 1000;
    float value_y;

    for(int i=0; i < size_array; i++)
    {   value_x = x2D[i];
        if(value_x < min_x)
            min_x =  value_x;
        if(value_x > max_x)
            max_x =  value_x;
    }

    for(int j=0; j < size_array; j++)
    {   value_y = y2D[j];
        if(value_y < min_y)
            min_y=  value_y;
        if(value_y > max_y)
            max_y =  value_y;
    }

    float dimReal_x = max_x - min_x;
    float dimReal_y = max_y - min_y;

    float relDimPix_x = (cloud->width / dimReal_x);
    float relDimPix_y = (cloud->height / dimReal_y);

    dataRelDimPix.x = relDimPix_x;
    dataRelDimPix.y = relDimPix_y;

    return dataRelDimPix;

}





///*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
///+		FUNCIÓN LOCALIZA LA RECTA CON MENOS PUNTOS JUNTOS               +
///+                                                                        +
///+	Argumentos que recibe:                                              +
///+	- cloud: nube de puntos a analizar                                  +
///+	- pos: posición más alta de la imágen                           	+
///+	- Dist_th: radio estimado de la cabeza para situar                  +
///+                los puntos de las rectas                                +
///+    - z_aument: variación que tenemos que hacer en z para localizar     +
///                 los hombros                                         	+
///+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/

line_data locateLessPoint(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int pos, float Dist_th, float z_aument)

{
    line_data datos;
    KdTreeFLANN<PointXYZ> kdtree;
    kdtree.setInputCloud(cloud);
    PointXYZ searchPoint_HB;

    std::vector<int> pointIdxRadiusSearch_HB;
    std::vector<float> pointRadiusSquaredDistance_HB;

    float a1_pos[3], b1_pos[3], c1_pos[3], d1_pos[3], a2_pos[3], b2_pos[3], c2_pos[3], d2_pos[3];//4 de los extremos del círculo de la cabeza

     /// ===================================BUSCAMOS LOS EXTREMOS DE LA CABEZA
     /*  situamos los 4 posibles puntos donde podrían estar los hombros
         Ampliamos Dist_th*2 para separarnos del radio de la cabeza */
     a1_pos[0] = cloud->points[pos].x;
     a1_pos[1] = cloud->points[pos].y + (Dist_th*(3/2));
     a1_pos[2] = cloud->points[pos].z;

     a2_pos[0] = a1_pos[0];
     a2_pos[1] = cloud->points[pos].y - (Dist_th*(3/2));
     a2_pos[2] = a1_pos[2];

     b1_pos[0] = cloud->points[pos].x + 0.7071*(Dist_th*(3/2));
     b1_pos[1] = cloud->points[pos].y + 0.7071*(Dist_th*(3/2));
     b1_pos[2] = cloud->points[pos].z;

     b2_pos[0] = cloud->points[pos].x - 0.7071*(Dist_th*(3/2));
     b2_pos[1] = cloud->points[pos].y - 0.7071*(Dist_th*(3/2));
     b2_pos[2] = cloud->points[pos].z;

     c1_pos[0] = cloud->points[pos].x + (Dist_th*(3/2));
     c1_pos[1] = cloud->points[pos].y;
     c1_pos[2] = cloud->points[pos].z;

     c2_pos[0] = cloud->points[pos].x - (Dist_th*(3/2));
     c2_pos[1] = cloud->points[pos].y;
     c2_pos[2] = cloud->points[pos].z;

     d1_pos[0] = cloud->points[pos].x + 0.7071*(Dist_th*(3/2));
     d1_pos[1] = cloud->points[pos].y - 0.7071*(Dist_th*(3/2));
     d1_pos[2] = cloud->points[pos].z;

     d2_pos[0] = cloud->points[pos].x - 0.7071*(Dist_th*(3/2));
     d2_pos[1] = cloud->points[pos].y + 0.7071*(Dist_th*(3/2));
     d2_pos[2] = cloud->points[pos].z;

     int puntosA1 = 0;
     int puntosA2 = 0;
     int puntosB1= 0;
     int puntosB2= 0;
     int puntosC1= 0;
     int puntosC2= 0;
     int puntosD1= 0;
     int puntosD2= 0;

     int contador = 0;


     ///------------------------------direccion A1!!!

         contador = 0;

         searchPoint_HB.x = a1_pos[0];
         searchPoint_HB.y = a1_pos[1];
         searchPoint_HB.z = a1_pos[2] + z_aument;



         if ( kdtree.radiusSearch (searchPoint_HB, 0.08, pointIdxRadiusSearch_HB, pointRadiusSquaredDistance_HB) > 0 )

         {
             for (size_t ha = 0; ha < pointIdxRadiusSearch_HB.size (); ++ha)
              {
               contador++;
               if (contador++ > puntosA1)
                puntosA1 = contador;
             }
         }

     ///------------------------------direccion A2!!!
         contador = 0;

         searchPoint_HB.x = a2_pos[0];
         searchPoint_HB.y = a2_pos[1];
         searchPoint_HB.z = a2_pos[2] + z_aument;

         if ( kdtree.radiusSearch (searchPoint_HB, 0.08, pointIdxRadiusSearch_HB, pointRadiusSquaredDistance_HB) > 0 )

         {
             for (size_t ha = 0; ha < pointIdxRadiusSearch_HB.size (); ++ha)
              {
               contador++;
               if (contador++ > puntosA2)
                puntosA2 = contador;
             }
         }


     ///-----------------------------direccion B1!!!


         contador = 0;

         searchPoint_HB.x = b1_pos[0];
         searchPoint_HB.y = b1_pos[1];
         searchPoint_HB.z = b1_pos[2] + z_aument;



         if ( kdtree.radiusSearch (searchPoint_HB, 0.08, pointIdxRadiusSearch_HB, pointRadiusSquaredDistance_HB) > 0 )

         {
             for (size_t hb = 0; hb < pointIdxRadiusSearch_HB.size (); ++hb)
              {
                contador++;
                if (contador++ > puntosB1)
                    puntosB1 = contador;
              }
                }

     ///--------------------------direccion B2!!!
         contador = 0;

         searchPoint_HB.x = b2_pos[0];
         searchPoint_HB.y = b2_pos[1];
         searchPoint_HB.z = b2_pos[2] + z_aument;



         if ( kdtree.radiusSearch (searchPoint_HB, 0.08, pointIdxRadiusSearch_HB, pointRadiusSquaredDistance_HB) > 0 )

         {
             for (size_t hb = 0; hb < pointIdxRadiusSearch_HB.size (); ++hb)
              {
                contador++;
                if (contador++ > puntosB2)
                    puntosB2 = contador;
              }
                }
     ///--------------------------direccion C1!!!

         contador = 0;

         searchPoint_HB.x = c1_pos[0];
         searchPoint_HB.y = c1_pos[1];
         searchPoint_HB.z = c1_pos[2] + z_aument;


         if ( kdtree.radiusSearch (searchPoint_HB, 0.08, pointIdxRadiusSearch_HB, pointRadiusSquaredDistance_HB) > 0 )
         {
             for (size_t hc = 0; hc < pointIdxRadiusSearch_HB.size (); ++hc)
              {
                contador++;
                if (contador++ > puntosC1)
                    puntosC1 = contador;
              }
          }

     ///--------------------------direccion C2!!!

         contador = 0;

         searchPoint_HB.x = c2_pos[0];
         searchPoint_HB.y = c2_pos[1];
         searchPoint_HB.z = c2_pos[2] + z_aument;


         if ( kdtree.radiusSearch (searchPoint_HB, 0.08, pointIdxRadiusSearch_HB, pointRadiusSquaredDistance_HB) > 0 )
         {
             for (size_t hc = 0; hc < pointIdxRadiusSearch_HB.size (); ++hc)
              {
                contador++;
                if (contador++ > puntosC2)
                    puntosC2 = contador;
              }
          }

     ///--------------------------direccion D1!!!

             contador = 0;
             searchPoint_HB.x = d1_pos[0];
             searchPoint_HB.y = d1_pos[1];
             searchPoint_HB.z = d1_pos[2] + z_aument;



           if ( kdtree.radiusSearch (searchPoint_HB, 0.08, pointIdxRadiusSearch_HB, pointRadiusSquaredDistance_HB) > 0 )
           {
              for (size_t hd = 0; hd < pointIdxRadiusSearch_HB.size (); ++hd)
               {
                contador++;
                 if (contador++ > puntosD1)
                    puntosD1 = contador;
               }
            }

     ///--------------------------direccion D2!!!

           contador = 0;
           searchPoint_HB.x = d2_pos[0];
           searchPoint_HB.y = d2_pos[1];
           searchPoint_HB.z = d2_pos[2] + z_aument;



         if ( kdtree.radiusSearch (searchPoint_HB, 0.08, pointIdxRadiusSearch_HB, pointRadiusSquaredDistance_HB) > 0 )
         {
            for (size_t hd = 0; hd < pointIdxRadiusSearch_HB.size (); ++hd)
             {
              contador++;
               if (contador++ > puntosD2)
                  puntosD2 = contador;
             }
          }


/*
         cout << "numero de puntos en cada segmento = " << endl;
         cout << "puntos en A1 = " << puntosA1 << endl;
         cout << "puntos en A2 = " << puntosA2 << endl;
         cout << "puntos en B1 = " << puntosB1 << endl;
         cout << "puntos en B2 = " << puntosB2 << endl;
         cout << "puntos en C1 = " << puntosC1 << endl;
         cout << "puntos en C2 = " << puntosC2 << endl;
         cout << "puntos en D1 = " << puntosD1 << endl;
         cout << "puntos en D2 = " << puntosD2 << endl;
*/

         char *letra = calculo_min_values6(puntosA1, puntosB1, puntosC1, puntosD1,puntosA2, puntosB2, puntosC2, puntosD2);
         cout << letra << endl;

         datos.line_min = letra;

         if (strcmp(letra,"A1"))
          {
             datos.point_array[0] = a1_pos[0];
             datos.point_array[1] = a1_pos[1];
             datos.point_array[2] = a1_pos[2];
          }

         if (strcmp(letra,"A2"))
          {
             datos.point_array[0] = a2_pos[0];
             datos.point_array[1] = a2_pos[1];
             datos.point_array[2] = a2_pos[2];
          }

         if (strcmp(letra,"B1"))
          {
             datos.point_array[0] = b1_pos[0];
             datos.point_array[1] = b1_pos[1];
             datos.point_array[2] = b1_pos[2];
          }

           if (strcmp(letra,"B2"))
           {
               datos.point_array[0] = b2_pos[0];
               datos.point_array[1] = b2_pos[1];
               datos.point_array[2] = b2_pos[2];
           }


           if (strcmp(letra,"C1"))
           {
               datos.point_array[0] = c1_pos[0];
               datos.point_array[1] = c1_pos[1];
               datos.point_array[2] = c1_pos[2];
           }

           if (strcmp(letra,"C2"))
           {
               datos.point_array[0] = c2_pos[0];
               datos.point_array[1] = c2_pos[1];
               datos.point_array[2] = c2_pos[2];
           }

           if (strcmp(letra,"D1"))
           {
               datos.point_array[0] = d1_pos[0];
               datos.point_array[1] = d1_pos[1];
               datos.point_array[2] = d1_pos[2];
           }

           if (strcmp(letra,"D2"))
           {
               datos.point_array[0] = d2_pos[0];
               datos.point_array[1] = d2_pos[1];
               datos.point_array[2] = d2_pos[2];
           }




return datos;
    ///----------------------SWITCH PARA CALCULAR LOS PARÁMETROS DE LA RECTA

}
