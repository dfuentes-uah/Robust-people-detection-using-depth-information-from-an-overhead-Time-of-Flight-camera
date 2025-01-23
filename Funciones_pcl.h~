
using namespace std;
using namespace cv;


typedef struct
  {
    char *line_min;
    float point_array[3];

  } line_data;


typedef struct
  {
    float x;
    float y;
    float z;
    int pos;
    int posz;
  } posHighest;



typedef struct
  {
    float x;
    float y;
  } RelationDimCloudData;

line_data locateLessPoint(pcl::PointCloud < pcl::PointXYZ >::Ptr cloud,
                          int pos, float Dist_th, float z_aument);
posHighest LocateMaxCloud(pcl::PointCloud < pcl::PointXYZ >::Ptr cloud);
RelationDimCloudData relationDimCloud(pcl::PointCloud <
                                      pcl::PointXYZ >::Ptr cloud, float *x2D,
                                      float *y2D, float *z2D,
                                      float size_array);
