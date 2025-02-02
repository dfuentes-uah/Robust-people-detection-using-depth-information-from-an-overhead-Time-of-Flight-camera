#include <stdarg.h>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

using namespace std;
using namespace cv;


typedef struct
  {
    int cont_ok;
      cv::Mat mat_contornos;
      vector < vector < Point > >contours;
      vector < Point2f > center_count;
      vector < float >radius_count;
  } struct_count;

typedef struct
  {
    cv::Mat mat_contorno;
    float radio;
    float altura_cont;

  } datos_contorno;

typedef struct
  {
    Mat ROI;
    Point2f rect_start;
    Point2f rect_add;

  } data_ROI;


void Calculo_histograma(cv::Mat & mat, cv::Mat & mask, char *nombre);
struct_count Calculo_contorno(cv::Mat mat1, cv::Mat mat2);
void cut_contour(struct_count data_contours, int numc, Mat mat1,
                 int num_frame);
double getOrientation(vector < Point > &pts, Mat & img);
Mat Calculo_gradiente(Mat & original);
datos_contorno alone_contour(Mat & dist_mat, Mat & original_aux,
                             struct_count contornos, int numc);
char calculo_max_values4(int A, int B, int C, int D);
char calculo_min_values4(int A, int B, int C, int D);
char *calculo_min_values6(int A1, int B1, int C1, int D1, int A2, int B2,
                          int C2, int D2);

struct_count contador_contour(Mat & original);

float sphera_point_distance(float xo, float yo, float zo, float radio,
                            float px, float py, float pz);

float nnSearch(Mat & original, int x, int y, int width);

data_ROI cutROI(Mat & original, Point2f center, float radius);
Mat pasteROI(Mat & original, data_ROI analizeROI);
Point2f maxPointMat(Mat & original);
