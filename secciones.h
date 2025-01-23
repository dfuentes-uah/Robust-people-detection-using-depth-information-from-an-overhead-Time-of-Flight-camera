int seccion(std :: vector < Mat > smallImages,cv::Point minLoc,double minVal,int posicion,int secciones[20], vector<Point> *extremosHead, vector<Point> *puntosCabeza);
int seccionAgrup (int secciones2cm[20], int secciones6cm[5]);
float headAxis (vector<Point> extremosHead, vector<Point> extremosHeadm, Mat img);
Mat Gradiente(Mat &original);
int normalizationVector(float **points, double **pointNorm, int numMax);
