
int entrenasvm(double** sombrero,int nsombrero,double** persona,int npersona,double** objeto,int nobjeto);
int clasificasvm(double** vectores,int numvectores,double** vectores2,int numvectores2,double** sombrero,int nsombrero,double* candidato,Mat *Imageout,Point position,Point subroipos,int *resultado,int mn);
int clasificasdos(double** vectores,int numvectores,double** vectores2,int numvectores2,double* candidato,Mat Imageout,Point position);


