#define maxFunct 4

#define CIRCLE 0
#define PLANE 1
#define PLANE2 2
#define ELLIPSE 3


///CODICE CERCHIO
double radius = 0.01;
double center_x = -2.5;
double center_y = -0.0;

double functionCircle(double x, double y, double z)
{
	return (pow(x - center_x, 2) + pow(y - center_y, 2) - pow(radius, 2));
}

void fgradCircle(double *res, double x, double y, double z){
	res[0] = 2 * (x - center_x);
	res[1] = 2 * (y - center_y);
	res[2] = 0;
}


///CODICE PIANO Z=0
double functionPlane(double x, double y, double z)
{
	double k_x = 0; double k_y = 0; double k_z = 1;
	return (k_x * x + k_y * y + k_z * z);
}
void fgradPlane(double *res, double x, double y, double z){
	double k_x = 0; double k_y = 0; double k_z = 1;
	res[0] = k_x;
	res[1] = k_y;
	res[2] = k_z;
}



///CODICE PIANO Y=0
double functionPlane2(double x, double y, double z)
{
	double k_x = 0; double k_y = 1; double k_z = 0;
	return (k_x * x + k_y * y + k_z * z);
}
void fgradPlane2(double *res, double x, double y, double z){
	double k_x = 0; double k_y = 1; double k_z = 0;
	res[0] = k_x;
	res[1] = k_y;
	res[2] = k_z;
}


//CODICE ELLISSE
double a = 0.5;	//Semi-major axis
double b = 1;	//Semi-minor axis

double ellipse_center_x = -2;
double ellipse_center_y = -1;

double functionEllipse(double x, double y, double z)
{
	return (pow(x - ellipse_center_x, 2) / pow(a, 2) + pow(y - ellipse_center_y, 2) / pow(b, 2) - 1);
}

void fgradEllipse(double *res, double x, double y, double z)
{
	res[0] = (2 / pow(a, 2))*(x - ellipse_center_x);
	res[1] = (2 / pow(b, 2))*(y - ellipse_center_y);
	res[2] = 0;
}
double(*array_function[maxFunct])(double, double, double) = { functionCircle, functionPlane, functionPlane2, functionEllipse };
void(*array_fgrad[maxFunct])(double*, double, double, double) = { fgradCircle, fgradPlane, fgradPlane2, fgradEllipse };

