#define maxFunct 9

#define CIRCLE_XY	0
#define CIRCLE_YZ	1 
#define CIRCLE_XZ	2
#define PLANE_Z		3
#define PLANE_Y		4
#define PLANE_X		5
#define PLANE_GEN	6
#define ELLIPSE		7
#define SPHERE		8


///CODICE CERCHIO XY
double circleXY_radius = 0.4;
double circleXY_center_x = -2;
double circleXY_center_y = -0.2;


double functionCircleXY(double x, double y, double z)
{
	return (pow(x - circleXY_center_x, 2) + pow(y - circleXY_center_y, 2) - pow(circleXY_radius, 2));
}
void fgradCircleXY(double *res, double x, double y, double z){
	res[0] = 2 * (x - circleXY_center_x);
	res[1] = 2 * (y - circleXY_center_y);
	res[2] = 0;
}

///CODICE CERCHIO YZ
double circleYZ_radius = 0.4;
double circleYZ_center_y = -0.2;
double circleYZ_center_z = 0;

double functionCircleYZ(double x, double y, double z)
{
	return (pow(y - circleYZ_center_y, 2) + pow(z - circleYZ_center_z, 2) - pow(circleYZ_radius, 2));
}
void fgradCircleYZ(double *res, double x, double y, double z){
	res[0] = 0;
	res[1] = 2 * (y - circleYZ_center_y);
	res[2] = 2 * (z - circleYZ_center_z);
}

///CODICE CERCHIO XZ
double circleXZ_radius = 0.4;
double circleXZ_center_x = -2;
double circleXZ_center_z = 0;

double functionCircleXZ(double x, double y, double z)
{
	return (pow(x - circleXZ_center_x, 2) + pow(z - circleXZ_center_z, 2) - pow(circleXZ_radius, 2));
}
void fgradCircleXZ(double *res, double x, double y, double z){
	res[0] = 2 * (x - circleXZ_center_x);
	res[1] = 0;
	res[2] = 2 * (z - circleXZ_center_z);
}


///CODICE PIANO Z=0
double offset_plane_z = 0;
double functionPlaneZ(double x, double y, double z)
{
	return (z - offset_plane_z);
}
void fgradPlaneZ(double *res, double x, double y, double z){
	res[0] = 0;
	res[1] = 0;
	res[2] = 1;
}

///CODICE PIANO Y=0
double offset_plane_y = 0;
double functionPlaneY(double x, double y, double z)
{
	return (y - offset_plane_y);
}
void fgradPlaneY(double *res, double x, double y, double z){
	res[0] = 0;
	res[1] = 1;
	res[2] = 0;
}

///CODICE PIANO X=0
double offset_plane_x = -2.0;
double functionPlaneX(double x, double y, double z)
{
	return (x - offset_plane_x);
}
void fgradPlaneX(double *res, double x, double y, double z){
	res[0] = 1;
	res[1] = 0;
	res[2] = 0;
}


///CODICE PIANO GENERICO
double k_x = 0;
double k_y = -0.3;
double k_z = 1;
double offset=-0.2;

double functionPlaneGen(double x, double y, double z)
{
	return (k_x * x + k_y * y + k_z * z + offset);
}
void fgradPlaneGen(double *res, double x, double y, double z){
	res[0] = k_x;
	res[1] = k_y;
	res[2] = k_z;
}




//CODICE ELLISSE
double a = 1;	//Semi-axis X
double b = 0.6;	//Semi-axis Y

double ellipse_center_x = -0.5;
double ellipse_center_y = -2.25;

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

//CODICE SFERA
double sphere_center_x = -2;
double sphere_center_y = -0.5;
double sphere_center_z = -0;
double sphere_radius = 0.4;

double functionSphere(double x, double y, double z)
{
	return (pow(x - sphere_center_x, 2) + pow(y - sphere_center_y, 2) + pow(z - sphere_center_z, 2) - pow(sphere_radius, 2));
}
void fgradSphere(double *res, double x, double y, double z){
	res[0] = 2 * (x - sphere_center_x);
	res[1] = 2 * (y - sphere_center_y);
	res[2] = 2 * (z - sphere_center_z);
}



double(*array_function[maxFunct])(double, double, double) = {	functionCircleXY,
																functionCircleYZ,
																functionCircleXZ,
																functionPlaneZ, 
																functionPlaneY,
																functionPlaneX,
																functionPlaneGen,
																functionEllipse,
																functionSphere };
void(*array_fgrad[maxFunct])(double*, double, double, double) = {	fgradCircleXY,
																	fgradCircleYZ,
																	fgradCircleXZ,
																	fgradPlaneZ,
																	fgradPlaneY,
																	fgradPlaneX,
																	fgradPlaneGen,
																	fgradEllipse,
																	fgradSphere };
