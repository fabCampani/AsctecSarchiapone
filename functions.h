#define maxFunct 2

#define CIRCLE 0
#define PLANE 1

double functionCircle(double x, double y, double z)
{
	double radius = 0.5; double center_x = -2.5; double center_y = 0;
	return (pow(x - center_x, 2) - pow(y - center_y, 2) - pow(radius, 2));
}

double functionPlane(double x, double y, double z)
{
	double k_x = 0; double k_y = 0; double k_z = 1;
	return (k_x * x + k_y * y + k_z * z);
}


void fgradCircle(double *res, double x, double y, double z){
	double radius = 0.5; double center_x = -2.5; double center_y = 0;
	res[0] = 2 * (x - center_x);
	res[1] = 2 * (y - center_y);
	res[2] = 0;
}

void fgradPlane(double *res, double x, double y, double z){
	double k_x = 0; double k_y = 0; double k_z = 1;
	res[0] = k_x;
	res[1] = k_y;
	res[2] = k_z;
}

double(*array_function[maxFunct])(double, double, double) = { functionCircle, functionPlane };
void(*array_fgrad[maxFunct])(double*, double, double, double) = { fgradCircle, fgradPlane };

