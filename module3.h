#include "const.h"
#include <stdio.h>
#include <ettimer.h>


class Module3 : public ETExpert
{
	private:
                long int message_sent;
		long int message_rec;
		int SendType;
		// variables for position extimation
	 	int averageCounter1;
	 	int averageCounter2;
		int maxCounter;
	 	int markerCounter;
		double *x_ext;
	 	double *y_ext;
	 	double *z_ext;
	 	double *yaw_ext;

	 	double *x_ext_back;
		double *y_ext_back;
	 	double *z_ext_back;
	 	double *yaw_ext_back;

	 	double *averagex;
		double *averagey;
		double *averagez;
		double *averageyaw;

		double xc, yc, zc;
		double xoc, yoc, zoc;
		//Pom: robot position rotated, i.e., wrt marker (before translation)
		double xom, yom, zom;

		double xr_back, yr_back, zr_back, aPsi_back;
		double dxr, dyr, dzr, dPsi;
		//rotations
		double theta, phi, psi, aPhi, aPsi;
		double xr, yr, zr;

	 	double thr1; //// threshold for opencv axes oscillations (y position)

	 	double dist;

	 	double markerx, markery, markerz, markeryaw;

	 	double start_average_x, start_average_y, start_average_z, start_average_yaw;

	 	double tempx;
		double tempy;
		double tempz;
		double tempyaw;
		
		double xmsg;
		double ymsg;
		double zmsg;
		double yawmsg;
		
		double dxmsg;
		double dymsg;
		double dzmsg;
		double dyawmsg;		

		double **measurements;

		double msec; 
		double mtime;
		double secc;
		double accum_time;

		int printTimeCounter;
		int printstep;
		
        public:
		 Module3 (ETDispatch*, long int per, int sm);
		 ~Module3();
		 void Init();
		 void Close();
		 void DoYourDuty(int wc=0);
};
