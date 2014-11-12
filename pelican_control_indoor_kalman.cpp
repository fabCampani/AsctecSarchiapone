/**
 
 2014 Carmine Tommaso Recchiuto - Unige, DIBRIS
 2014 Aste Edoardo, Campani Fabio

 (SPORADIC) EXPERT DEDICATED TO THE PATH FOLLOWING
 
 **/

#define n_accx  0.0080653 // *10
#define n_accy  0.0076799 // *10
#define n_accz  0.043222 // *100
#define n_posx  0.00000019239
#define n_posy  0.0000040953
#define n_posz  0.0000042075
#define n_velx  0.000041904
#define n_vely  0.00072175
#define n_velz  0.00087454
#define n_bias  0.001 

// definitions for the error (and therefore speed) limitation
#define ex_thr 2.5
#define ey_thr 2.5
#define ez_thr 2.5
#define integrativeok 1.5

//asctec
#include "asctecCommIntf.h"

//c
#include <unistd.h> // UNIX standard function definitions
#include <fcntl.h>  // File control definitions
#include <termios.h> // POSIX terminal control definitionss
#include <pthread.h>
#include <time.h>
#include <stdio.h>
#include <sys/time.h>

//aruco
#include "aruco.h"
#include "cvdrawingutils.h"

//c++
#include <cmath>
#include <fstream>
#include <sstream>
#include <iostream>

#include "module0.h"
#include "functions.h"

using namespace std;
using namespace cv;

struct timeval starttime2, endtime2;  // variables for the timing estimation

double dt = 0.035;
int gps_flag = 0;

float A_mat[9][9] = { { 1, dt, -(dt*dt) / 2, 0, 0, 0, 0, 0, 0 },
{ 0, 1, -dt, 0, 0, 0, 0, 0, 0 },
{ 0, 0, 1, 0, 0, 0, 0, 0, 0 },
{ 0, 0, 0, 1, dt, -(dt*dt) / 2, 0, 0, 0 },
{ 0, 0, 0, 0, 1, -dt, 0, 0, 0 },
{ 0, 0, 0, 0, 0, 1, 0, 0, 0 },
{ 0, 0, 0, 0, 0, 0, 1, dt, -(dt*dt) / 2 },
{ 0, 0, 0, 0, 0, 0, 0, 1, -dt },
{ 0, 0, 0, 0, 0, 0, 0, 0, 1 } };

float Q_mat[9][9] = { { n_accx*(dt*dt*dt*dt) / 4, n_accx*(dt*dt*dt) / 2, n_accx*(dt*dt) / 2, 0, 0, 0, 0, 0, 0 },
{ n_accx*(dt*dt*dt) / 2, n_accx*(dt*dt), n_accx*dt, 0, 0, 0, 0, 0, 0 },
{ n_accx*(dt*dt) / 2, n_accx*dt, n_accx, 0, 0, 0, 0, 0, 0 },
{ 0, 0, 0, n_accy*(dt*dt*dt*dt) / 4, n_accy*(dt*dt*dt) / 2, n_accy*(dt*dt) / 2, 0, 0, 0 },
{ 0, 0, 0, n_accy*(dt*dt*dt) / 2, n_accy*(dt*dt), n_accy*dt, 0, 0, 0 },
{ 0, 0, 0, n_accy*(dt*dt) / 2, n_accy*dt, n_accy, 0, 0, 0 },
{ 0, 0, 0, 0, 0, 0, n_accz*(dt*dt*dt*dt) / 4, n_accz*(dt*dt*dt) / 2, n_accz*(dt*dt) / 2 },
{ 0, 0, 0, 0, 0, 0, n_accz*(dt*dt*dt) / 2, n_accz*(dt*dt), n_accz*dt },
{ 0, 0, 0, 0, 0, 0, n_accz*(dt*dt) / 2, n_accz*dt, n_accz } };


float B_mat[9][3] = { { (dt*dt) / 2, 0, 0 },
{ dt, 0, 0 },
{ 1, 0, 0 },
{ 0, (dt*dt) / 2, 0 },
{ 0, dt, 0 },
{ 0, 1, 0 },
{ 0, 0, (dt*dt) / 2 },
{ 0, 0, dt },
{ 0, 0, 1 } };

float R_mat[9][9] = { { n_posx, 0, 0, 0, 0, 0, 0, 0, 0 },
{ 0, n_velx, 0, 0, 0, 0, 0, 0, 0 },
{ 0, 0, n_bias, 0, 0, 0, 0, 0, 0 },
{ 0, 0, 0, n_posy, 0, 0, 0, 0, 0 },
{ 0, 0, 0, 0, n_vely, 0, 0, 0, 0 },
{ 0, 0, 0, 0, 0, n_bias, 0, 0, 0 },
{ 0, 0, 0, 0, 0, 0, n_posz, 0, 0 },
{ 0, 0, 0, 0, 0, 0, 0, n_velz, 0 },
{ 0, 0, 0, 0, 0, 0, 0, 0, n_bias } };

cv::Mat Aa(9, 9, CV_32F, A_mat);
cv::Mat Aq(9, 9, CV_32F, Q_mat);
cv::Mat Ab(9, 3, CV_32F, B_mat);
cv::Mat Ah = Mat::eye(9, 9, CV_32F);
cv::Mat Ar(9, 9, CV_32F, R_mat);
cv::Mat Ap = Mat::zeros(9, 9, CV_32F);
cv::Mat Au(3, 1, CV_32F);
cv::Mat Ax(9, 1, CV_32F);
cv::Mat Ak = Mat::zeros(9, 9, CV_32F);
cv::Mat Ay(9, 1, CV_32F);

///GLOBAL VARIABLES (defined in module2.cpp). VARIABLES AND COMMAND FROM AND TO THE ROBOT (Serial Interface)
extern int16_t CTRL_pitch;
extern int16_t CTRL_roll;
extern int16_t CTRL_yaw;
extern u_int16_t CTRL_thrust;
extern int32_t angle_pitch;
extern int32_t angle_roll;
extern int32_t angle_yaw;
extern int initialize;
extern int ended;
extern int height;

extern int16_t acc_x;
extern int16_t acc_y;
extern int16_t acc_z;

extern int landing;
//////////////////////


#define LANDING 95

#define NPAST 30

//Tipo di funzione
int Nfunc1;
int Nfunc2;

//Velocità frame NED (debug)
double dxrDeb;
double dyrDeb;
double dzrDeb;
//Errori
double edx;
double edy;
double edz;

//Predisposizione filtro
double dxPast[NPAST];
double dyPast[NPAST];
double dzPast[NPAST];
int i;

double dxCurrent;
double dyCurrent;
double dzCurrent;


//Funzione di normalizzazione(OpenCV) {non utilizzata}
void normalize1(Mat mat){
	if (norm(mat,NORM_L2) != 0){
		mat = mat / norm(mat,NORM_L2);
	}
	else
		mat = 0.0 * mat;
}
//Funzione normallizzazione vettori
void normalize1(double *vett){
	double norm = sqrt(pow(vett[0], 2) + pow(vett[1], 2) + pow(vett[2], 2));
	if (norm == 0)
	{
		vett[0] = 0;
		vett[1] = 0;
		vett[2] = 0;
	}
	else
	{
		vett[0] = vett[0]/norm;
		vett[1] = vett[1]/norm;
		vett[2] = vett[2]/norm;
	}

}

void initializeVett(double *vett, int lenght){
	for (int i = 0; i < lenght; i++)
	{
		vett[i] = 0;
	}
}

double media(double *vett, int lenght){
	double mean = 0;
	for (int i = 0; i < lenght; i++)
	{
		mean += vett[i];
	}
	return(mean / lenght);
}

// debug function -- the parameters can be printed to check if they were correctly loaded.
void
Module0::printParams() {

}

//Caricamento parametri (da file params2.txt se esiste)
void
Module0::loadParams() {
	kpvx = -760;
	kpvy = 760;
	kpvz = -550.0;

	kpyaw = 1500.0;

	kdvx = 0.0;
	kdvy = 0.0;
	kdvz = 0.0;
	kdyaw = 0.0;
	
	kivx = 0;
	kivy = 0;
	kivz = -10;
	kiyaw = 0;	
	
	ke1 = -33;
	ke2 = -33;

	thre1 = 100;
	thre2 = 100;
	//Costante per componente tangente:
	ktg = 0.5;

	//velocità di marcia
	veld = 0.2;   // m/s

	//altezza del suolo
	ground = 1.45; // m

	//gravity compensation
	gravity = 2555;	//da tarare


	cumulx = 0,  // reset
	cumuly = 0,
	cumulz = 0,
	cumulyaw = 0;
	pitchOffset = 170;  // pitch offset, manually estimated
	rollOffset = 0;  // roll offset
	yawOffset = 0;

	xback = 0;
	yback = 0;
	zback = 0;
	//per derivativo errore
	edxback = 0;
	edyback = 0;
	edzback = 0;

	//Punto target per telecamera drone.
	x_target = 0;
	y_target = 0;

	//funzioni
	Nfunc1 = CIRCLE;
	Nfunc2 = PLANE;

	edx = 0;
	edy = 0;
	edz = 0;

	initializeVett(dxPast, NPAST);
	initializeVett(dyPast, NPAST);
	initializeVett(dzPast, NPAST);
	i = 0;

	//Carcamento parametri da file
	ifstream fs("params2.txt");	
	if (fs.is_open()){
		//file format:
		//1st line: kpx kpy kpz
		//2nd line: kix kiy kiz
		//3rd line: kdx kdy kdz
		//4th line: ktg speed
		//5th line: gravitycompensation pitchOffset
		//6th line: ke1 ke2 (always negative)
		fs >> kpvx;
		fs >> kpvy;
		fs >> kpvz;

		fs >> kivx;
		fs >> kivy;
		fs >> kivz;

		fs >> kdvx;
		fs >> kdvy;
		fs >> kdvz;

		fs >> ktg;
		fs >> veld;
		fs >> gravity;
		fs >> pitchOffset;

		fs >> ke1;
		fs >> ke2;
	}
}

//A text file to save the parameters is initialized with the variables to be saved
void
Module0::initializeDataSaving() {
	
	ofstream fs2("dataasctec.txt", std::ofstream::out | std::ofstream::trunc);

	if(fs2.is_open()){
        fs2 << "time \t";
		fs2 << "e1\t";
		fs2 << "e2\t";
		fs2 << "ctrlpitch\t";
		fs2 << "ctrlroll\t";
		fs2 << "ctrlthrust\t";
		fs2 << "ctrlyaw\t"; 
		fs2 << "edx" << "\t" << "edy" << "\t" << "edz" << "\t";
		fs2 << "cumulx" << "\t" << "cumuly" << "\t" << "cumulz" << "\t";
        fs2 << "x\t";
		fs2 << "y\t";
		fs2 << "z\t";
		fs2 << "xk\t";
		fs2 << "yk\t";
		fs2 << "zk\t";
        fs2 << "dx\t";
		fs2 << "dy\t";
		fs2 << "dz\t";
		fs2 << "dxk\t";
		fs2 << "dyk\t";
		fs2 << "dzk\t";
		fs2 << "dxd\t";
		fs2 << "dyd\t";
		fs2 << "dzd\t";
		//fs2 << "dxNED\t";
		//fs2 << "dyNED\t";
		//fs2 << "dzNED\t";
		fs2 << "pitch\t";
		fs2 << "roll\t";
		fs2 << "yaw\t";
		//Parameters
		fs2 << "kpvx\t";
		fs2 << "kpvy\t";
		fs2 << "kpvz\t";

		fs2 << "kivx\t";
		fs2 << "kivy\t";
		fs2 << "kivz\t";

		fs2 << "kdvx\t";
		fs2 << "kdvy\t";
		fs2 << "kdvz\t";

		fs2 << "ktg\t";
		fs2 << "veld\t";
		fs2 << "pitchoff\t";
		fs2 << "ke1\tke2\t";
		fs2 << "gravity\n";

		

		cout << "File with controls initialized" << endl;
	} else {
		cerr << "File not open!" << endl;
	}
	
}

Module0::Module0 (ETDispatch* dis)
:ETExpert(dis, 0, 0)
{
}

Module0::~Module0()
{
}

void
Module0::Init()
{
	message_rec = 0;
	AddRequest(1); // user interface
	AddRequest(2); // IMU
	AddRequest(3); // positions
 	SetActivationCondition(GetAllMsgTypeMask());
	loadParams(); 
	initializeDataSaving();
	gettimeofday(&starttime2, NULL);
	
	mtime=0; 
	accum_time=0;

	dxrDeb = 0;
	dyrDeb = 0;
	dzrDeb = 0;

	printstep = 70; // limits the visualization of the status on the terminal
	printTimeCounter = 0;
    gps_flag=0;
    limit = 25;  // limits the streaming of ETHNOS messages 
    count_ethnos=0;
}

void
Module0::DoYourDuty (int wc)
	{

	static double *R = new double[9];
	double accx, accy, accz;

    printTimeCounter++;

    theta = 0.001 * (double) angle_pitch *M_PI/180.0;  // Radiants
    phi = 0.001 * (double) angle_roll *M_PI/180.0;    // Radiants
	psi = 0.001 * (double)angle_yaw *M_PI / 180.0;  // Radiants

	if (wc)  return;
	
	while (MsgToBeRead())  // EHTNOS MESSAGE RECEIVED
	{
		const ETMessage *rxMsg = GetNextMsg();
		message_rec++;
		if (rxMsg->ReadType() == 1)
		{
			loadParams();
			cout << "********************************************" << endl;
			cout << "*         press P to take a picture        *" << endl;
			cout << "* press K to change the control parameters *" << endl;
			cout << "*          press S to stop the motors      *" << endl;
			cout << "*             press A to land              *" << endl;
			cout << "********************************************" << endl;
		}
		else if (rxMsg->ReadType() == 3)
		{
			xr = *((double*)rxMsg->ReadData());
			yr = *((double*)((char*)(rxMsg->ReadData()) + sizeof(double)));
			zr = *((double*)((char*)(rxMsg->ReadData()) + 2 * sizeof(double)));
			yawr = *((double*)((char*)(rxMsg->ReadData()) + 3 * sizeof(double)));
			dxr = *((double*)((char*)(rxMsg->ReadData()) + 4 * sizeof(double)));
			dyr = *((double*)((char*)(rxMsg->ReadData()) + 5 * sizeof(double)));
			dzr = *((double*)((char*)(rxMsg->ReadData()) + 6 * sizeof(double)));
			dyawr = *((double*)((char*)(rxMsg->ReadData()) + 7 * sizeof(double)));


			dxrDeb = dxr;
			dyrDeb = dyr;
			dzrDeb = dzr;
			
		}
	}
	RemoveCurrentMsg();

	// time calculation
	gettimeofday(&endtime2, NULL);
	secc = endtime2.tv_sec - starttime2.tv_sec;
	msec = endtime2.tv_usec - starttime2.tv_usec;
	mtime = ((secc)+(msec / 1000000.0)); //time in sec
	accum_time = accum_time + mtime;
	gettimeofday(&starttime2, NULL);
	dt = mtime;

	//Rotation matrix NED -> drone

	R[0] = cos(theta) * cos(yawr);
	R[3] = -cos(phi) * sin(yawr) + sin(phi) * sin(theta) * cos(yawr);
	R[6] = sin(phi) * sin(yawr) + cos(phi) * sin(theta) * cos(yawr);
	R[1] = cos(theta) * sin(yawr);
	R[4] = cos(phi) * cos(yawr) + sin(phi) * sin(theta) * sin(yawr);
	R[7] = -sin(phi) * cos(yawr) + cos(phi) * sin(theta) * sin(yawr);
	R[2] = -sin(theta);
	R[5] = sin(phi) * cos(theta);
	R[8] = cos(phi) * cos(theta);


	///Kalman prediction
	double ax = acc_x*9.81 / 10000;
	double ay = acc_y*9.81 / 10000;
	double az = acc_z*9.81 / 10000;

	//Moltiplichiamo per la trasposta di R per portarlo nel frame NED
	accx = R[0] * ax + R[3] * ay + R[6] * az;
	accy = R[1] * ax + R[4] * ay + R[7] * az;
	accz = R[2] * ax + R[5] * ay + R[8] * az;

	Au.at<float>(0, 0) = accx;
	Au.at<float>(1, 0) = accy;
	Au.at<float>(2, 0) = accz;

	Ax = Aa*Ax + Ab*Au;
	Ap = Aa*Ap*(Aa.t()) + Aq;     // prediction ok

	Ay.at<float>(0, 0) = xr;
	Ay.at<float>(1, 0) = dxr;
	Ay.at<float>(2, 0) = 0;
	Ay.at<float>(3, 0) = yr;
	Ay.at<float>(4, 0) = dyr;
	Ay.at<float>(5, 0) = 0;
	Ay.at<float>(6, 0) = zr;
	Ay.at<float>(7, 0) = dzr;
	Ay.at<float>(8, 0) = 0;

	//Kalman Update

	Ak = Ap*(Ah.t())*((Ah*Ap*(Ah.t()) + Ar).inv());
	Ax = Ax + Ak*(Ay - Ax);
	Ap = (Ah - Ak*Ah)*Ap;

	///End Kalman

	double xk, yk, zk, dxk, dyk, dzk;

	xk = Ax.at<float>(0, 0);
	dxk = Ax.at<float>(1, 0);
	yk = Ax.at<float>(3, 0);
	dyk = Ax.at<float>(4, 0);
	zk = Ax.at<float>(6, 0);
	dzk = Ax.at<float>(7, 0);

	//Kalman NED -> drone
	double dxr1 = R[0] * dxk + R[1] * dyk + R[2] * dzk;
	double dyr1 = R[3] * dxk + R[4] * dyk + R[5] * dzk;
	double dzr1 = R[6] * dxk + R[7] * dyk + R[8] * dzk;

	dxk = dxr1;
	dyk = dyr1;
	dzk = dzr1;

	//No Kalman NED -> drone
	dxr1 = R[0] * dxr + R[1] * dyr + R[2] * dzr;
	dyr1 = R[3] * dxr + R[4] * dyr + R[5] * dzr;
	dzr1 = R[6] * dxr + R[7] * dyr + R[8] * dzr;

	dxr = dxr1;
	dyr = dyr1;
	dzr = dzr1;

   
    if ((initialize==0)|(ended==1))
    {
        cumulx=0;
        cumuly=0;
        cumulz=0;
        cumulyaw=0;
    }        
    if ((initialize==1)&(ended==0)){
		
		/***CONTROLLO INIZIA QUI****/
		
		e1 = array_function[Nfunc1](xr, yr, zr);
		e2 = array_function[Nfunc2](xr, yr, zr);
		
		array_fgrad[Nfunc1](grad1, xr, yr, zr);
		array_fgrad[Nfunc2](grad2, xr, yr, zr);
		
		/*
		e1 = array_function[Nfunc1](xk, yk, zk);
		e2 = array_function[Nfunc2](xk, yk, zk);

		array_fgrad[Nfunc1](grad1, xk, yk, zk);
		array_fgrad[Nfunc2](grad2, xk, yk, zk);
		*/
		//Soglia errore
		if (e1 > thre1){
			e1 = thre1;
		}
		else if (e1 < -thre1){
			e1 = -thre1;
		}
		if (e2 > thre2){
			e2 = thre2;
		}
		else if (e2 < -thre2){
			e2 = -thre2;
		}
		
		//guadagno sull'errore
		e1 = ke1*e1;
		e2 = ke2*e2;
		
		//Prodotto vettore
		double Tang[]
		{
			- grad1[2] * grad2[1] + grad1[1] * grad2[2],
			grad1[2] * grad2[0] - grad1[0] * grad2[2],
			-grad1[1] * grad2[0] + grad1[0] * grad2[1]
		};

		//normalizzazione gradienti
		normalize1(grad1);
		normalize1(grad2);

		normalize1(Tang);

		//vettore risultante
		double SumNED[] = {
			e1*grad1[0] + e2*grad2[0] + ktg*Tang[0],
			e1*grad1[1] + e2*grad2[1] + ktg*Tang[1],
			e1*grad1[2] + e2*grad2[2] + ktg*Tang[2],
		};

		normalize1(SumNED);

		SumNED[0] *= veld;
		SumNED[1] *= veld;
		SumNED[2] *= veld;

		//NED -> drone
		dxd = R[0] * SumNED[0] + R[1] * SumNED[1] + R[2] * SumNED[2];
		dyd = R[3] * SumNED[0] + R[4] * SumNED[1] + R[5] * SumNED[2];
		dzd = R[6] * SumNED[0] + R[7] * SumNED[1] + R[8] * SumNED[2];
		

		/*ATTENZIONE!*/
		//Setup Controllore PID	(cancellare)	
		dxd = veld;
		dyd = 0;
		dzd = 0;


		if (landing == 1){
			dxd = 0;
			dyd = 0;
			dzd = 0;
		}
		//Calcolo errore: (velocità desiderata - misurata)
		/*
		edx = dxd - dxr;
		edy = dyd - dyr;
		edz = dzd - dzr;
		*/
		//Kalman
		
		edx = dxd - dxk;
		edy = dyd - dyk;
		edz = dzd - dzk;
		
		cumulx = cumulx + edx * mtime;
		cumuly = cumuly + edy * mtime;
		cumulz = cumulz + edz * mtime;
		
		//Derivativo:
		dedx = (edx - edxback) / mtime;
		dedy = (edy - edyback) / mtime;
		dedz = (edz - edzback) / mtime;

		edxback = edx;
		edyback = edy;
		edzback = edz;

        up = pitchOffset + kpvx*(edx) + kivx*(cumulx) + kdvx*(dedx); // pitch command
		
		ur = rollOffset + kpvy*(edy) + kivy*(cumuly)+ kdvy*(dedy); // roll command
		
		//Procedura di ATTERRAGGIO (non ancora testata)

		if (landing == 1)
		{
			if (zr > ground - 0.07){
				/*
				landing = 0;
				initialize = 0;
				ended = 1;
				ut = 1400;
				*/
				printf("atterrato\n");
			}
			else{
				if ((printTimeCounter == printstep))
					//printf("Procedura di atterraggio: Thrust: %f\n", ut);
				ut = ut - 0.2;
			}

		}else
		ut = gravity + kpvz*(edz) + kivz*(cumulz) + kdvz*(dedz); // thrust command calculation

		if (ut < 1700){
			ut = 1700;
		}
		
		//Controllo Yaw
		yawd = atan2(y_target, x_target);
		eyaw = sin(yawd - yawr);
		uy = kpyaw*(eyaw);

        // thresholds to avoid too fast movements
        int16_t CTRL_back;
        int16_t thr3 = 50;
        CTRL_back=CTRL_pitch;
        CTRL_pitch=(int16_t)up;
            
        if ((CTRL_pitch-CTRL_back)> thr3) {
            CTRL_pitch=CTRL_back+thr3;
        }
        else if((CTRL_pitch-CTRL_back)< -thr3) {
            CTRL_pitch=CTRL_back-thr3;
        }

        CTRL_back=CTRL_roll;
        CTRL_roll=(int16_t)ur;
            
        if ((CTRL_roll-CTRL_back)> thr3) {
            CTRL_roll=CTRL_back+thr3;
        }
        else if((CTRL_roll-CTRL_back)< -thr3) {
            CTRL_roll=CTRL_back-thr3;
        }
            
        CTRL_back=CTRL_thrust;
        CTRL_thrust=(int16_t)ut;
            
        if ((CTRL_thrust-CTRL_back)> thr3) {
            CTRL_thrust=CTRL_back+thr3;
        }
        else if((CTRL_thrust-CTRL_back)< -thr3) {
            CTRL_thrust=CTRL_back-thr3;
        }
            
        CTRL_back=CTRL_yaw;
        CTRL_yaw=(int16_t)uy;
            
        if ((CTRL_yaw-CTRL_back)> thr3) {
            CTRL_yaw=CTRL_back+thr3;
        }
        else if((CTRL_yaw-CTRL_back)< -thr3) {
            CTRL_yaw=CTRL_back-thr3;
            
        }
    }

    // data saving
	ofstream fs2("dataasctec.txt", std::ofstream::out | std::ofstream::app);
	
	if(fs2.is_open()){
	fs2 << accum_time << "\t";
	fs2 << e1 << "\t" << e2 << "\t";
	fs2 << CTRL_pitch << "\t" << CTRL_roll << "\t" << CTRL_thrust << "\t" << CTRL_yaw << "\t";
    //fs2 << longitude << "\t" << latitude << "\t" << height << "\t";
    //fs2 << fus_longitude << "\t" << fus_latitude << "\t" << fus_height << "\t";
    //fs2 << GPS_heading << "\t" << position_accuracy << "\t" << height_accuracy << "\t" << GPS_num << "\t" << GPS_status << "\t";
	fs2 << edx << "\t" << edy << "\t" << edz << "\t";
	fs2 << cumulx << "\t" << cumuly << "\t" << cumulz << "\t";
    fs2 << xr << "\t" << yr << "\t" << zr << "\t";
	fs2 << xk << "\t" << yk << "\t" << zk << "\t";
    fs2 << dxr << "\t" << dyr << "\t" << dzr<<"\t";
	fs2 << dxk << "\t" << dyk << "\t" << dzk << "\t";
	fs2 << dxd << "\t" << dyd << "\t" << dzd << "\t";
	//fs2 << dxrDeb << "\t" << dyrDeb << "\t" << dzrDeb << "\t";
    fs2 << theta << "\t" << phi << "\t" << yawr << "\t";
	fs2 << kpvx << "\t";
	fs2 << kpvy << "\t";
	fs2 << kpvz << "\t";

	fs2 << kivx << "\t";
	fs2 << kivy << "\t";
	fs2 << kivz << "\t";

	fs2 << kdvx << "\t";
	fs2 << kdvy << "\t";
	fs2 << kdvz << "\t";

	fs2 << ktg << "\t";
	fs2 << veld << "\t";
	fs2 << pitchOffset << "\t";
	fs2 << ke1 << "\t" << ke2 << "\t";
	fs2 << gravity << "\t";
    fs2 <<  "\n";

	}
	else {
	cerr << "File not open!" << endl;
	}

    // terminal visualization (debug)
        
   if((printTimeCounter == printstep)){
        //printf ("GPS DATA: long %d lat %d height %d", longitude, latitude, height);
        //printf ("gps_cartesian: long %f lat %f height %f \n", long_cart, lat_cart, height_cart);
        //printf ("  fus long %d lat %d height %d \n", fus_longitude, fus_latitude, fus_height);
        //printf ("GPS STATUS:  gps: heading: %d, yaw: %f, accuracy: %d, height accuracy: %d, sat: %d, status: %d  \n", GPS_heading, psi, position_accuracy, height_accuracy, GPS_num, GPS_status);       
		//printf("\n\nROBOT POSITION: x, y, z, yaw: %f %f %f %f\n", xr, yr, zr, yawr*180/M_PI);
        //printf("COMMANDS: command pitch, roll, thrust, yaw: %d %d %d %d \n", CTRL_pitch, CTRL_roll, CTRL_thrust, CTRL_yaw);
		//printf("COMMANDS: edx, edy, edz: %f %f %f \n", edx, edy, edz);
        //printf("VELOCITY   : dx, dy, dz: %f %f %f \n", dxr, dyr, dzr);
		//printf("VELOCITYNED: dx, dy, dz: %f %f %f \n", dxrDeb, dyrDeb, dzrDeb);
		//printf("ANGLES   : pitch, roll, yaw: %f %f %f \n", theta, phi, yawr);
		//printf("Rotation:\n %f\t %f\t %f\n %f\t %f\t %f\n%f\t %f\t %f\n", R[0], R[1], R[2], R[3], R[4], R[5], R[6], R[7], R[8]);
		//printf("DESIDERED VELOCITY: dx, dy, dz: %f %f %f \n", dxd, dyd, dzd);
		//printf("ERRORS: e1, e2 %f %f\n", e1, e2);
		//printf("YAW CTRL: yawd, yawr %f %f\n", yawd, yawr);
        printTimeCounter = 0;
    }
	
        
}	

void
Module0::Close()
{
    printParams();
	printf("\nSporadic N %d  ",ExpertId);
 	printf(" executed %d times ",message_rec);
    fflush(stdout);
}

