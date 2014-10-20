/**
 
 2014 Carmine Tommaso Recchiuto - Unige, DIBRIS
 2014 Aste Edoardo, Campani Fabio

 (SPORADIC) EXPERT DEDICATED TO THE PATH FOLLOWING
 
 **/

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

using namespace std;
using namespace cv;

struct timeval starttime2, endtime2;  // variables for the timing estimation

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
extern int landing;
//////////////////////

#define LANDING 95

double Module0::function1(double x, double y, double z)
{
	return (y);
}

double Module0::function2(double x, double y, double z)
{
	return (z);
}
//Calcolo dei gradienti
void Module0::fgrad1(double x, double y, double z, double *res){
	res[0] = 0;
	res[1] = 1;
	res[2] = 0;
}

void Module0::fgrad2(double x, double y, double z, double *res){
	res[0] = 0;
	res[1] = 0;
	res[2] = 1;
}

void normalize1(Mat mat){
	if (norm(mat,NORM_L2) != 0){
		mat = mat / norm(mat,NORM_L2);
	}
	else
		mat = 0.0 * mat;
}

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
		vett[0] = vett[0] / norm;
		vett[1] = vett[1]/norm;
		vett[2] = vett[2]/norm;
	}

}

// debug function -- the parameters can be printed to check if they were correctly loaded.
void
Module0::printParams() {

}

// function used to load control parameters (from "firefly_params_out.txt" and "pelican_params_out.txt" from the Build folder)
// When new params are loaded, the integrative action is set to zero.
void
Module0::loadParams() {
	kpvx = -550;
	kpvy = 550;
	kpvz = -200.0;

	kpyaw = 1500.0;

	kdvx = 0.0;
	kdvy = 0.0;
	kdvz = 1000.0;
	kdyaw = 0;
	
	kivx = 0;
	kivy = 0;
	kivz = -0.5;
	kiyaw = 0;	
	
	ke1 = -1;
	ke2 = -1;
	thre1 = 100;
	thre2 = 100;
	ktg = 0.5;	
	veld = 0.5;   //* m/s

	ground = 1; //m

	gravity = 2200;	//da tarare

	cumulx = 0,  // reset
	cumuly = 0,
	cumulz = 0,
	cumulyaw = 0;
	pitchOffset = 0;  // pitch offset, manually estimated
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
	/*
	ifstream fs("pelican_params_out.txt");
	if (fs.is_open()){
		//file format:
		//1st line: thrustOffset pitchOffset rollOffset yawOffset
		//2nd line: kpx kpy kpz kpyaw
		//3rd line: kdx kdy kdz kdyaw
		//4th line: kix kiy kiz kiyaw
		fs >> thrustOffset;
		fs >> pitchOffset;
		fs >> rollOffset;
		fs >> yawOffset;
		fs >> kpx;
		fs >> kpy;
		fs >> kpz;
		fs >> kpyaw;
		fs >> kdx;
		fs >> kdy;
		fs >> kdz;
		fs >> kdyaw;
		fs >> kix;
		fs >> kiy;
		fs >> kiz;
		fs >> kiyaw;
		cout << endl << "Params loaded" << endl;
	}
	else {
		cerr << endl << "File not found!" << endl;
	}
	*/
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
        fs2 << "x\t";
		fs2 << "y\t";
		fs2 << "z\t";
        fs2 << "dx\t";
		fs2 << "dy\t";
		fs2 << "dz\t";
		fs2 << "dxNED\t";
		fs2 << "dyNED\t";
		fs2 << "dzNED\t";
		fs2 << "pitch\t";
		fs2 << "roll\t";
		fs2 << "yaw\n";

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

	printstep = 70; // limits the visualization of the status on the terminal
	printTimeCounter = 0;
    gps_flag=0;
    limit = 25;  // limits the streaming of ETHNOS messages 
    count_ethnos=0;
}

void
Module0::DoYourDuty (int wc)
	{

	double dxrDeb=0;
	double dyrDeb=0;
	double dzrDeb=0;

	static double *R = new double[9];

    printTimeCounter++; 
           
    theta = 0.001 * (double) angle_pitch *M_PI/180.0;  // Radiants
    phi = 0.001 * (double) angle_roll *M_PI/180.0;    // Radiants
	psi = 0.001 * (double)angle_yaw *M_PI / 180.0;  // Radiants -> Da controllare
  
	if (wc)  return;
	
	while (MsgToBeRead())  // EHTNOS MESSAGE RECEIVED
	{
		const ETMessage *rxMsg = GetNextMsg();
		message_rec++;
		if (rxMsg->ReadType() == 3)
		{
			xr = *((double*)rxMsg->ReadData());
			yr = *((double*)((char*)(rxMsg->ReadData()) + sizeof(double)));
			zr = *((double*)((char*)(rxMsg->ReadData()) + 2 * sizeof(double)));
			yawr = *((double*)((char*)(rxMsg->ReadData()) + 3 * sizeof(double)));
			dxr = *((double*)((char*)(rxMsg->ReadData()) + 4 * sizeof(double)));
			dyr = *((double*)((char*)(rxMsg->ReadData()) + 5 * sizeof(double)));
			dzr = *((double*)((char*)(rxMsg->ReadData()) + 6 * sizeof(double)));
			/*dxr = (xr - xback) / mtime;
			xback = xr;
			dyr = (yr - yback) / mtime;
			yback = yr;

			dzr = (zr - zback) / mtime;
			zback = zr;
			*/
			dyawr = *((double*)((char*)(rxMsg->ReadData()) + 7 * sizeof(double)));

			R[0] = cos(theta) * cos(yawr);
			R[3] = -cos(phi) * sin(yawr) + sin(phi) * sin(theta) * cos(yawr);
			R[6] = sin(phi) * sin(yawr) + cos(phi) * sin(theta) * cos(yawr);
			R[1] = cos(theta) * sin(yawr);
			R[4] = cos(phi) * cos(yawr) + sin(phi) * sin(theta) * sin(yawr);
			R[7] = -sin(phi) * cos(yawr) + cos(phi) * sin(theta) * sin(yawr);
			R[2] = -sin(theta);
			R[5] = sin(phi) * cos(theta);
			R[8] = cos(phi) * cos(theta);

			dxrDeb = dxr;
			dyrDeb = dyr;
			dzrDeb = dzr;

			double dxr1 = R[0] * dxr + R[1] * dyr + R[2] * dzr;
			double dyr1 = R[3] * dxr + R[4] * dyr + R[5] * dzr;
			double dzr1 = R[6] * dxr + R[7] * dyr + R[8] * dzr;

			dxr = dxr1;
			dyr = dyr1;
			dzr = dzr1;
			
		}
		if (rxMsg->ReadType() == 5){
			int rec = *rxMsg->ReadData();
			if (rec == LANDING)
			{
				landing = 1;
			}
		}
	}
	RemoveCurrentMsg();
	/*
    lat_cart=(((double)latitude*60.0*1852.0/10000000.0)- lat_off); // actual positions are related to the starting position
    long_cart=((((double)longitude*60.0*1852.0/10000000.0)*(cos(((double)latitude*M_PI)/(10000000.0*180.0))))- long_off);
    height_cart =- (((double)fus_height/1000.0) - height_off);
        
    ETMessage *txMessage = new ETMessage(3*sizeof(double), ETHNOS_POSITION);
    // COORDINATES IN NED FRAME
        
    *((double*)txMessage->GetData())=lat_cart;
    *((double*)((char*)(txMessage->GetData())+sizeof(double)))=long_cart;
    *((double*)((char*)(txMessage->GetData())+2*sizeof(double)))=height_cart;
        
    ShareMsg(txMessage);
	*/

	// time calculation
	gettimeofday(&endtime2, NULL);
	secc = endtime2.tv_sec - starttime2.tv_sec;
	msec = endtime2.tv_usec - starttime2.tv_usec;
	mtime = ((secc)+(msec / 1000000.0)); //time in sec
	accum_time = accum_time + mtime;
	gettimeofday(&starttime2, NULL);
	/*
	static double *R = new double[9];
	
	R[0] = cos(theta) * cos(yawr);
	R[3] = -cos(phi) * sin(yawr) + sin(phi) * sin(theta) * cos(yawr);
	R[6] = sin(phi) * sin(yawr) + cos(phi) * sin(theta) * cos(yawr);
	R[1] = cos(theta) * sin(yawr);
	R[4] = cos(phi) * cos(yawr) + sin(phi) * sin(theta) * sin(yawr);
	R[7] = -sin(phi) * cos(yawr) + cos(phi) * sin(theta) * sin(yawr);
	R[2] = -sin(theta);
	R[5] = sin(phi) * cos(theta);
	R[8] = cos(phi) * cos(theta);
	
	/*
	R[0] = cos(theta) * cos(yawr);
	R[1] = -cos(phi) * sin(yawr) + sin(phi) * sin(theta) * cos(yawr);
	R[2] = sin(phi) * sin(yawr) + cos(phi) * sin(theta) * cos(yawr);
	R[3] = cos(theta) * sin(yawr);
	R[4] = cos(phi) * cos(yawr) + sin(phi) * sin(theta) * sin(yawr);
	R[5] = -sin(phi) * cos(yawr) + cos(phi) * sin(theta) * sin(yawr);
	R[6] = -sin(theta);
	R[7] = sin(phi) * cos(theta);
	R[8] = cos(phi) * cos(theta);
	//*/
	
	/*
	cv::Mat RotMat(3, 3, CV_32F, R);

	cv::Mat dxyz(3, 1, CV_32F, { dxr, dyr, dzr });
	dxyz = RotMat * dxyz;
	//*/
	/*
	//DEBUG: velocit� in NED frame
	dxrDeb = dxr;
	dyrDeb = dyr;
	dzrDeb = dzr;


	double dxr1 = R[0] * dxr + R[1] * dyr + R[2] * dzr;
	double dyr1 = R[3] * dxr + R[4] * dyr + R[5] * dzr;
	double dzr1 = R[6] * dxr + R[7] * dyr + R[8] * dzr;

	dxr = dxr1;
	dyr = dyr1;
	dzr = dzr1;
	*/
	//******PROVA!
	initialize = 1;
   
    if ((initialize==0)|(ended==1))
    {
        cumulx=0;
        cumuly=0;
        cumulz=0;
        cumulyaw=0;
    }        
    if ((initialize==1)&(ended==0)){
		
		/***CONTROLLO INIZIA QUI****/
		e1 = function1(xr, yr, zr);
		e2 = function2(xr, yr, zr);

		fgrad1(xr, yr, zr, grad1);
		fgrad2(xr, yr, zr, grad2);

		//vettori OpenCv

		/*
		cv::Mat Grad1(3, 1, CV_32F, grad1);
		cv::Mat Grad2(3, 1, CV_32F, grad2);
		//*/

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
		//ATTENZIONE!! normalizzazione con modulo 0;
		//Prodotto vettore
		
		double Tang[]
		{
			grad1[0] * grad2[1] - grad2[0] * grad1[1],
			grad1[1] * grad2[2] - grad1[2] * grad2[1],
			grad1[0] * grad2[2] - grad1[2] * grad2[0]
		};

		/*
		cv::Mat Tang = Grad1.cross(Grad2);
		normalize1(Tang);
		//*/




		//normalizzazione gradienti
		normalize1(grad1);
		normalize1(grad2);
		//Grad1 = Grad1 / Grad1.norm();
		//Grad2 = Grad2 / Grad2.norm();
		//vettore risultante
		double SumNED[] = {
			e1*grad1[0] + e2*grad2[0] + ktg*Tang[0],
			e1*grad1[1] + e2*grad2[1] + ktg*Tang[1],
			e1*grad1[2] + e2*grad2[2] + ktg*Tang[2],
		};
//		cv::Mat SumNED = Grad1*e1 + Grad2 *e2 + ktg*Tang;
		normalize1(SumNED);

		SumNED[0] *= veld;
		SumNED[1] *= veld;
		SumNED[2] *= veld;
		//SumNED = veld* Sum / Sum.norm();
		//Controllo che non vada sotto terra!
		/*
		if (zr > ground){
			SumNED = SumNED * 0;
		}
		*/

		//vettore velocit� nello spazio del drone
		/*cv::Mat Sum = RotMat * SumNED;
		
		dxd = Sum.at<double>(0, 0);
		dyd = Sum.at<double>(1, 0);
		dzd = Sum.at<double>(2, 0);
		//*/

		dxd = R[0] * SumNED[0] + R[1] * SumNED[1] + R[3] * SumNED[2];
		dyd = R[3] * SumNED[0] + R[4] * SumNED[1] + R[5] * SumNED[2];
		dzd = R[6] * SumNED[0] + R[7] * SumNED[1] + R[8] * SumNED[2];


		if (landing == 1){
			printf("ATTERRAGGIO");
			dxd = 0;
			dyd = 0;
			dzd = 0;
		}
		double edx = dxd - dxr;
		double edy = dyd - dyr;
		double edz = dzd - dzr;

		//printf("\nedx, edy: %f %f", edx, edy);
		cumulx = cumulx + edx*mtime;
		cumuly = cumuly + edy*mtime;
		cumulz = cumulz + edz*mtime;
		
		//Derivativo:
		dedx = (edx - edxback) / mtime;
		dedy = (edy - edyback) / mtime;
		dedz = (edz - edzback) / mtime;

		edxback = edx;
		edyback = edy;
		edzback = edz;

		//ATTERRAGGIO?

        up = pitchOffset + kpvx*(edx) + kivx*(cumulx) + kdvx*(dedx); // pitch command
		ur = rollOffset + kpvy*(edy) + kivy*(cumuly)+ kdvy*(dedy); // roll command

		if (landing == 1)
		{
			ut = ut - 0.5;
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
    fs2 << xr << "\t" << yr << "\t" << zr << "\t";
    fs2 << dxr << "\t" << dyr << "\t" << dzr<<"\t";
	fs2 << dxrDeb << "\t" << dyrDeb << "\t" << dzrDeb << "\t";
    fs2 << theta << "\t" << phi << "\t" << yawr << "\t";
    fs2 <<  "\n";
	}
	else {
	cerr << "File not open!" << endl;
	}

    // terminal visualization (debug)
        
   if(printTimeCounter == printstep ){
        //printf ("GPS DATA: long %d lat %d height %d", longitude, latitude, height);
        //printf ("gps_cartesian: long %f lat %f height %f \n", long_cart, lat_cart, height_cart);
        //printf ("  fus long %d lat %d height %d \n", fus_longitude, fus_latitude, fus_height);
        //printf ("GPS STATUS:  gps: heading: %d, yaw: %f, accuracy: %d, height accuracy: %d, sat: %d, status: %d  \n", GPS_heading, psi, position_accuracy, height_accuracy, GPS_num, GPS_status);       
		printf("\n\nROBOT POSITION: x, y, z, yaw: %f %f %f %f\n", xr, yr, zr, yawr*180/M_PI);
        printf("COMMANDS: command pitch, roll, thrust, yaw: %d %d %d %d \n", CTRL_pitch, CTRL_roll, CTRL_thrust, CTRL_yaw);
        printf("VELOCITY   : dx, dy, dz: %f %f %f \n", dxr, dyr, dzr);
		printf("VELOCITYNED: dx, dy, dz: %f %f %f \n", dxrDeb, dyrDeb, dzrDeb);
		printf("DESIDERED VELOCITY: dx, dy, dz: %f %f %f \n", dxd, dyd, dzd);
		printf("ERRORS: e1, e2 %f %f\n", e1, e2);
		printf("YAW CTRL: yawd, yawr %f %f\n", yawd, yawr);

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

