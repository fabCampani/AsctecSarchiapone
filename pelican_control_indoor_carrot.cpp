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
#include "functions.h"

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

//Velocità frame NED
double dxC;
double dyC;
double dzC;

double dxrDeb, dyrDeb, dzrDeb;
double thr_vel;

//Errori
double ex;
double ey;
double ez;
//passo soglia
double step;
bool prima;

double kpx, kpy, kpz, kix, kiy, kiz, kdx, kdy, kdz;
double xg, yg, zg;
double xp, yp, zp;
double dex, dey, dez;

double dist;




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


double Module0::distance(){
	return(sqrt(pow(xr - xp, 2) + pow(yr - yp, 2) + pow(zr - zp, 2)));
}

// debug function -- the parameters can be printed to check if they were correctly loaded.
void
Module0::printParams() {

}

//Caricamento parametri (da file params2.txt se esiste)
void
Module0::loadParams() {
	prima = true;

	kpx = -150;
	kpy = 150;
	kpz = -550.0;

	kpyaw = 1500.0;

	kdx = -600.0;
	kdy = 600.0;
	kdz = 0.0;
	kdyaw = 0.0;
	
	kix = 0;
	kiy = 0;
	kiz = -10;
	kiyaw = 0;	
	
	ke1 = -33;
	ke2 = -33;

	thre1 = 100;
	thre2 = 100;
	//Costante per componente tangente:
	ktg = 0.5;

	//velocità di marcia
	dist = 0.2;   // m/s
	step = 0.19;

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
	/*
	exback = 0;
	eyback = 0;
	ezback = 0;

	*/
	//Punto target per telecamera drone.
	x_target = 0;
	y_target = 0;

	//funzioni
	Nfunc1 = 0;
	Nfunc2 = 3;

	ex = 0;
	ey = 0;
	ez = 0;

	thr_vel = 0.075;

	//Caricamento parametri da file
	ifstream fs("params3.txt");	
	if (fs.is_open()){
		//file format:
		//1st line: kpx kpy kpz
		//2nd line: kix kiy kiz
		//3rd line: kdx kdy kdz
		//4th line: ktg speed step
		//5th line: gravitycompensation pitchOffset
		//6th line: ke1 ke2 (always negative)
		//7th line: thr_vel
		fs >> kpx;
		fs >> kpy;
		fs >> kpz;

		fs >> kix;
		fs >> kiy;
		fs >> kiz;

		fs >> kdx;
		fs >> kdy;
		fs >> kdz;

		fs >> ktg;
		fs >> dist;
		fs >> step;

		fs >> gravity;
		fs >> pitchOffset;

		fs >> ke1;
		fs >> ke2;
		fs >> thr_vel;
	}
	//Caricamento Funzioni da file
	ifstream fs3("functions.txt");
	if (fs3.is_open()){
		//file format:
		//funzione 1 funzione2
		//spiegazione funzioni disponibili
		fs3 >> Nfunc1;
		fs3 >> Nfunc2;
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
		fs2 << "ex" << "\t" << "ey" << "\t" << "ez" << "\t";
		fs2 << "dex" << "\t" << "dey" << "\t" << "dez" << "\t";
		fs2 << "cumulx" << "\t" << "cumuly" << "\t" << "cumulz" << "\t";
        fs2 << "x\t";
		fs2 << "y\t";
		fs2 << "z\t";
		fs2 << "xp\t";
		fs2 << "yp\t";
		fs2 << "zp\t";
		fs2 << "xgoal\t";
		fs2 << "ygoal\t";
		fs2 << "zgoal\t";
		//fs2 << "xk\t";
		//fs2 << "yk\t";
		//fs2 << "zk\t";
        fs2 << "dxr\t";
		fs2 << "dyr\t";
		fs2 << "dzr\t";
		//fs2 << "dxk\t";
		//fs2 << "dyk\t";
		//fs2 << "dzk\t";
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
		fs2 << "kpx\t";
		fs2 << "kpy\t";
		fs2 << "kpz\t";

		fs2 << "kix\t";
		fs2 << "kiy\t";
		fs2 << "kiz\t";

		fs2 << "kdx\t";
		fs2 << "kdy\t";
		fs2 << "kdz\t";

		fs2 << "ktg\t";
		fs2 << "dist\t";
		fs2 << "step\t";
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

	dxC = 0;
	dyC = 0;
	dzC = 0;

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


			//Soglia velocità (filtro rumore)
			//thr_vel = 0.08; //0.1 0.08 0.04
			int i = 0;

			if ((dxrDeb - dxr) > thr_vel){
				dxC = dxrDeb - thr_vel;
			}
			else if ((dxrDeb - dxr) < -thr_vel){
				dxC = dxrDeb + thr_vel;
			}
			else
				dxC = dxr;

			if ((dyrDeb - dyr) > thr_vel){
				dyC = dyrDeb - thr_vel;
			}
			else if ((dyrDeb - dyr) < -thr_vel){
				dyC = dyrDeb + thr_vel;
			}
			else
				dyC = dyr;
			if ((dzrDeb - dzr) > thr_vel){
				dzC = dzrDeb - thr_vel;
			}
			else if ((dzrDeb - dzr) < -thr_vel){
				dzC = dzrDeb + thr_vel;
			}
			else
				dzC = dzr;

			//Valore precedente
			dxrDeb = dxC;
			dyrDeb = dyC;
			dzrDeb = dzC;
			
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

	//No Kalman NED -> drone
	double dxr1 = R[0] * dxC + R[1] * dyC + R[2] * dzC;
	double dyr1 = R[3] * dxC + R[4] * dyC + R[5] * dzC;
	double dzr1 = R[6] * dxC + R[7] * dyC + R[8] * dzC;

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
		if ((prima)||(distance() >= step)){
			xp = xr;
			yp = yr;
			zp = zr;
			prima = false;
			//NO KALMAN
			e1 = array_function[Nfunc1](xr, yr, zr);
			e2 = array_function[Nfunc2](xr, yr, zr);

			array_fgrad[Nfunc1](grad1, xr, yr, zr);
			array_fgrad[Nfunc2](grad2, xr, yr, zr);

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
				-grad1[2] * grad2[1] + grad1[1] * grad2[2],
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

			SumNED[0] *= dist;
			SumNED[1] *= dist;
			SumNED[2] *= dist;

			/*
			//NED -> drone
			dxd = R[0] * SumNED[0] + R[1] * SumNED[1] + R[2] * SumNED[2];
			dyd = R[3] * SumNED[0] + R[4] * SumNED[1] + R[5] * SumNED[2];
			dzd = R[6] * SumNED[0] + R[7] * SumNED[1] + R[8] * SumNED[2];
			*/
			dxd = SumNED[0];
			dyd = SumNED[1];
			dxd = SumNED[2];


			if (landing == 1){
				dxd = 0;
				dyd = 0;
				dzd = 0;
			}

			xg = dxd + xr;
			yg = dyd + yr;
			zg = dzd + zr;
		}

		//CALCOLO errore (Frame NED)
		ex = xg - xr;
		ey = yg - yr;
		ez = zg - zr;

		//NED -> drone
		double ex1 = R[0] * ex + R[1] * ey + R[2] * ez;
		double ey1 = R[3] * ex + R[4] * ey + R[5] * ez;
		double ez1 = R[6] * ex + R[7] * ey + R[8] * ez;

		ex = ex1;
		ey = ey1;
		ez = ez1;

		cumulx = cumulx + ex * mtime;
		cumuly = cumuly + ey * mtime;
		cumulz = cumulz + ez * mtime;

		//Derivativo:
		dex = -dxr;
		dey = -dyr;
		dez = -dzr;

        up = pitchOffset + kpx*(ex) + kix*(cumulx) + kdx*(dex); // pitch command
		
		ur = rollOffset + kpy*(ey) + kiy*(cumuly)+ kdy*(dey); // roll command

		
		//Procedura di ATTERRAGGIO (MAI testata)

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
		ut = gravity + kpz*(ez) + kiz*(cumulz) + kdz*(dez); // thrust command calculation

		if (ut < 1700){
			ut = 1700;
		}
		
		//Controllo Yaw (TESTATO)
		yawd = atan2(y_target - yr, x_target - xr);
		eyaw = sin(yawd - yawr);
		uy = kpyaw*(eyaw)+kdyaw*(-dyawr);

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
	fs2 << ex << "\t" << ey << "\t" << ez << "\t";
	fs2 << dex << "\t" << dey << "\t" << dez << "\t";
	fs2 << cumulx << "\t" << cumuly << "\t" << cumulz << "\t";
    fs2 << xr << "\t" << yr << "\t" << zr << "\t";
	fs2 << xp << "\t" << yp << "\t" << zp << "\t";
	fs2 << xg << "\t" << yg << "\t" << zg << "\t";
	//fs2 << xk << "\t" << yk << "\t" << zk << "\t";
    fs2 << dxr << "\t" << dyr << "\t" << dzr<<"\t";
	//fs2 << dxk << "\t" << dyk << "\t" << dzk << "\t";
	fs2 << dxd << "\t" << dyd << "\t" << dzd << "\t";
	//fs2 << dxC << "\t" << dyC << "\t" << dzC << "\t";
    fs2 << theta << "\t" << phi << "\t" << yawr << "\t";
	fs2 << kpx << "\t";
	fs2 << kpy << "\t";
	fs2 << kpz << "\t";

	fs2 << kix << "\t";
	fs2 << kiy << "\t";
	fs2 << kiz << "\t";

	fs2 << kdx << "\t";
	fs2 << kdy << "\t";
	fs2 << kdz << "\t";

	fs2 << ktg << "\t";
	fs2 << dist << "\t";
	fs2 << step << "\t";
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
		printf("\n\nROBOT POSITION: x, y, z, yaw: %f %f %f %f\n", xr, yr, zr, yawr*180/M_PI);
        printf("COMMANDS: command pitch, roll, thrust, yaw: %d %d %d %d \n", CTRL_pitch, CTRL_roll, CTRL_thrust, CTRL_yaw);
		//printf("COMMANDS: ex, ey, ez: %f %f %f \n", ex, ey, ez);
        //printf("VELOCITY   : dx, dy, dz: %f %f %f \n", dxr, dyr, dzr);
		//printf("VELOCITYNED: dx, dy, dz: %f %f %f \n", dxC, dyC, dzC);
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