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


//c

#include <fcntl.h>  // File control definitions

#include <time.h>
#include <stdio.h>

//c++
#include <cmath>
#include <fstream>
#include <sstream>
#include <iostream>

#include "module0.h"
#include "functions.h"

using namespace std;


///GLOBAL VARIABLES (defined in module2.cpp). VARIABLES AND COMMAND FROM AND TO THE ROBOT (Serial Interface)
int CTRL_pitch;
int CTRL_roll;
int CTRL_yaw;
int CTRL_thrust;
int angle_pitch;
int angle_roll;
int angle_yaw;
int initialize;
int ended;
int height;
int landing;
//////////////////////
int message_rec;
//posizione e velocità attuale
double xr, yr, zr, yawr;
double dxr, dyr, dzr, dyawr;
//velocità desiderata e yaw desiderata
double dxd, dyd, dzd, yawd;
//parametri regolatori : k_costante,(p,i,d)_Proporzionale integrativa o derivativa, vx_ velocità lungo x
double kpvx, kpvy, kpvz, kpyaw;
double kdvx, kdvy, kdvz, kdyaw;
double kivx, kivy, kivz, kiyaw;

double thrp, thrr;		//soglie angoli
//altri parametri
double ke1, ke2;		//Guadagno sull'errore
double thre1, thre2;	//soglie errori
double ktg;			//costante componente tangenziale
double veld;			//velocità di riferimento (x,y,z)
//errori
double e1, e2, eyaw;
//gradienti
double grad1[3], grad2[3];
//angoli
double theta, phi, psi;
//parte integrale
double cumulx, cumuly, cumulz, cumulyaw;
//parte derivativa
double edxback, edyback, edzback;
double dedx, dedy, dedz;

//Offset
double pitchOffset, rollOffset, yawOffset;

//gravity compensation
double gravity;

//Punto in cui guarda
double x_target, y_target;

//livello del terreno
double ground;

double mtime, accum_time;
double msec, secc;
double dzback;


//Controlli angoli
double up, ur, ut, uy;

int printstep;
int printTimeCounter;
int gps_flag;
double long_off, lat_off, height_off;
double xback, yback, zback, yawback;
int limit;
int count_ethnos;
double lat_cart, long_cart, height_cart;


#define LANDING 95

int Nfunc1;
int Nfunc2;

double dxrDeb;
double dyrDeb;
double dzrDeb;

double edx;
double edy;
double edz;


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
		vett[1] = vett[1] / norm;
		vett[2] = vett[2] / norm;
	}

}

Module0::Module0(){};
// debug function -- the parameters can be printed to check if they were correctly loaded.
void
Module0::printParams() {

}

// function used to load control parameters (from "firefly_params_out.txt" and "pelican_params_out.txt" from the Build folder)
// When new params are loaded, the integrative action is set to zero.
void loadParams() {
	kpvx = -550;
	kpvy = 550;
	kpvz = -200.0;

	kpyaw = 1500.0;

	kdvx = 0.0;
	kdvy = 0.0;
	kdvz = 0.0;
	kdyaw = 0;

	kivx = 0;
	kivy = 0;
	kivz = -0.5;
	kiyaw = 0;

	ke1 = -1;
	ke2 = -1;
	thre1 = 100;
	thre2 = 100;
	ktg = 0.1;

	veld = 0.2;   //* m/s

	ground = 1.2; //m

	gravity = 2450;	//da tarare

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

	Nfunc1 = CIRCLE;
	Nfunc2 = PLANE;

	edx = 0;
	edy = 0;
	edz = 0;
}

//A text file to save the parameters is initialized with the variables to be saved
void
initializeDataSaving() {

	ofstream fs2("dataasctec.txt", std::ofstream::out | std::ofstream::trunc);

	if (fs2.is_open()){
		fs2 << "time \t";
		fs2 << "e1\t";
		fs2 << "e2\t";
		fs2 << "ctrlpitch\t";
		fs2 << "ctrlroll\t";
		fs2 << "ctrlthrust\t";
		fs2 << "ctrlyaw\t";
		fs2 << "edx" << "\t" << "edy" << "\t" << "edz" << "\t";
		fs2 << "x\t";
		fs2 << "y\t";
		fs2 << "z\t";
		fs2 << "dx\t";
		fs2 << "dy\t";
		fs2 << "dz\t";
		fs2 << "dxd\t";
		fs2 << "dyd\t";
		fs2 << "dzd\t";
		fs2 << "dxNED\t";
		fs2 << "dyNED\t";
		fs2 << "dzNED\t";
		fs2 << "pitch\t";
		fs2 << "roll\t";
		fs2 << "yaw\n";

		cout << "File with controls initialized" << endl;
	}
	else {
		cerr << "File not open!" << endl;
	}

}


Module0::~Module0()
{
}


void Init()
{
	message_rec = 0;
	loadParams();
	initializeDataSaving();
	mtime = 0;
	accum_time = 0;

	dxrDeb = 0;
	dyrDeb = 0;
	dzrDeb = 0;

	printstep = 70; // limits the visualization of the status on the terminal
	printTimeCounter = 0;
	gps_flag = 0;
	limit = 25;  // limits the streaming of ETHNOS messages 
	count_ethnos = 0;
}



void main(){

	static double *R = new double[9];
	Init();
	loadParams();
	printTimeCounter++;

	theta = 0;  // Radiants
	phi = 0;    // Radiants
	psi = 0;  // Radiants -> ?
	yawr = 0;

	while (true)  // EHTNOS MESSAGE RECEIVED
	{
		cout << "posizione x: ";
		cin >> xr;
		cout << "posizione y: ";
		cin >> yr;
		cout << "posizione z: ";
		cin >> zr;

		dxr = 0;
		dzr = 0;
		dyr = 0;


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

		initialize = 1;
		ended = 0;
		landing = 0;

		if ((initialize == 0) | (ended == 1))
		{
			cumulx = 0;
			cumuly = 0;
			cumulz = 0;
			cumulyaw = 0;
		}
		if ((initialize == 1)&(ended == 0)){

			/***CONTROLLO INIZIA QUI****/

			e1 = array_function[Nfunc1](xr, yr, zr);
			cout << "\ne1: " << e1;
			e2 = array_function[Nfunc2](xr, yr, zr);
			cout << "\ne2: " << e2;
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

			dxd = R[0] * SumNED[0] + R[1] * SumNED[1] + R[2] * SumNED[2];
			dyd = R[3] * SumNED[0] + R[4] * SumNED[1] + R[5] * SumNED[2];
			dzd = R[6] * SumNED[0] + R[7] * SumNED[1] + R[8] * SumNED[2];


			if (landing == 1){
				dxd = 0;
				dyd = 0;
				dzd = 0;
			}
			
			edx = dxd - dxr;
			edy = dyd - dyr;
			edz = dzd - dzr;

			/*
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
			*/
			up = pitchOffset + kpvx*(edx)+kivx*(cumulx)+kdvx*(dedx); // pitch command
			ur = rollOffset + kpvy*(edy)+kivy*(cumuly)+kdvy*(dedy); // roll command
			//ATTERRAGGIO?
			if (landing == 1)
			{
				if (zr > ground - 0.1){
					landing = 0;
					initialize = 0;
					ut = 1400;
					printf("atterrato\n");
				}
				else{
					if ((printTimeCounter == printstep))
						printf("Procedura di atterraggio: Thrust: %f\n", ut);
					ut = ut - 0.2;
				}

			}
			else
				ut = gravity + kpvz*(edz)+kivz*(cumulz)+kdvz*(dedz); // thrust command calculation

			if (ut < 1700){
				ut = 1700;
			}

			//Controllo Yaw
			yawd = atan2(y_target, x_target);
			eyaw = sin(yawd - yawr);
			uy = kpyaw*(eyaw);

			cout << "\nCTRL_pitch " << up << " CTRL_roll " << ur << " CTRL_thrust " << ut<< endl;

		}

	}
}
