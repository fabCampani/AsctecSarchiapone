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
extern int landing;

extern int16_t acc_x;
extern int16_t acc_y;
extern int16_t acc_z;
//////////////////////


#define LANDING 95


//Tipo di funzione
int Nfunc1;
int Nfunc2;


//Errori
double edx;
double edy;
double edz;

/*
double dxr3, dxr4, dxr5;
double dyr3, dyr4, dyr5;
double dzr3, dzr4, dzr5;
*/

//velocit� filtrate con Thresold
double dxrT, dyrT, dzrT;
double thr_vel;
double dxrDeb;
double dyrDeb;
double dzrDeb;



double dxrUn;
double dyrUn;
double dzrUn;

double Tangx, Tangy, Tangz;
double Perpx, Perpy, Perpz;

double kpvxMod, kpvyMod;

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
		vett[0] = vett[0] / norm;
		vett[1] = vett[1] / norm;
		vett[2] = vett[2] / norm;
	}

}
double norma(double *vett){
	return sqrt(pow(vett[0], 2) + pow(vett[1], 2) + pow(vett[2], 2));
}


//Roba che inutile ma che c'�
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

	kpyaw = 1700.0;

	kdvx = 0.0;
	kdvy = 0.0;
	kdvz = 0.0;
	kdyaw = 250.0;

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

	//velocit� di marcia
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

	//Inizializzazione funzioni
	/*
	funzioni a disposizione
	CIRCLE_XY	0
	CIRCLE_YZ	1 
	CIRCLE_XZ	2
	PLANE_Z		3
	PLANE_Y		4
	PLANE_X		5
	PLANE_GEN	6
	ELLIPSE		7
	SPHERE		8
	*/
	Nfunc1 = CIRCLE_XY;
	Nfunc2 = PLANE_Z;

	edx = 0;
	edy = 0;
	edz = 0;
	

	//Carcamento parametri da file
	ifstream fs("params2.txt");
	if (fs.is_open()){
		//file format:
		//1st line: kpx kpy kpz kpyaw
		//2nd line: kix kiy kiz
		//3rd line: kdx kdy kdz kdyaw
		//4th line: ktg speed
		//5th line: gravitycompensation pitchOffset
		//6th line: ke1 ke2 (always negative)
		//7th line: thr_vel
		fs >> kpvx;
		fs >> kpvy;
		fs >> kpvz;
		fs >> kpyaw;

		fs >> kivx;
		fs >> kivy;
		fs >> kivz;

		fs >> kdvx;
		fs >> kdvy;
		fs >> kdvz;
		fs >> kdyaw;

		fs >> ktg;
		fs >> veld;
		fs >> gravity;
		fs >> pitchOffset;

		fs >> ke1;
		fs >> ke2;

		fs >> thr_vel;
	}

	//Carcamento Funzioni da file
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

	if (fs2.is_open()){
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
		fs2 << "dxr\t";
		fs2 << "dyr\t";
		fs2 << "dzr\t";

		fs2 << "dxrT" << "\t" << "dyrT" << "\t" << "dzrT" << "\t";
		fs2 << "dxrUn" << "\t" << "dyrUn" << "\t" << "dzrUn" << "\t";

		fs2 << "dxd\t";
		fs2 << "dyd\t";
		fs2 << "dzd\t";

		fs2 << "Perpx\t";
		fs2 << "Tangx\t";
		fs2 << "Perpy\t";
		fs2 << "Tangy\t";
		fs2 << "Perpz\t";
		fs2 << "Tanz\t";

		fs2 << "kpvxMod\tkpvyMod\t";

		fs2 << "ax\tay\taz\t";

		//fs2 << "dxNED\t";
		//fs2 << "dyNED\t";
		//fs2 << "dzNED\t";
		fs2 << "pitch\t";
		fs2 << "roll\t";
		fs2 << "yaw\t";

		fs2 << "yawd\t";
		fs2 << "eyaw\t";
		//Parameters
		fs2 << "kpvx\t";
		fs2 << "kpvy\t";
		fs2 << "kpvz\t";
		fs2 << "kpyaw\t";

		fs2 << "kivx\t";
		fs2 << "kivy\t";
		fs2 << "kivz\t";

		fs2 << "kdvx\t";
		fs2 << "kdvy\t";
		fs2 << "kdvz\t";
		fs2 << "kdyaw\t";

		fs2 << "ktg\t";
		fs2 << "veld\t";
		fs2 << "pitchoff\t";
		fs2 << "ke1\tke2\t";
		fs2 << "gravity\t";
		fs2 << "thr_vel\n";



		cout << "File with controls initialized" << endl;
	}
	else {
		cerr << "File not open!" << endl;
	}

}

Module0::Module0(ETDispatch* dis)
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

	mtime = 0;
	accum_time = 0;

	//inizializzazione soglia
	dxrDeb = 0;
	dyrDeb = 0;
	dzrDeb = 0;

	printstep = 70; // limits the visualization of the status on the terminal
	printTimeCounter = 0;
	gps_flag = 0;
	limit = 25;  // limits the streaming of ETHNOS messages 
	count_ethnos = 0;
}

void
Module0::DoYourDuty(int wc)
{

	static double *R = new double[9];

	printTimeCounter++;

	theta = 0.001 * (double)angle_pitch *M_PI / 180.0;  // Radiants
	phi = 0.001 * (double)angle_roll *M_PI / 180.0;    // Radiants
	psi = 0.001 * (double)angle_yaw *M_PI / 180.0;  // Radiants -> ?

	double ax = acc_x*9.81 / 10000;
	double ay = acc_y*9.81 / 10000;
	double az = acc_z*9.81 / 10000;

	//Calcolo della matrice di rotazione ogni volta:
	R[0] = cos(theta) * cos(yawr);
	R[3] = -cos(phi) * sin(yawr) + sin(phi) * sin(theta) * cos(yawr);
	R[6] = sin(phi) * sin(yawr) + cos(phi) * sin(theta) * cos(yawr);
	R[1] = cos(theta) * sin(yawr);
	R[4] = cos(phi) * cos(yawr) + sin(phi) * sin(theta) * sin(yawr);
	R[7] = -sin(phi) * cos(yawr) + cos(phi) * sin(theta) * sin(yawr);
	R[2] = -sin(theta);
	R[5] = sin(phi) * cos(theta);
	R[8] = cos(phi) * cos(theta);

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
			ax = 0;
			ay = 0;
			az = 0;

			xr = *((double*)rxMsg->ReadData());
			yr = *((double*)((char*)(rxMsg->ReadData()) + sizeof(double)));
			zr = *((double*)((char*)(rxMsg->ReadData()) + 2 * sizeof(double)));
			yawr = *((double*)((char*)(rxMsg->ReadData()) + 3 * sizeof(double)));
			dxr = *((double*)((char*)(rxMsg->ReadData()) + 4 * sizeof(double)));
			dyr = *((double*)((char*)(rxMsg->ReadData()) + 5 * sizeof(double)));
			dzr = *((double*)((char*)(rxMsg->ReadData()) + 6 * sizeof(double)));
			dyawr = *((double*)((char*)(rxMsg->ReadData()) + 7 * sizeof(double)));

			//Aggiornamento Rotation Matrix per angolo yaw (telecamere)
			R[0] = cos(theta) * cos(yawr);
			R[3] = -cos(phi) * sin(yawr) + sin(phi) * sin(theta) * cos(yawr);
			R[6] = sin(phi) * sin(yawr) + cos(phi) * sin(theta) * cos(yawr);
			R[1] = cos(theta) * sin(yawr);
			R[4] = cos(phi) * cos(yawr) + sin(phi) * sin(theta) * sin(yawr);
			R[7] = -sin(phi) * cos(yawr) + cos(phi) * sin(theta) * sin(yawr);
			R[2] = -sin(theta);
			R[5] = sin(phi) * cos(theta);
			R[8] = cos(phi) * cos(theta);

			
			//Soglia velocit� (filtro rumore)
			//thr_vel = 0.08; //0.1 0.08 0.04
			int i = 0;

			if ((dxrDeb - dxr) > thr_vel){
				dxrT = dxrDeb - thr_vel;
			}
			else if ((dxrDeb - dxr) < -thr_vel){
				dxrT = dxrDeb + thr_vel;
			}
			else
				dxrT = dxr;

			if ((dyrDeb - dyr) > thr_vel){
				dyrT = dyrDeb - thr_vel;
			}
			else if ((dyrDeb - dyr) < -thr_vel){
				dyrT = dyrDeb + thr_vel;
			}
			else
				dyrT = dyr;
			if ((dzrDeb - dzr) > thr_vel){
				dzrT = dzrDeb - thr_vel;
			}
			else if ((dzrDeb - dzr) < -thr_vel){
				dzrT = dzrDeb + thr_vel;
			}
			else
				dzrT = dzr;

			//Valore precedente
			dxrDeb = dxrT;
			dyrDeb = dyrT;
			dzrDeb = dzrT;

			//velocit� non filtrata
			dxrUn = dxr;
			dyrUn = dyr;
			dzrUn = dzr;

			//Assegnamento velocit� reale (ORA NED)
			dxr = dxrT;
			dyr = dyrT;
			dzr = dzrT;
			
			//NED -> drone
			double dxr1 = R[0] * dxr + R[1] * dyr + R[2] * dzr;
			double dyr1 = R[3] * dxr + R[4] * dyr + R[5] * dzr;
			double dzr1 = R[6] * dxr + R[7] * dyr + R[8] * dzr;

			dxr = dxr1;
			dyr = dyr1;
			dzr = dzr1;
			
		}
	}
	RemoveCurrentMsg();

	//Previsione velocit� (NEW)
	double delay = 1;
	dxr += ax*mtime*delay;
	dyr += ay*mtime*delay;

	// time calculation
	gettimeofday(&endtime2, NULL);
	secc = endtime2.tv_sec - starttime2.tv_sec;
	msec = endtime2.tv_usec - starttime2.tv_usec;
	mtime = ((secc)+(msec / 1000000.0)); //time in sec
	accum_time = accum_time + mtime;
	gettimeofday(&starttime2, NULL);


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

		double normSumNED = norma(SumNED);
		if (normSumNED != 0)
		{
			Tangx = ktg*Tang[0] * veld / normSumNED;
			Tangy = ktg*Tang[1] * veld / normSumNED;
			Tangz = ktg*Tang[2] * veld / normSumNED;

			Perpx = (e1*grad1[0] + e2*grad2[0]) * veld / normSumNED;
			Perpy = (e1*grad1[1] + e2*grad2[1]) * veld / normSumNED;
			Perpz = (e1*grad1[2] + e2*grad2[2]) * veld / normSumNED;
		}
		else
		{
			Tangx = 0;
			Tangy = 0;
			Tangz = 0;

			Perpx = 0;
			Perpy = 0;
			Perpz = 0;
		}

		normalize1(SumNED);

		SumNED[0] *= veld;
		SumNED[1] *= veld;
		SumNED[2] *= veld;

		dxd = SumNED[0];
		dyd = SumNED[1];
		dzd = SumNED[2];

		//NED -> drone
		dxd = R[0] * SumNED[0] + R[1] * SumNED[1] + R[2] * SumNED[2];
		dyd = R[3] * SumNED[0] + R[4] * SumNED[1] + R[5] * SumNED[2];
		dzd = R[6] * SumNED[0] + R[7] * SumNED[1] + R[8] * SumNED[2];
		

		/*ATTENZIONE!*/
		//Setup Controllore PID
		/*
		dxd = 0;
		dyd = 0;
		dzd = 0;
		*/

		if (landing == 1){
			dxd = 0;
			dyd = 0;
			dzd = 0;
		}

		edx = dxd - dxr;
		edy = dyd - dyr;
		edz = dzd - dzr;
		/*
		double dxr1 = R[0] * edx + R[1] * edy + R[2] * edz;
		double dyr1 = R[3] * edx + R[4] * edy + R[5] * edz;
		double dzr1 = R[6] * edx + R[7] * edy + R[8] * edz;

		edx = dxr1;
		edy = dyr1;
		edz = dzr1;
		*/
		
		cumulx = cumulx + edx*mtime;
		cumuly = cumuly + edy*mtime;
		cumulz = cumulz + edz*mtime;

		//Derivativo: (NON VA FATTO cos�)
		/*
		dedx = (edx - edxback) / mtime;
		dedy = (edy - edyback) / mtime;
		dedz = (edz - edzback) / mtime;
		edxback = edx;
		edyback = edy;
		edzback = edz;
		*/

		dedx = -ax;
		dedy = -ay;
		dedz = -az;

		kpvxMod = kpvx * abs(edx)/vel_d;
		kpvyMod = kpvy * abs(edy)/vel_d;

		up = pitchOffset + kpvx*(edx)+ kivx*(cumulx)+ kdvx*(dedx); // pitch command

		ur = rollOffset + kpvy*(edy)+ kivy*(cumuly)+ kdvy*(dedy); // roll command

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

		}
		else
			ut = gravity + kpvz*(edz)+kivz*(cumulz)+kdvz*(dedz); // thrust command calculation

		if (ut < 1700){
			ut = 1700;
		}

		//Controllo Yaw (MAI TESTATO)
		yawd = atan2(y_target - yr, x_target - xr);
		eyaw = sin(yawd - yawr);
		uy = kpyaw*(eyaw) + kdyaw*(-dyawr);

		// thresholds to avoid too fast movements
		int16_t CTRL_back;
		int16_t thr3 = 50;		//prima era 50
		CTRL_back = CTRL_pitch;
		CTRL_pitch = (int16_t)up;

		if ((CTRL_pitch - CTRL_back)> thr3) {
			CTRL_pitch = CTRL_back + thr3;
		}
		else if ((CTRL_pitch - CTRL_back)< -thr3) {
			CTRL_pitch = CTRL_back - thr3;
		}

		CTRL_back = CTRL_roll;
		CTRL_roll = (int16_t)ur;

		if ((CTRL_roll - CTRL_back)> thr3) {
			CTRL_roll = CTRL_back + thr3;
		}
		else if ((CTRL_roll - CTRL_back)< -thr3) {
			CTRL_roll = CTRL_back - thr3;
		}

		CTRL_back = CTRL_thrust;
		CTRL_thrust = (int16_t)ut;

		if ((CTRL_thrust - CTRL_back)> thr3) {
			CTRL_thrust = CTRL_back + thr3;
		}
		else if ((CTRL_thrust - CTRL_back)< -thr3) {
			CTRL_thrust = CTRL_back - thr3;
		}

		CTRL_back = CTRL_yaw;
		CTRL_yaw = (int16_t)uy;

		if ((CTRL_yaw - CTRL_back)> thr3) {
			CTRL_yaw = CTRL_back + thr3;
		}
		else if ((CTRL_yaw - CTRL_back)< -thr3) {
			CTRL_yaw = CTRL_back - thr3;

		}
	}

	// data saving
	if (initialize == 1){
		ofstream fs2("dataasctec.txt", std::ofstream::out | std::ofstream::app);

		if (fs2.is_open()){
			fs2 << accum_time << "\t";
			fs2 << e1 << "\t" << e2 << "\t";
			fs2 << CTRL_pitch << "\t" << CTRL_roll << "\t" << CTRL_thrust << "\t" << CTRL_yaw << "\t";
			//fs2 << longitude << "\t" << latitude << "\t" << height << "\t";
			//fs2 << fus_longitude << "\t" << fus_latitude << "\t" << fus_height << "\t";
			//fs2 << GPS_heading << "\t" << position_accuracy << "\t" << height_accuracy << "\t" << GPS_num << "\t" << GPS_status << "\t";
			fs2 << edx << "\t" << edy << "\t" << edz << "\t";
			fs2 << cumulx << "\t" << cumuly << "\t" << cumulz << "\t";
			fs2 << xr << "\t" << yr << "\t" << zr << "\t";
			fs2 << dxr << "\t" << dyr << "\t" << dzr << "\t";

			fs2 << dxrT << "\t" << dyrT << "\t" << dzrT << "\t";
			fs2 << dxrUn << "\t" << dyrUn << "\t" << dzrUn << "\t";
			/*
			fs2 << dxrf << "\t" << dyrf << "\t" << dzr3 << "\t";
			fs2 << dxr4 << "\t" << dyr4 << "\t" << dzr4 << "\t";
			fs2 << dxr5 << "\t" << dyr5 << "\t" << dzr5 << "\t";
			*/

			fs2 << dxd << "\t" << dyd << "\t" << dzd << "\t";
			//fs2 << dxrDeb << "\t" << dyrDeb << "\t" << dzrDeb << "\t";
			fs2 << Perpx << "\t" << Tangx << "\t";
			fs2 << Perpy << "\t" << Tangy << "\t";
			fs2 << Perpz << "\t" << Tangz << "\t";
			fs2 << kpvxMod << "\t" << kpvyMod << "\t";

			fs2 << ax << "\t" << ay << "\t" << az << "\t";

			fs2 << theta << "\t" << phi << "\t" << yawr << "\t";
			fs2 << yawd << "\t" << eyaw << "\t";
			fs2 << kpvx << "\t";
			fs2 << kpvy << "\t";
			fs2 << kpvz << "\t";
			fs2 << kpyaw << "\t";

			fs2 << kivx << "\t";
			fs2 << kivy << "\t";
			fs2 << kivz << "\t";

			fs2 << kdvx << "\t";
			fs2 << kdvy << "\t";
			fs2 << kdvz << "\t";
			fs2 << kdyaw << "\t";

			fs2 << ktg << "\t";
			fs2 << veld << "\t";
			fs2 << pitchOffset << "\t";
			fs2 << ke1 << "\t" << ke2 << "\t";
			fs2 << gravity << "\t";
			fs2 << thr_vel;
			fs2 << "\n";

		}
		else {
			cerr << "File not open!" << endl;
		}
	}
	// terminal visualization (debug)

	if ((printTimeCounter == printstep)){
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
	printf("\nSporadic N %d  ", ExpertId);
	printf(" executed %d times ", message_rec);
	fflush(stdout);
}

