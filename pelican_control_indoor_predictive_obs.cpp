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

#define RITARDO 10
#define DEFAULTTIME 10
#define DEFAULTSIZE 0.20

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

#include <stdlib.h>

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

#include <list>
#include "Obstacle.h"

using namespace std;
using namespace cv;

struct timeval starttime2, endtime2;  // variables for the timing epredation

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


double xrNow, dxrNow, dxr5;
double yrNow, dyrNow, dyr5;
double zrNow, dzrNow, dzr5;

int ostacoli = 0;


//velocità filtrate con Thresold
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


//ROBA
double last_time = 0;
double timeVec[RITARDO];
double contrx[RITARDO];
double contry[RITARDO];
double contrz[RITARDO];
bool telecamere;
//Roba Errore Predizione
double x_past_pred[RITARDO];
double y_past_pred[RITARDO];
double z_past_pred[RITARDO];

double dx_past_pred[RITARDO];
double dy_past_pred[RITARDO];
double dz_past_pred[RITARDO];

double x_error_pred;
double y_error_pred;
double z_error_pred;
double dx_error_pred;
double dy_error_pred;
double dz_error_pred;

double cumulx_pred = 0;
double cumuly_pred = 0;
double cumulz_pred = 0;

double cumuldx_pred = 0;
double cumuldy_pred = 0;
double cumuldz_pred = 0;

//ostacoli

list<Obstacle*> Obstacles;

//GRIGLIA OSTACOLI
#define cells 20
//dimensione lato griglia
double size_grid = 10;
//position of cell [0][0]
double start_cell_x = 0;
double start_cell_y = 0;
int occ_grid[cells][cells];

void new_occ_grid(double xCenter, double yCenter){
	for (int i = 0; i < cells; i++)
	{
		for (int j = 0; j < cells; j++)
		{
			occ_grid[i][j] = 0;
		}
	}
	start_cell_x = xCenter - size_grid / 2;
	start_cell_y = yCenter - size_grid / 2;
}
void new_occ(double xo, double yo){
	if ((xo < start_cell_x) || (xo > start_cell_x + size_grid) || (yo < start_cell_y) || (yo> start_cell_y + size_grid)){
		cout << xo << "" << yo << " return\n";
		return;
	}
	cout << xo << ":" << yo << "->";
	int index1 = (int)((xo - start_cell_x) / (size_grid / cells));
	cout << index1 <<" ";
	int index2 = (int)((yo - start_cell_y) / (size_grid / cells));
	cout << index2 << endl;
	occ_grid[index1][index2]++;
	return;
}
void decade_occ(){
	for (int i = 0; i < cells; i++)
	{
		for (int j = 0; j < cells; j++)
		{
			if (occ_grid[i][j] > 0)
				occ_grid[i][j]--;
		}
	}
}
void enlist_occ(double zAtt){
	Obstacles.clear();
		for (int i = 0; i < cells; i++)
		{
			for (int j = 0; j < cells; j++)
			{
				if (occ_grid[i][j] > 0){
					double xo = start_cell_x + (size_grid / cells) *i;
					double yo = start_cell_y + (size_grid / cells) *j;
					double zo = zAtt;
					Obstacle *obs = new Obstacle(xo, yo, zo, occ_grid[i][j], (size_grid / cells));
					Obstacles.push_back(obs);
				}
			}
		}
}
bool firstGriglia = true;

void print_occ_Grid(){
	system("clear");
	cout << start_cell_x << " " << start_cell_y<<endl;
	for (int i = 0; i < cells; i++)
	{
		for (int j = 0; j < cells; j++)
		{
			cout << occ_grid[i][j] << "\t";
		}
		cout << endl;
	}
	cout << endl << endl;
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
		vett[0] = vett[0] / norm;
		vett[1] = vett[1] / norm;
		vett[2] = vett[2] / norm;
	}

}
double norma(double *vett){
	return sqrt(pow(vett[0], 2) + pow(vett[1], 2) + pow(vett[2], 2));
}
double media(double *vett, int lenght){
	double mean = 0;
	for (int i = 0; i < lenght; i++)
	{
		mean += vett[i];
	}
	return(mean / lenght);
}

void PredizioneStato(double* vPos, double* vVel, int delay)
{
	
	double x_att = vPos[0];
	double vx_att = vVel[0];
	double y_att = vPos[1];
	double vy_att = vVel[1];
	double z_att = vPos[2];
	double vz_att = vVel[2];
	double t_medio = media(timeVec, delay);


	double x_next;
	double y_next;
	double z_next;
	double vx_next;
	double vy_next;
	double vz_next;

	
	double ax = contrx[delay - 1];
	double ay = contry[delay - 1];
	double az = contry[delay - 1];
	for (int j = 0; j < delay; j++)
	{

		x_next = x_att + vx_att * t_medio + 1 / 2 * ax * t_medio * t_medio;
		vx_next = vx_att + ax* t_medio;

		x_att = x_next;
		vx_att = vx_next;
		ax = contrx[delay - j - 1];

		y_next = y_att + vy_att * t_medio + 1 / 2 * ay * t_medio * t_medio;
		vy_next = vy_att + ay* t_medio;

		y_att = y_next;
		vy_att = vy_next;
		ay = contry[delay - j - 1];


		z_next = z_att + vz_att * t_medio + 1 / 2 * az * t_medio * t_medio;
		vz_next = vz_att + az* t_medio;

		z_att = z_next;
		vz_att = vz_next;
		az = contrz[delay - j - 1];
	}

	//----Errore di preda
	//Shift vettori
	for (int i = RITARDO - 1; i >0; i--)
	{
		x_past_pred[i] = x_past_pred[i - 1];
		y_past_pred[i] = y_past_pred[i - 1];
		z_past_pred[i] = z_past_pred[i - 1];
		dx_past_pred[i] = dx_past_pred[i - 1];
		dy_past_pred[i] = dy_past_pred[i - 1];
		dz_past_pred[i] = dz_past_pred[i - 1];
	}


	if (x_past_pred[RITARDO - 1] != 0){
		//Calcolo errori di predizione
		x_error_pred = vPos[0] - x_past_pred[RITARDO - 1];
		y_error_pred = vPos[1] - y_past_pred[RITARDO - 1];
		z_error_pred = vPos[2] - z_past_pred[RITARDO - 1];
		dx_error_pred = vVel[0] - dx_past_pred[RITARDO - 1];
		dy_error_pred = vVel[1] - dy_past_pred[RITARDO - 1];
		dz_error_pred = vVel[2] - dz_past_pred[RITARDO - 1];

		cumulx_pred += x_error_pred * t_medio;
		cumuly_pred += y_error_pred * t_medio;
		cumulz_pred += z_error_pred * t_medio;

		cumuldx_pred += dx_error_pred * t_medio;
		cumuldy_pred += dy_error_pred * t_medio;
		cumuldz_pred += dz_error_pred * t_medio;
	}

	double k1 = 0.5;
	double k2 = 0.5;
	double k3 = 0.5;
	double kd1 = 0;
	double kd2 = 0;
	double kd3 = 0;

	vPos[0] = x_att + k1*cumulx_pred;
	vVel[0] = vx_att + kd1*cumuldx_pred;
	vPos[1] = y_att + k2*cumuly_pred;
	vVel[1] = vy_att + kd2*cumuldy_pred;
	vPos[2] = z_att + k3*cumulz_pred;
	vVel[2] = vz_att + kd3*cumuldz_pred;

	x_past_pred[0] = vPos[0];
	y_past_pred[0] = vPos[1];
	z_past_pred[0] = vPos[2];
	dx_past_pred[0] = vVel[0];
	dy_past_pred[0] = vVel[1];
	dz_past_pred[0] = vVel[2];
	
	/*
	vPos[0] = x_att;
	vVel[0] = vx_att;
	vPos[1] = y_att;
	vVel[1] = vy_att;*/
	//vPos[2] = z_att;
	//vVel[2] = vz_att;
	
}
//Roba che inutile ma che c'è
void initializeVett(double *vett, int lenght){
	for (int i = 0; i < lenght; i++)
	{
		vett[i] = 0;
	}
}
#define TIME_GRID 1
double time_grid = TIME_GRID;
void removeOld(double timeInterval){
	time_grid -= timeInterval;
	if (time_grid < 0){
		decade_occ();
		time_grid = TIME_GRID;
	}
	list<Obstacle*>::iterator it;
		for(it = Obstacles.begin(); it != Obstacles.end(); ){
			(*it)->time -= timeInterval;
			if ((*it)->time < 0){
				it = Obstacles.erase(it);
				ostacoli--;
			}
			else
				it++;
	}
}


void printList(){

	list<Obstacle*>::iterator it;
	for (it = Obstacles.begin(); it != Obstacles.end(); ++it){
		cout << "(" << (*it)->x << ":" << (*it)->y <<")\t";
	}
	cout << "\n";
	cout << Obstacles.size() << endl;
	
	
}

double gauss(double x, double y, double center_x, double center_y)
{
	double a = 0.8;
	double var = 1;

	return a*exp((-pow((x - center_x),2) - pow((y - center_y),2)) / pow(var,2));
}

//funzione ricorsiva
double funzione(double x, double y, double z, list<Obstacle*> punti_prec){

	if (punti_prec.empty()){
		return array_function[Nfunc1](x, y, z);
	}
	Obstacle* obs0 = punti_prec.front();
	punti_prec.pop_front();
	double precedente_xyz = funzione(x, y, z, punti_prec);
	double precedente_xyz0 = funzione(obs0->x, obs0->y, obs0->z, punti_prec);
	if (precedente_xyz > precedente_xyz0)
		return precedente_xyz - gauss(x, y, obs0->x, obs0->y);
	else
		return precedente_xyz + gauss(x, y, obs0->x, obs0->y);
}

//funzione iterativa
double obstacleAdder(double x_coordinate, double y_coordinate, double livello)
{

	list<int> sign;
	list<Obstacle*>::iterator it;
	list<Obstacle*>::iterator itback;
	list<int>::iterator itsign;
	list<int>::iterator itsignback;
	double livello_ostacolo;

	double livello_influenzato = livello;

	for (it = Obstacles.begin(); it != Obstacles.end(); ++it)
	{
		//calcolo curva di livello su ostacolo
		livello_ostacolo = array_function[Nfunc1]((*it)->x, (*it)->y, (*it)->z);

		for (itback = Obstacles.begin(), itsignback = sign.begin(); it != itback; ++itback, ++itsignback)
		{
			//aggiungo le influenze delle gaussiane associate degli altri ostacoli
			livello_ostacolo = livello_ostacolo + (*itsignback) * gauss((*it)->x, (*it)->y, (*itback)->x, (*itback)->y);
		}


		//mi salvo il segno della gaussiana
		if (livello_influenzato > livello_ostacolo)
		{
			sign.push_back(-1);
		}
		else
		{
			sign.push_back(1);
		}


		//il nuovo livello è dato da il livello influenzato precedentemente e la nuova gaussiana "SUPERCAZZOLA"
		livello_influenzato = livello_influenzato + sign.back() * gauss(x_coordinate, y_coordinate, (*it)->x, (*it)->y);
	}

	return livello_influenzato;

}


// debug function -- the parameters can be printed to check if they were correctly loaded.
void
Module0::printParams() {

}

//Caricamento parametri (da file params2.txt se esiste)
void
Module0::loadParams() {
	/*
	double xo = -2;
	double yo = -1;
	double zo = 0;
	/*
	Obstacle *obs = new Obstacle(xo, yo, zo, 3600, 0.25);
	//cout << "prova push" << endl;
	Obstacles.push_front(obs);
	*/
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
	pitchOffset = 170;  // pitch offset, manually epredd
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


	//Inizializzazione Vettori per Predittore
	initializeVett(timeVec, RITARDO);
	initializeVett(contrx, RITARDO);
	initializeVett(contry, RITARDO);
	initializeVett(contrz, RITARDO);
	initializeVett(x_past_pred, RITARDO);
	initializeVett(y_past_pred, RITARDO);
	initializeVett(z_past_pred, RITARDO);
	initializeVett(dx_past_pred, RITARDO);
	initializeVett(dy_past_pred, RITARDO);
	initializeVett(dz_past_pred, RITARDO);
	

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
	ofstream fso("ostacoli.txt", std::ofstream::out | std::ofstream::trunc);
	ofstream fs2("dataasctec.txt", std::ofstream::out | std::ofstream::trunc);

	if (fs2.is_open()){
		fs2 << "time \t";
		fs2 << "e1\t";
		fs2 << "e2\t";
		fs2 << "e1Ori\t";
		fs2 << "ctrlpitch\t";
		fs2 << "ctrlroll\t";
		fs2 << "ctrlthrust\t";
		fs2 << "ctrlyaw\t";
		fs2 << "edx" << "\t" << "edy" << "\t" << "edz" << "\t";
		fs2 << "cumulx" << "\t" << "cumuly" << "\t" << "cumulz" << "\t";
		fs2 << "x_pred\t";
		fs2 << "y_pred\t";
		fs2 << "z_pred\t";
		fs2 << "dxr_pred\t";
		fs2 << "dyr_pred\t";
		fs2 << "dzr_pred\t";

		fs2 << "dxrT" << "\t" << "dyrT" << "\t" << "dzrT" << "\t";
		fs2 << "dxrUn" << "\t" << "dyrUn" << "\t" << "dzrUn" << "\t";


		fs2 << "x" << "\t" << "y" << "\t" << "z" << "\t";
		fs2 << "dx" << "\t" << "dy" << "\t" << "dz" << "\t";
		fs2 << "ux" << "\t" << "uy" << "\t" << "uz" << "\t";

		fs2 << "dxd\t";
		fs2 << "dyd\t";
		fs2 << "dzd\t";

		fs2 << "Perpx\t";
		fs2 << "Tangx\t";
		fs2 << "Perpy\t";
		fs2 << "Tangy\t";
		fs2 << "Perpz\t";
		fs2 << "Tanz\t";

		//fs2 << "kpvxMod\tkpvyMod\t";

		fs2 << "ax\tay\taz\t";

		fs2 << "err_sti_x\terr_sti_y\terr_sti_z\t";
		fs2 << "err_sti_dx\terr_sti_dy\terr_sti_dz\t";

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
	AddRequest(5); //obstacles

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

	new_occ_grid(0, 0);
}

void
Module0::DoYourDuty(int wc)
{

	static double *R = new double[9];

	printTimeCounter++;

	theta = 0.001 * (double)angle_pitch *M_PI / 180.0;  // Radiants
	phi = 0.001 * (double)angle_roll *M_PI / 180.0;    // Radiants
	psi = 0.001 * (double)angle_yaw *M_PI / 180.0;  // Radiants -> ?


	//Calcolo delle accelerazioni
	double T = (ut * 9.81 / gravity);
	double ax = T * sin((pitchOffset* 0.8936) / 2047 - theta) / 1.6;
	double ay = T * sin (phi)/1.6;
	double az = 9.81 - T/1.6; //Ma anche no

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
		else if (rxMsg->ReadType() == 5){
			bool flag = true;
			//drone wtr new obstacle
			//Per ora numero di punti fisso
			int DIM_OBS = 10;
			double thr_dim = 0.5;

			double xo = *((double*)rxMsg->ReadData());
			double yo = *((double*)((char*)(rxMsg->ReadData()) + sizeof(double)));
			double zo = *((double*)((char*)(rxMsg->ReadData()) + 2 * sizeof(double)));

			xo = xrNow - xo;
			yo = yrNow- yr;
			zo = zrNow - zo;
			
			new_occ(xo, yo);

			//cout << "prova" << endl;
			//DIstance: sqrt(pow(xr - xr_back, 2) + pow(yr - yr_back, 2) + pow(zr - zr_back, 2)) < thr_dim
			/*
			list<Obstacle*>::iterator it;
			for (it = Obstacles.begin(); it != Obstacles.end(); ++it){
				if (sqrt(pow((*it)->x - xo, 2) + pow((*it)->y - yo, 2)) < thr_dim){
					(*it)->time = 100;
					flag = false;
					break;
				}
			}			
			if (flag){
				Obstacle *obs = new Obstacle(xo, yo, zo, 10, 0.25);
				//cout << "prova push" << endl;
				//Obstacles.push_front(obs);
				
				cout << "Drone              Posizione: " << xrNow << " " << yrNow << " " << zrNow << "\n";
				cout << "Ostacolo rilevato! Posizione: " << xo << " " << yo << " " << zo << "\n\n";
				
				//cout << "prova push ok" << endl;
				if (ostacoli < DIM_OBS)
					ostacoli++;
				else{
					//cout << "prova pop" << endl;
					Obstacles.pop_back();
				}
			}
			*/

		}
		else if (rxMsg->ReadType() == 3)
		{
			
			/*
			ax = 0;
			ay = 0;
			az = 0;
			*/
			xr = *((double*)rxMsg->ReadData());
			yr = *((double*)((char*)(rxMsg->ReadData()) + sizeof(double)));
			zr = *((double*)((char*)(rxMsg->ReadData()) + 2 * sizeof(double)));
			yawr = *((double*)((char*)(rxMsg->ReadData()) + 3 * sizeof(double)));
			dxr = *((double*)((char*)(rxMsg->ReadData()) + 4 * sizeof(double)));
			dyr = *((double*)((char*)(rxMsg->ReadData()) + 5 * sizeof(double)));
			dzr = *((double*)((char*)(rxMsg->ReadData()) + 6 * sizeof(double)));
			dyawr = *((double*)((char*)(rxMsg->ReadData()) + 7 * sizeof(double)));

			if (firstGriglia){
				new_occ_grid(-2, 0);
				firstGriglia = false;
			}

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

			//velocità non filtrata
			dxrUn = dxr;
			dyrUn = dyr;
			dzrUn = dzr;

			//Soglia velocità (filtro rumore)
			//thr_vel = 0.08; //0.1 0.08 0.04

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

			double pos[3] = { xr, yr, zr };
			double vel[3] = { dxrT, dyrT, dzrT };

			PredizioneStato(pos, vel, RITARDO);

			xrNow = xr;
			yrNow = yr;
			zrNow = zr;
			dxrNow = dxr;
			dyrNow = dyr;
			dzrNow = dzr;

			xr = pos[0];
			yr = pos[1];
			zr = pos[2];
			dxr = vel[0];
			dyr = vel[1];
			dzr = vel[2];
			
			//NED -> drone
			double dxr1 = R[0] * dxr + R[1] * dyr + R[2] * dzr;
			double dyr1 = R[3] * dxr + R[4] * dyr + R[5] * dzr;
			double dzr1 = R[6] * dxr + R[7] * dyr + R[8] * dzr;

			dxr = dxr1;
			dyr = dyr1;
			dzr = dzr1;

			telecamere = true;
			for (int i = RITARDO-1; i >0; i--)
			{
				contrx[i] = contrx[i - 1];
				contry[i] = contry[i - 1];
				contrz[i] = contrz[i - 1];
				timeVec[i] = timeVec[i - 1];
				
			}
			timeVec[0] = accum_time - last_time;
			last_time = accum_time;
		}
	}
	RemoveCurrentMsg();

	//Previsione velocità attuale (con angolo attuale)	
	/*
	dxr += ax*mtime;
	dyr += ay*mtime;
	*/


	// time calculation
	gettimeofday(&endtime2, NULL);
	secc = endtime2.tv_sec - starttime2.tv_sec;
	msec = endtime2.tv_usec - starttime2.tv_usec;
	mtime = ((secc)+(msec / 1000000.0)); //time in sec
	accum_time = accum_time + mtime;
	gettimeofday(&starttime2, NULL);

	//Remove old (elimazione di osctacoli vecchi)
	removeOld(mtime);
	//printList();

	if ((initialize == 0) | (ended == 1))
	{
		cumulx = 0;
		cumuly = 0;
		cumulz = 0;
		cumulyaw = 0;
	}
	if ((initialize == 1)&(ended == 0)){

		/***CONTROLLO INIZIA QUI****/
		//Metto gli ostacoli nella lista
		enlist_occ(zr);

		e1 = array_function[Nfunc1](xr, yr, zr);
		e2 = array_function[Nfunc2](xr, yr, zr);

		array_fgrad[Nfunc1](grad1, xr, yr, zr);
		array_fgrad[Nfunc2](grad2, xr, yr, zr);

		dxr5 = e1;
		double funcRico = funzione(xr, yr, zr, Obstacles);
		double funcIter = obstacleAdder(xr, yr, e1);

		e1 = funcRico;

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
		/*
		dxd -= dx_error_pred;
		dyd -= dy_error_pred;
		dzd -= dz_error_pred;
		*/
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

		//Derivativo: (NON VA FATTO così)
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

		/*
		kpvxMod = kpvx * abs(edx);
		kpvyMod = kpvy * abs(edy);
		*/

		up = pitchOffset + kpvx*(edx) + kivx*(cumulx) + kdvx*(dedx); // pitch command

		ur = rollOffset + kpvy*(edy) + kivy*(cumuly) + kdvy*(dedy); // roll command

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
			ut = gravity + kpvz*(edz) + kivz*(cumulz) + kdvz*(dedz); // thrust command calculation

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


		
		//CONVERSIONE
		if (telecamere)
		{
			T = CTRL_thrust* 9.81 / gravity;
			double peso = 1.5;
			contrx[0] = (T * sin(((pitchOffset - CTRL_pitch) * 0.8936) / 2047))/ peso;
			contry[0] = (T * sin((CTRL_roll * 0.8936) / 2047))/ peso;
			contrz[0] = (9.81 - T )/ peso;
			telecamere = false;
		}

	}

	// data saving
	if (initialize == 1){
		ofstream fs2("dataasctec.txt", std::ofstream::out | std::ofstream::app);

		if (fs2.is_open()){
			fs2 << accum_time << "\t";
			fs2 << e1 << "\t" << e2 << "\t";
			fs2 << dxr5 << "\t";
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
			
			fs2 << xrNow << "\t" << yrNow << "\t" << zrNow << "\t";
			fs2 << dxrNow << "\t" << dyrNow << "\t" << dzrNow << "\t";
			/*fs2 << dxr5 << "\t" << dyr5 << "\t" << dzr5 << "\t";
			*/
			fs2 << contrx[0] << "\t" << contry[0] << "\t" << contrz[0] << "\t";



			fs2 << dxd << "\t" << dyd << "\t" << dzd << "\t";
			//fs2 << dxrDeb << "\t" << dyrDeb << "\t" << dzrDeb << "\t";
			fs2 << Perpx << "\t" << Tangx << "\t";
			fs2 << Perpy << "\t" << Tangy << "\t";
			fs2 << Perpz << "\t" << Tangz << "\t";
			//fs2 << kpvxMod << "\t" << kpvyMod << "\t";

			fs2 << ax << "\t" << ay << "\t" << az << "\t";
			fs2 << cumulx_pred << "\t" << cumuly_pred << "\t" << cumulz_pred << "\t";
			fs2 << cumuldx_pred << "\t" << cumuldy_pred << "\t" << cumuldz_pred << "\t";

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
		print_occ_Grid();
		printList();
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

