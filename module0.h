#include <stdio.h>
#include "const.h"


class Module0 : public ETExpert
{
private:
	int message_rec;
	//posizione e velocit� attuale
	double xr, yr, zr, yawr;
	double dxr, dyr, dzr, dyawr;
	//velocit� desiderata e yaw desiderata
	double dxd, dyd, dzd, yawd;
	//parametri regolatori : k_costante,(p,i,d)_Proporzionale integrativa o derivativa, vx_ velocit� lungo x
	double kpvx, kpvy, kpvz, kpyaw;
	double kdvx, kdvy, kdvz, kdyaw;
	double kivx, kivy, kivz, kiyaw;

	double thrp, thrr;		//soglie angoli
	//altri parametri
	double ke1, ke2;		//Guadagno sull'errore
	double thre1, thre2;	//soglie errori
	double ktg;			//costante componente tangenziale
	double veld;			//velocit� di riferimento (x,y,z)
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


public:


	Module0(ETDispatch*);
	~Module0();
	void Init();
	void Close();
	void DoYourDuty(int wc = 0);
	void loadParams();
	void printParams();
	void initializeDataSaving();
};