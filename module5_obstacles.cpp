///////////////////////////////
// OBSTACLES RECOGNITION     //
//	    15 hz	??			//
///////////////////////////////
//#define SAMPLESLEN 3
#define SAMPLESLEN 3
#define movingwindow 5
#define mn 35
#define OBSTACLE_ID 1021
//aruco
#include "aruco.h"
#include "cvdrawingutils.h"
#include <semaphore.h>


#include <cmath>
#include <fstream>
#include <sstream>
#include <iostream>
#include <time.h>
#include <stdio.h>
#include <sys/time.h>

//ours
#include "enums.h"

//header
#include "module5.h"

using namespace cv;
using namespace aruco;
using namespace std;

extern int32_t angle_pitch;
extern int32_t angle_roll;
extern int32_t angle_yaw;
extern int16_t acc_x;
extern int16_t acc_y;
extern int16_t acc_z;

//aruco
int ThePyrDownLevel2;
float TheMarkers2ize2=0.200;
string TheInputVideo2;
MarkerDetector MDetector2;
VideoCapture TheVideoCapturer2;
vector<Marker> TheMarkers2;
Mat TheInputImage2,TheInputImage2Copy;
CameraParameters TheCameraParameters2;
double ThresParam12,ThresParam22;
int iThresParam12,iThresParam22;
int waitTime2=0;
cv::Mat Rcv2;
struct timeval starttime3, endtime3;

extern vector<Marker> TheMarkersOriginal;
extern sem_t mutex;

//statistics
static double* sorted = new double[SAMPLESLEN];
static int* occurrences = new int[SAMPLESLEN];

double thr_dim = 0.0;

// aruco
int myarucoSetup (){
		TheVideoCapturer2.open(0);
		waitTime2=10;
		//check video is open
		if (!TheVideoCapturer2.isOpened()) 
			{
			cerr<<"Could not open video"<<endl;
			return -1;
			}
		//read first image to get the dimensions
		TheVideoCapturer2>>TheInputImage2;
		//read camera parameters if passed
			TheCameraParameters2.readFromXMLFile("utils/aruco.yml");
			TheCameraParameters2.resize(TheInputImage2.size());
		//Configure other parameters
		if (ThePyrDownLevel2>0)
			MDetector2.pyrDown(ThePyrDownLevel2);
		///SET THRESHOLDS
		MDetector2.setThresholdParams(7,1);
		}

void myarucoGrabCycle(){
    TheVideoCapturer2.grab();
	 TheVideoCapturer2.retrieve( TheInputImage2);
	 //copy image
	 //Detection of markers in the image passed
	 MDetector2.detect(TheInputImage2,TheMarkers2,TheCameraParameters2,TheMarkers2ize2);
	 TheInputImage2.copyTo(TheInputImage2Copy);

 }

Module5::Module5 (ETDispatch* dis, long int period, int sm)
:ETExpert(dis,period, PRIORITY_AUTO,0,EXPERT_USER)
{
	SendType = sm;
}

Module5::~Module5()
{
}

void
Module5::Init()
{
	//AddRequest(2);
	//SetActivationCondition(GetAllMsgTypeMask());
	message_sent=0; message_rec=0;    
	//distances COM robot - Camera
	xoc=-0.120, yoc=-0.180, zoc=-0.055;
	//robot wrt marker
	xr=0, yr=0, zr=0, aPsi=0;
	//angle_pitch=0;
	//angle_roll=0;
	thr1 = 0.5; // threshold for opencv axes oscillations (y position)
	//movingwindow = 5; // moving window for the average on the marker positions --- default = 5
    //movingwindow = 5;
	measurements = new double*[StatusSize];
	 for(int i = 0; i < StatusSize; i++){
		 measurements[i] = new double[SAMPLESLEN];
		 memset(measurements[i],0,sizeof(double)*(SAMPLESLEN)); //zero fill
	 } 
	 cout << endl << "Measurements initialized" << endl;
	myarucoSetup();
	//initializePosSaving();
	gettimeofday(&starttime3, NULL);
	mtime=0; 
	accum_time=0;
	printstep = 15;
	printTimeCounter=0;
}

void
Module5::DoYourDuty(int wc)
{

	if (wc)  return;
	sem_wait(&mutex);
	if (TheMarkersOriginal.size() == 0){
		myarucoGrabCycle();
		//cout << "myCycle\n";
	}
	else
	{	
		TheMarkers2 = TheMarkersOriginal;
		//cout << "NotmyCycle\n";
	}
	//sem_post(&mutex);

	printTimeCounter++;
	//check if there is a new frame
	if (TheMarkers2.size() > 0){
		//convert angles from asctec //theta = pitch
		theta = 0.001 * (double)angle_pitch *M_PI / 180.0;
		//phi = roll
		phi = 0.001 * (double)angle_roll *M_PI / 180.0;
		//psi = yaw  ---- just for saving.. we use aPsi calculated with ARUCO
		psi = 0.001 * (double)(angle_yaw)*M_PI / 180.0;
		//robot position backup
		for (int m = (TheMarkers2.size() - 1); m >= 0; m--) {
			if (TheMarkers2[m].id == OBSTACLE_ID){			
				cv::Rodrigues(TheMarkers2[m].Rvec, Rcv2); // R is 3x3
				Rcv2 = Rcv2.t();  // rotation of inverse
				TheMarkers2[m].Tvec = -Rcv2 * TheMarkers2[m].Tvec; // translation of inverse
				
				///marker position with respect to the camera ( - per invertire gli assi?)
				
				xc = -TheMarkers2[m].Tvec.at<float>(1, 0); //aruco y
				yc = -TheMarkers2[m].Tvec.at<float>(0, 0); //aruco x
				zc = -TheMarkers2[m].Tvec.at<float>(2, 0); //aruco z
				
				aPhi = asin(-Rcv2.at<float>(2, 0)); //roll calculated with aruco
				double cosphi = cos(aPhi);
				//aTheta =- ((acos(Rcv2.at<float>(2,2)/cosphi)) - M_PI/2.0); //pitch calculated with aruco
				aPsi = asin(Rcv2.at<float>(1, 0) / cosphi); // yaw calculated with aruco
				//rotation matrix R linerized:
				// 11 12 13 21 22 23 31 32 33
				//using theta and phi from asctec and psi from aruco
				
				static double *R = new double[9];
				//first row
				R[0] = cos(theta) * cos(aPsi);
				R[1] = -cos(phi) * sin(aPsi) + sin(phi) * sin(theta) * cos(aPsi);
				R[2] = sin(phi) * sin(aPsi) + cos(phi) * sin(theta) * cos(aPsi);
				//second row
				R[3] = cos(theta) * sin(aPsi);
				R[4] = cos(phi) * cos(aPsi) + sin(phi) * sin(theta) * sin(aPsi);
				R[5] = -sin(phi) * cos(aPsi) + cos(phi) * sin(theta) * sin(aPsi);
				//third row
				R[6] = -sin(theta);
				R[7] = sin(phi) * cos(theta);
				R[8] = cos(phi) * cos(theta);
				//updated offset wrt marker
				//Pom = R * Poc

				xom = R[0] * xoc + R[1] * yoc + R[2] * zoc;
				yom = R[3] * xoc + R[4] * yoc + R[5] * zoc;
				zom = R[6] * xoc + R[7] * yoc + R[8] * zoc;
				
				
				xr = xoc + xc;
				yr = yoc + yc;
				zr = zoc + zc;
				//cout << xr << ":" << yr << "-> ";
				//cout << "marker: x, y, z:" << xr << " " << yr << " " << zr << endl;
			}

		}
		gettimeofday(&endtime3, NULL);
		secc = endtime3.tv_sec - starttime3.tv_sec;
		msec = endtime3.tv_usec - starttime3.tv_usec;
		mtime = ((secc)+(msec / 1000000.0)); //time in sec
		accum_time = accum_time + mtime;
		//theresold to distance between two consecutive obstacles (if necessary)
		if (sqrt(pow(xr - xr_back, 2) + pow(yr - yr_back, 2) + pow(zr - zr_back, 2)) > thr_dim){
			xr_back = xr;
			yr_back = yr;
			zr_back = zr;
			ETMessage *txMessage = new ETMessage(3 * sizeof(double), /*SendType*/ 5);
			*((double*)txMessage->GetData()) = xr;
			*((double*)((char*)(txMessage->GetData()) + sizeof(double))) = yr;
			*((double*)((char*)(txMessage->GetData()) + 2 * sizeof(double))) = zr;
			ShareMsg(txMessage);			
		}
	}
	message_sent++;
	sem_post(&mutex);
}

void
Module5::Close()
{
	delete[] sorted;
	delete[] occurrences;
 	printf("\nPeriodic obstacles N %d missed %ld  deadlines ",ExpertId, NMissedDeadLine);
 	printf("  and executed %ld times ",message_sent);
}
