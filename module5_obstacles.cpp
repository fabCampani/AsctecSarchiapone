///////////////////////////////
// OBSTACLES RECOGNITION     //
//	    30 hz	??			//
///////////////////////////////
//#define SAMPLESLEN 3
#define SAMPLESLEN 3
#define movingwindow 5
#define mn 35
#define OBSTACLE_ID 65
//aruco
#include "aruco.h"
#include "cvdrawingutils.h"


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
int ThePyrDownLevel;
float TheMarkerSize=0.200;
string TheInputVideo;
MarkerDetector MDetector;
VideoCapture TheVideoCapturer;
vector<Marker> TheMarkers;
Mat TheInputImage,TheInputImageCopy;
CameraParameters TheCameraParameters;
double ThresParam1,ThresParam2;
int iThresParam1,iThresParam2;
int waitTime=0;
cv::Mat Rcv;
struct timeval starttime, endtime;

//statistics
static double* sorted = new double[SAMPLESLEN];
static int* occurrences = new int[SAMPLESLEN];

double thr_dim = 0.25

// aruco
int arucoSetup (){
		TheVideoCapturer.open(0);
		waitTime=10;
		//check video is open
		if (!TheVideoCapturer.isOpened()) 
			{
			cerr<<"Could not open video"<<endl;
			return -1;
			}
		//read first image to get the dimensions
		TheVideoCapturer>>TheInputImage;
		//read camera parameters if passed
			TheCameraParameters.readFromXMLFile("utils/aruco.yml");
			TheCameraParameters.resize(TheInputImage.size());
		//Configure other parameters
		if (ThePyrDownLevel>0)
			MDetector.pyrDown(ThePyrDownLevel);
		///SET THRESHOLDS
		MDetector.setThresholdParams(7,1);
		}

void arucoGrabCycle(){
    TheVideoCapturer.grab();
	 TheVideoCapturer.retrieve( TheInputImage);
	 //copy image
	 //Detection of markers in the image passed
	 MDetector.detect(TheInputImage,TheMarkers,TheCameraParameters,TheMarkerSize);
	 TheInputImage.copyTo(TheInputImageCopy);

 }

void initializePosSaving() {
	ofstream fs("positions_xyz.txt", std::ofstream::out | std::ofstream::trunc);

	if(fs.is_open()){
		fs << "Positions (x,y,z, yaw) in meters (degrees), calculated with the Aruco Markers with the Asctec axes convention, velocities, angles and accelerations \n";
		cout << "File with position data initialized" << endl;
	} else {
		cerr << "File not open!" << endl;
	}

}
	
// median filter

inline void copyArray( double *from, double *to, int len){
	assert(NULL!=from);
	assert(NULL!=to);
	for (int i = 0; i < len; ++i) {
		to[i] = from[i];
	}
}

inline void mySort(double *arr, int len){
	assert(NULL!=arr);
	static double tmp;
	for (int i = len - 1; i > 0; --i) {
		for (int j = 0; j < i; ++j) {
			if (arr[j] > arr[j+1]) {
				tmp = arr[j];
				arr[j] = arr[j+1];
				arr[j+1] = tmp;
			}
		}
	}
}

inline double mean(double *measurements, int len) {
	assert(NULL!=measurements);
	double res = measurements[0];
	for (int i = 0; i < len; ++i) {
		res += measurements[i];
	}
	res/=(double)len;
	return res;
}

inline double mean(const double &a, const double &b){
	return ((a+b)/2.0);
}

inline double median(double *measurements, int len, double *sorted) {
	assert(len > 0);
	assert(NULL!= measurements);
	assert(NULL != sorted);

	double res = 0.0;
	switch(len){
	case 1:
		res = measurements[0];
		break;
	case 2:
		res = mean(measurements[0], measurements[1]);
		break;
	default:
		copyArray(measurements, sorted, len);
		mySort(sorted, len);
		if ((len % 2) == 0) {
			res=mean(sorted[len/2], sorted[(len/2)-1]);
		} else {
			res = sorted[len/2];
		}
		break;
	}

	return res;
}

double mode(double *measurements, int len) { 
	assert(NULL!=measurements);
	for (int i = 0; i < len; ++i) {
		occurrences[i] = 0;
		int j = 0;
		//bool bFound = false;
		while ((j < i) && (measurements[i] != measurements[j])) {
			if (measurements[i] != measurements[j]) {
				++j;
			}
		}
		++(occurrences[j]);
	}
	int iMaxRepeat = 0;
	for (int i = 1; i < len; ++i) {
		if (occurrences[i] > occurrences[iMaxRepeat]) {
			iMaxRepeat = i;
		}
	}
	return measurements[iMaxRepeat];
}

void shiftBack(double *measurements, int len){
	for( int i = 0; i < len-1; i++){
		measurements[i] = measurements[i+1];
	}
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
	arucoSetup();
	//initializePosSaving();
	gettimeofday(&starttime, NULL);
	mtime=0; 
	accum_time=0;
	printstep = 15;
	printTimeCounter=0;
}

void
Module5::DoYourDuty (int wc)
{

	if (wc)  return;

	arucoGrabCycle();
   
	printTimeCounter++;
	//check if there is a new frame
	 if(TheMarkers.size() >0){
		//convert angles from asctec //theta = pitch
		theta = 0.001 * (double) angle_pitch *M_PI/180.0;
		//phi = roll
		phi = 0.001 * (double) angle_roll *M_PI/180.0;
		//psi = yaw  ---- just for saving.. we use aPsi calculated with ARUCO
		psi = 0.001 * (double) (angle_yaw) *M_PI/180.0;
		//robot position backup
		xr_back=xr;
		yr_back=yr;
		zr_back=zr;
		for (int m =(TheMarkers.size()-1);m>=0;m--) {
			if (TheMarkers[m].id == OBSTACLE_ID){
				cv::Rodrigues(TheMarkers[m].Rvec, Rcv); // R is 3x3
				Rcv = Rcv.t();  // rotation of inverse
				TheMarkers[m].Tvec = -Rcv * TheMarkers[m].Tvec; // translation of inverse
				///marker position with respect to the camera ( - per invertire gli assi)
				xc = -TheMarkers[m].Tvec.at<float>(1, 0); //aruco y
				yc = -TheMarkers[m].Tvec.at<float>(0, 0); //aruco x
				zc = -TheMarkers[m].Tvec.at<float>(2, 0); //aruco z
				aPhi = asin(-Rcv.at<float>(2, 0)); //roll calculated with aruco
				double cosphi = cos(aPhi);
				//aTheta =- ((acos(Rcv.at<float>(2,2)/cosphi)) - M_PI/2.0); //pitch calculated with aruco
				aPsi = asin(Rcv.at<float>(1, 0) / cosphi); // yaw calculated with aruco
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

				//drone position wrt obstacle position
				xr = xom + xc;
				yr = yom + yc;
				zr = zom + zc;
			}

		}
	gettimeofday(&endtime, NULL);
	secc=endtime.tv_sec - starttime.tv_sec;
	msec=endtime.tv_usec-starttime.tv_usec;
	mtime = ((secc)+(msec/1000000.0)); //time in sec
	accum_time=accum_time+mtime;
	if (sqrt(pow(xr - xr_back, 2) + pow(yr - yr_back, 2) + pow(zr - zr_back, 2)) > thr_dim){
		ETMessage *txMessage = new ETMessage(3 * sizeof(double), /*SendType*/ 5);
		*((double*)txMessage->GetData()) = xr;
		*((double*)((char*)(txMessage->GetData()) + sizeof(double))) = yr;
		*((double*)((char*)(txMessage->GetData()) + 2 * sizeof(double))) = zr;
		ShareMsg(txMessage);
		message_sent++;
	}
}

void
Module5::Close()
{
	delete[] sorted;
	delete[] occurrences;
 	printf("\nPeriodic N %d missed %ld  deadlines ",ExpertId, NMissedDeadLine);
 	printf("  and executed %ld times ",message_sent);
}
