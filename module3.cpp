///////////////////////////////
// CAMERA + ARUCO PROCESSING //
//	    30 hz	     //
///////////////////////////////
//#define SAMPLESLEN 3
#define SAMPLESLEN 3
#define movingwindow 5
#define mn 35
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
#include "module3.h"

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

vector<Marker> TheMarkersOriginal;

//statistics
static double* sorted = new double[SAMPLESLEN];
static int* occurrences = new int[SAMPLESLEN];

//Semaphores ?
sem_t mutex;

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
	 MDetector.detect(TheInputImage, TheMarkersOriginal, TheCameraParameters, TheMarkerSize);
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


Module3::Module3 (ETDispatch* dis, long int period, int sm)
:ETExpert(dis,period, PRIORITY_AUTO,0,EXPERT_USER)
{
	SendType = sm;
	sem_init(&mutex, 0, 1);
}

Module3::~Module3()
{
}

void
Module3::Init()
{
	//AddRequest(2);
	//SetActivationCondition(GetAllMsgTypeMask());
	message_sent=0; message_rec=0;
	averageCounter1=0; averageCounter2=0;
	maxCounter=20; markerCounter=0;
    x_ext_back= new double [mn*movingwindow];
    y_ext_back= new double [mn*movingwindow];
    z_ext_back= new double [mn*movingwindow];
    yaw_ext_back= new double [mn*movingwindow];
    x_ext= new double [mn];
    y_ext= new double [mn];
    z_ext= new double [mn];
    yaw_ext= new double [mn];
    averagex= new double [movingwindow];
    averagey= new double [movingwindow];
    averagez= new double [movingwindow];
    averageyaw= new double [movingwindow];
   
    memset(x_ext_back,-1000,sizeof(double)*mn*movingwindow);
    memset(y_ext_back,-1000,sizeof(double)*mn*movingwindow);
    memset(z_ext_back,-1000,sizeof(double)*mn*movingwindow);
    memset(yaw_ext_back,-1000,sizeof(double)*mn*movingwindow);
    memset(x_ext,-1000,sizeof(double)*mn);
    memset(y_ext,-1000,sizeof(double)*mn);
    memset(z_ext,-1000,sizeof(double)*mn);
    memset(yaw_ext,-1000,sizeof(double)*mn);
    memset(averagex,0,sizeof(double)*movingwindow);
    memset(averagey,0,sizeof(double)*movingwindow);
    memset(averagez,0,sizeof(double)*movingwindow);
    memset(averageyaw,0,sizeof(double)*movingwindow);
    
	//distances COM robot - Camera
	xoc=-0.120, yoc=-0.180, zoc=-0.055;
	//robot wrt marker
	xr=0, yr=0, zr=0, aPsi=0;
	//angle_pitch=0;
	//angle_roll=0;
	thr1 = 0.5; // threshold for opencv axes oscillations (y position)
	//movingwindow = 5; // moving window for the average on the marker positions --- default = 5
    //movingwindow = 5;
	tempx=0; tempy=0; tempz=0; tempyaw=0;
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
Module3::DoYourDuty (int wc)
{
    //printf("c");
    //fflush(stdout);
if (wc)  return;
//while (MsgToBeRead())
//	{
//	const ETMessage *rxMsg = GetNextMsg();
//	message_rec++;
//	if (rxMsg->ReadType() == 2)
  //      	{
	//	angle_pitch=*((int32_t*)rxMsg->ReadData());
	//	angle_roll= *((int32_t*)((char*)(rxMsg->ReadData())+sizeof(int32_t)));
	//	angle_yaw=  *((int32_t*)((char*)(rxMsg->ReadData())+2*sizeof(int32_t)));
      //  acc_x = *((int16_t*)((char*)(rxMsg->ReadData())+3*sizeof(int32_t)));
      //  acc_y = *((int16_t*)((char*)(rxMsg->ReadData())+3*sizeof(int32_t)+sizeof(int16_t)));
//		acc_z = *((int16_t*)((char*)(rxMsg->ReadData())+3*sizeof(int32_t)+2*sizeof(int16_t)));
//		}
//	}
//	RemoveCurrentMsg();
    //arucoSetup();
	sem_wait(&mutex);
	arucoGrabCycle();
	TheMarkers = TheMarkersOriginal;
	sem_post(&mutex);
    
	for (unsigned int i=(mn*movingwindow)-1; i>(mn-1); i--)
		{
		x_ext_back[i]=x_ext_back[i-mn];
		y_ext_back[i]=y_ext_back[i-mn];
		z_ext_back[i]=z_ext_back[i-mn];
		yaw_ext_back[i]=yaw_ext_back[i-mn];
		}
	for (unsigned int i=movingwindow-1; i>0; i--)
		{
		averagex[i]=averagex[i-1];
		averagey[i]=averagey[i-1];
		averagez[i]=averagez[i-1];
		averageyaw[i]=averageyaw[i-1];
		}
	for (unsigned int i=0;i<mn;i++){
		x_ext_back[i] = x_ext[i];
		y_ext_back[i] = y_ext[i];
		z_ext_back[i] = z_ext[i];
		yaw_ext_back[i] = yaw_ext[i];	
		}
	for (unsigned int i=0;i<mn;i++){
		x_ext[i] = -1000;
		y_ext[i] = -1000;
		z_ext[i] = -1000;
		yaw_ext[i] = -1000;
		}

	printTimeCounter++;
	//check if there is a new frame
	 if(TheMarkers.size() >0){
        // printf("%d \n", TheMarkers.size());
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
		aPsi_back=aPsi;

		for (int m =(TheMarkers.size()-1);m>=0;m--) {
			/*if (TheMarkers[m].id == 1021){
				continue;
			}*/
			cv::Rodrigues(TheMarkers[m].Rvec, Rcv); // R is 3x3
			Rcv = Rcv.t();  // rotation of inverse
			TheMarkers[m].Tvec = -Rcv * TheMarkers[m].Tvec; // translation of inverse
			///marker position with respect to the camera ( - per invertire gli assi)
			xc = -TheMarkers[m].Tvec.at<float>(1,0); //aruco y
			yc = -TheMarkers[m].Tvec.at<float>(0,0); //aruco x
			zc = -TheMarkers[m].Tvec.at<float>(2,0); //aruco z
			aPhi = asin(-Rcv.at<float>(2,0)); //roll calculated with aruco
			double cosphi = cos(aPhi);
			//aTheta =- ((acos(Rcv.at<float>(2,2)/cosphi)) - M_PI/2.0); //pitch calculated with aruco
			aPsi = asin(Rcv.at<float>(1,0)/cosphi); // yaw calculated with aruco
			 //rotation matrix R linerized:
			 // 11 12 13 21 22 23 31 32 33
			 //using theta and phi from asctec and psi from aruco
			 static double *R = new double[9];
			 //first row
			 R[0] = cos(theta) * cos(aPsi);
			 R[1] = - cos(phi) * sin(aPsi) + sin(phi) * sin(theta) * cos(aPsi);
			 R[2] =  sin(phi) * sin(aPsi) + cos(phi) * sin(theta) * cos(aPsi);
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
			 xom = R[0]*xoc + R[1]*yoc + R[2]*zoc;
			 yom = R[3]*xoc + R[4]*yoc + R[5]*zoc;
			 zom = R[6]*xoc + R[7]*yoc + R[8]*zoc;

			 //robot position
			 xr = xom + xc;
			 yr = yom + yc;
			 zr = zom + zc;
			switch(TheMarkers[m].id){
				case 65:
                    x_ext[0]=xr;
                    y_ext[0]=yr;
                    z_ext[0]=zr;
                    yaw_ext[0]=(aPsi*180.0/M_PI);
                    break;
				case 123:
                    x_ext[1]=xr;
                    y_ext[1]=yr-0.004;
                    z_ext[1]=zr+0.593;
                    yaw_ext[1]=(aPsi*180.0/M_PI);
                    break;
				case 243:
                    x_ext[2]=xr;
                    y_ext[2]=yr;
                    z_ext[2]=zr-0.543;
                    yaw_ext[2]=(aPsi*180.0/M_PI);
                    break;
				case 313:
                    x_ext[3]=xr;
                    y_ext[3]=yr-0.557;
                    z_ext[3]=zr-0.004;
                    yaw_ext[3]=(aPsi*180.0/M_PI);
                    break;
				case 402:
                    x_ext[4]=xr;
                    y_ext[4]=yr-0.557;
                    z_ext[4]=zr-0.545;
                    yaw_ext[4]=(aPsi*180.0/M_PI);
                    break;
				case 532:
                    x_ext[5]=xr;
                    y_ext[5]=yr-0.559;
                    z_ext[5]=zr+0.590;
                    yaw_ext[5]=(aPsi*180.0/M_PI);
                    break;
				case 653:
                    x_ext[6]=xr;
                    y_ext[6]=yr+0.526;
                    z_ext[6]=zr-0.003;
                    yaw_ext[6]=(aPsi*180.0/M_PI);
                    break;
				case 741:
                    x_ext[7]=xr;
                    y_ext[7]=yr+0.527;
                    z_ext[7]=zr+0.588;
                    yaw_ext[7]=(aPsi*180.0/M_PI);
                    break;
				case 888:
                    x_ext[8]=xr;
                    y_ext[8]=yr+0.527;
                    z_ext[8]=zr-0.544;
                    yaw_ext[8]=(aPsi*180.0/M_PI);
                    break;
                case 200:
                    x_ext[9]=xr;
                    y_ext[9]=yr-0.556;
                    z_ext[9]=zr+0.932;
                    yaw_ext[9]=(aPsi*180.0/M_PI);
                    break;
                case 201:
                    x_ext[10]=xr;
                    y_ext[10]=yr-0.555;
                    z_ext[10]=zr+1.171;
                    yaw_ext[10]=(aPsi*180.0/M_PI);
                    break;
                case 202:
                    x_ext[11]=xr;
                    y_ext[11]=yr-0.002;
                    z_ext[11]=zr+0.929;
                    yaw_ext[11]=(aPsi*180.0/M_PI);
                    break;
                case 203:
                    x_ext[12]=xr;
                    y_ext[12]=yr-0.003;
                    z_ext[12]=zr+1.173;
                    yaw_ext[12]=(aPsi*180.0/M_PI);
                    break;
                case 204:
                    x_ext[13]=xr;
                    y_ext[13]=yr+0.529;
                    z_ext[13]=zr+0.933;
                    yaw_ext[13]=(aPsi*180.0/M_PI);
                    break;
                case 205:
                    x_ext[14]=xr;
                    y_ext[14]=yr+0.528;
                    z_ext[14]=zr+1.177;
                    yaw_ext[14]=(aPsi*180.0/M_PI);
                    break;
                case 206:
                    x_ext[15]=xr;
                    y_ext[15]=yr-0.558;
                    z_ext[15]=zr+0.268;
                    yaw_ext[15]=(aPsi*180.0/M_PI);
                    break;
                case 207:
                    x_ext[16]=xr;
                    y_ext[16]=yr+0.002;
                    z_ext[16]=zr+0.258;
                    yaw_ext[16]=(aPsi*180.0/M_PI);
                    break;
                case 208:
                    x_ext[17]=xr;
                    y_ext[17]=yr+0.529;
                    z_ext[17]=zr+0.250;
                    yaw_ext[17]=(aPsi*180.0/M_PI);
                    break;
                case 209:
                    x_ext[18]=xr;
                    y_ext[18]=yr-0.554;
                    z_ext[18]=zr-0.286;
                    yaw_ext[18]=(aPsi*180.0/M_PI);
                    break;
                case 210:
                    x_ext[19]=xr;
                    y_ext[19]=yr;
                    z_ext[19]=zr-0.286;
                    yaw_ext[19]=(aPsi*180.0/M_PI);
                    break;
                case 211:
                    x_ext[20]=xr;
                    y_ext[20]=yr+0.528;
                    z_ext[20]=zr-0.289;
                    yaw_ext[20]=(aPsi*180.0/M_PI);
                    break;
                case 212:
                    x_ext[21]=xr;
                    y_ext[21]=yr-0.285;
                    z_ext[21]=zr+0.932;
                    yaw_ext[21]=(aPsi*180.0/M_PI);
                    break;
                case 213:
                    x_ext[22]=xr;
                    y_ext[22]=yr-0.283;
                    z_ext[22]=zr+1.171;
                    yaw_ext[22]=(aPsi*180.0/M_PI);
                    break;
                case 214:
                    x_ext[23]=xr;
                    y_ext[23]=yr+0.258;
                    z_ext[23]=zr+0.929;
                    yaw_ext[23]=(aPsi*180.0/M_PI);
                    break;
                case 215:
                    x_ext[24]=xr;
                    y_ext[24]=yr+0.259;
                    z_ext[24]=zr+1.172;
                    yaw_ext[24]=(aPsi*180.0/M_PI);
                    break;
                case 216:
                    x_ext[25]=xr;
                    y_ext[25]=yr-0.280;
                    z_ext[25]=zr+0.590;
                    yaw_ext[25]=(aPsi*180.0/M_PI);
                    break;
                case 217:
                    x_ext[26]=xr;
                    y_ext[26]=yr+0.259;
                    z_ext[26]=zr+0.588;
                    yaw_ext[26]=(aPsi*180.0/M_PI);
                    break;
                case 218:
                    x_ext[27]=xr;
                    y_ext[27]=yr-0.285;
                    z_ext[27]=zr+0.260;
                    yaw_ext[27]=(aPsi*180.0/M_PI);
                    break;
                case 219:
                    x_ext[28]=xr;
                    y_ext[28]=yr-0.287;
                    z_ext[28]=zr-0.006;
                    yaw_ext[28]=(aPsi*180.0/M_PI);
                    break;
                case 220:
                    x_ext[29]=xr;
                    y_ext[29]=yr-0.287;
                    z_ext[29]=zr-0.284;
                    yaw_ext[29]=(aPsi*180.0/M_PI);
                    break;
                case 221:
                    x_ext[30]=xr;
                    y_ext[30]=yr-0.286;
                    z_ext[30]=zr-0.545;
                    yaw_ext[30]=(aPsi*180.0/M_PI);
                    break;
                case 222:
                    x_ext[31]=xr;
                    y_ext[31]=yr+0.258;
                    z_ext[31]=zr+0.250;
                    yaw_ext[31]=(aPsi*180.0/M_PI);
                    break;
                case 223:
                    x_ext[32]=xr;
                    y_ext[32]=yr+0.256;
                    z_ext[32]=zr-0.009;
                    yaw_ext[32]=(aPsi*180.0/M_PI);
                    break;
                case 224:
                    x_ext[33]=xr;
                    y_ext[33]=yr+0.257;
                    z_ext[33]=zr-0.288;
                    yaw_ext[33]=(aPsi*180.0/M_PI);
                    break;
                case 225:
                    x_ext[34]=xr;
                    y_ext[34]=yr+0.258;
                    z_ext[34]=zr-0.547;
                    yaw_ext[34]=(aPsi*180.0/M_PI);
                    break;
            }

			}
		////average of the first maxCounter (valid) values

		if (averageCounter1 <= maxCounter){

			markerCounter=0;
			markerx=0;
			markery=0;
			markerz=0;
			markeryaw=0;
			for (unsigned int i=0;i<mn;i++){
				if ((y_ext[i]!=-1000)){
					markerCounter++;
					markerx=markerx+x_ext[i];
					markery=markery+y_ext[i];
					markerz=markerz+z_ext[i];
					markeryaw=markeryaw+yaw_ext[i];
				}
			}
			if (markerCounter!=0){
				tempx=markerx/markerCounter;
				tempy=markery/markerCounter;
				tempz=markerz/markerCounter;
				tempyaw=markeryaw/markerCounter;
			}

			start_average_x=start_average_x+tempx;
			start_average_y=start_average_y+tempy;
			start_average_z=start_average_z+tempz;
			start_average_yaw=start_average_yaw+tempyaw;
			averageCounter1++;

			if (averageCounter1==maxCounter)
			{
				start_average_x=start_average_x/maxCounter;
				start_average_y=start_average_y/maxCounter;
				start_average_z=start_average_z/maxCounter;
				start_average_yaw=start_average_yaw/maxCounter;
				averageCounter1++;
				for (int i=0;i<movingwindow*mn;i++){
					x_ext_back[i]= start_average_x;
					y_ext_back[i]= start_average_y;
					z_ext_back[i]= start_average_z;
					yaw_ext_back[i]= start_average_yaw;
				}
				for (int i=0;i<movingwindow;i++){
					averagex[i]= start_average_x;
					averagey[i]= start_average_y;
					averagez[i]= start_average_z;
					averageyaw[i]= start_average_yaw;
				}
			}
		}///marker position extimation
		else {
			markerCounter = 0;
			markerx = 0;
			markery = 0;
			markerz = 0;
			markeryaw = 0;
			averageCounter2 = 0;

			for (unsigned int i = 0; i < mn; i++){
				if ((y_ext[i] != -1000)){
					averageCounter2 = averageCounter2 + 1;
					markerx = markerx + x_ext[i];
					markery = markery + y_ext[i];
					markerz = markerz + z_ext[i];
					markeryaw = markeryaw + yaw_ext[i];
				}
			}

			if (averageCounter2 != 0){
				averagex[0] = markerx / averageCounter2;
				averagey[0] = markery / averageCounter2;
				averagez[0] = markerz / averageCounter2;
				averageyaw[0] = markeryaw / averageCounter2;
			}

			else{
				averagex[0] = averagex[1];
				averagey[0] = averagey[1];
				averagez[0] = averagez[1];
				averageyaw[0] = averageyaw[1];
			}

			markerx = 0;
			markery = 0;
			markerz = 0;
			markeryaw = 0;

			for (unsigned int i = 0; i < mn; i++){
				if (y_ext[i] != -1000){
					if (y_ext_back[i] != -1000){
						dist = abs(y_ext[i] - y_ext_back[i]);
					}
					else{
						dist = 0;
					}
					if (dist < thr1){
						for (int k = 0; k < movingwindow; k++){
							if (y_ext_back[i + mn*k] != -1000){
								x_ext[i] = x_ext[i] + x_ext_back[i + mn*k];
								y_ext[i] = y_ext[i] + y_ext_back[i + mn*k];
								z_ext[i] = z_ext[i] + z_ext_back[i + mn*k];
								yaw_ext[i] = yaw_ext[i] + yaw_ext_back[i + mn*k];
							}
							else{

								x_ext[i] = x_ext[i] + averagex[k];
								y_ext[i] = y_ext[i] + averagey[k];
								z_ext[i] = z_ext[i] + averagez[k];
								yaw_ext[i] = yaw_ext[i] + averageyaw[k];
							}
						}
						x_ext[i] = x_ext[i] / (movingwindow + 1);
						y_ext[i] = y_ext[i] / (movingwindow + 1);
						z_ext[i] = z_ext[i] / (movingwindow + 1);
						yaw_ext[i] = yaw_ext[i] / (movingwindow + 1);
					}
					else{
						x_ext[i] = x_ext_back[i];
						y_ext[i] = y_ext_back[i];
						z_ext[i] = z_ext_back[i];
						yaw_ext[i] = yaw_ext_back[i];

					}
				}

				if ((y_ext[i] != -1000)&(y_ext_back[i] != -1000)&(y_ext_back[i + mn] != -1000)){
					markerCounter++;
					markerx = markerx + x_ext[i];
					markery = markery + y_ext[i];
					markerz = markerz + z_ext[i];
					markeryaw = markeryaw + yaw_ext[i];
				}

			}

			if (markerCounter != 0){
				xr = markerx / markerCounter;
				yr = markery / markerCounter;
				zr = markerz / markerCounter;
				aPsi = (markeryaw / markerCounter)*M_PI / 180;
			}
			else{
				xr = xr_back;
				yr = yr_back;
				zr = zr_back;
				aPsi = aPsi_back;
			}
		}
	gettimeofday(&endtime, NULL);
	secc=endtime.tv_sec - starttime.tv_sec;
	msec=endtime.tv_usec-starttime.tv_usec;
	mtime = ((secc)+(msec/1000000.0)); //time in sec

	accum_time=accum_time+mtime;


	dxr=(xr-xr_back)/mtime;
	dyr=(yr-yr_back)/mtime;
	dzr=(zr-zr_back)/mtime;
	dPsi=(aPsi-aPsi_back)/mtime;

	gettimeofday(&starttime, NULL);
	


for(int i = 0; i < StatusSize; i++){
	shiftBack(measurements[i], SAMPLESLEN);
	}


measurements[X][SAMPLESLEN-1] = xr;
measurements[Y][SAMPLESLEN-1] = yr;
measurements[Z][SAMPLESLEN-1] = zr;
measurements[Yaw][SAMPLESLEN-1] = aPsi;
measurements[Dx][SAMPLESLEN-1] = dxr;
measurements[Dy][SAMPLESLEN-1] = dyr;
measurements[Dz][SAMPLESLEN-1] = dzr;
measurements[Dyaw][SAMPLESLEN-1] = dPsi;
	
xmsg= median(measurements[X],SAMPLESLEN,sorted);
ymsg= median(measurements[Y],SAMPLESLEN,sorted);
zmsg= median(measurements[Z],SAMPLESLEN,sorted);
yawmsg= median(measurements[Yaw],SAMPLESLEN,sorted);
dxmsg= median(measurements[Dx],SAMPLESLEN,sorted);
dymsg= median(measurements[Dy],SAMPLESLEN,sorted);
dzmsg= median(measurements[Dz],SAMPLESLEN,sorted);
dyawmsg= median(measurements[Dyaw],SAMPLESLEN,sorted);
		

if(printTimeCounter >= printstep )
         {
	//printf("robot position x, y, z, yaw: %f %f %f %f \n", xmsg, ymsg, zmsg, yawmsg*180/M_PI);
	//printf("robot velocity dx, dy, dz, dpsi: %f %f %f %f \n", dxmsg, dymsg, dzmsg, dyawmsg*180/M_PI);
    //printf("robot acceleration x, y, z: %f %f %f \n", acc_x*9.81/10000, acc_y*9.81/10000, acc_z*9.81/10000);
	printTimeCounter = 0;
}
	
}

else 
	{
	gettimeofday(&endtime, NULL);
	secc=endtime.tv_sec - starttime.tv_sec;
	msec=endtime.tv_usec-starttime.tv_usec;
	mtime = ((secc)+(msec/1000000.0)); //time in sec
	accum_time=accum_time+mtime;
	gettimeofday(&starttime, NULL);
	if(printTimeCounter >= printstep ){
		printf("\n TARGET LOST!! \n");
		printTimeCounter = 0;
		}
	}

//ofstream fs("positions_xyz.txt", std::ofstream::out | std::ofstream::app);
//if(fs.is_open()){
//	fs << accum_time << "\t";
//	fs << xmsg << "\t" << ymsg << "\t" << zmsg << "\t" << yawmsg*180/M_PI << "\t";
//    fs << dxmsg << "\t" << dymsg << "\t" << dzmsg << "\t" << dyawmsg*180/M_PI << "\t";
//	fs << theta *180/M_PI << "\t" << phi *180/M_PI << "\t" << psi << "\t";
//	fs << acc_x*9.81/10000 << "\t" << acc_y*9.81/10000 << "\t" << acc_z*9.81/10000 << "\t";
//	fs << "\n";
//	} else {
//	cerr << "File not open!" << endl;
//	}


	ETMessage *txMessage = new ETMessage(8*sizeof(double), SendType);
	*((double*)txMessage->GetData())=xmsg;
	*((double*)((char*)(txMessage->GetData())+sizeof(double)))=ymsg;
	*((double*)((char*)(txMessage->GetData())+2*sizeof(double)))=zmsg;
	*((double*)((char*)(txMessage->GetData())+3*sizeof(double)))=yawmsg;
	*((double*)((char*)(txMessage->GetData())+4*sizeof(double)))=dxmsg;
	*((double*)((char*)(txMessage->GetData())+5*sizeof(double)))=dymsg;
	*((double*)((char*)(txMessage->GetData())+6*sizeof(double)))=dzmsg;
	*((double*)((char*)(txMessage->GetData())+7*sizeof(double)))=dyawmsg;
	ShareMsg(txMessage);
	message_sent++;
	//sem_post(&mutex);
}

void
Module3::Close()
{
	delete[] sorted;
	delete[] occurrences;
 	printf("\nPeriodic cameras N %d missed %ld  deadlines ",ExpertId, NMissedDeadLine);
 	printf("  and executed %ld times ",message_sent);
}
