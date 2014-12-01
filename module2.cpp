// IMU   //
// 100hz //

//asctec
#include "asctecCommIntf.h"
//header
#include "module2.h"
//c
#include <unistd.h> // UNIX standard function definitions
#include <fcntl.h>  // File control definitions
#include <termios.h> // POSIX terminal control definitionss
#include <pthread.h>
#include <time.h>
#include <stdio.h>
#include <sys/time.h>
//c++
#include <cmath>
#include <fstream>
#include <sstream>
#include <iostream>

using namespace std;

/// GLOBAL VARIABLES
u_int8_t rpm[6];
int16_t CTRL_pitch;
int16_t CTRL_roll;
int16_t CTRL_yaw;
u_int16_t CTRL_thrust;

int32_t angle_pitch;
int32_t angle_roll;
int32_t angle_yaw;
int16_t acc_x;
int16_t acc_y;
int16_t acc_z;
int fd;
int var_getted;
int initialize;
int ended;
int wp;
//int height;
int landing;
int flag;


int32_t GPS_latitude;
int32_t GPS_longitude;
int32_t GPS_height;
int32_t fusion_latitude;
int32_t fusion_longitude;
int32_t fusion_height;
int32_t GPS_heading;
u_int32_t position_accuracy;
u_int32_t height_accuracy;
u_int32_t GPS_num;
int32_t GPS_status;
int32_t fusion_speed_x;
int32_t fusion_speed_y;
int32_t fusion_speed_z;
int32_t GPS_speed_x;
int32_t GPS_speed_y;

///////////////

void Module2::initializeAccSaving() {
    //fs = new ofstream ("accelerations_xyz.txt", std::ofstream::out | std::ofstream::trunc);
    ofstream fs ("accelerations_xyz.txt", std::ofstream::out | std::ofstream::trunc);
	//if(fs->is_open()){
    if(fs.is_open()){
	//	(*fs) << "Accelerations (x,y,z) \n";
        fs <<"Accelerations (x,y,z) \n";
		cout << "File with accelerations data initialized" << endl;
	} else {
		cerr << "File not open!" << endl;
	}
    
}

unsigned char readSerial(int result, unsigned char data) {
    result = read(fd, &data, 1);
    while (result != -1 && result != 0) {
        aciReceiveHandler(data);
        result = read(fd, &data, 1);
    }
    return data;
}


Module2::Module2 (ETDispatch* dis, long int period, int ac)
:ETExpert(dis,period, PRIORITY_AUTO,0,EXPERT_USER)
{
 	SendType = ac;
    
	
}

Module2::~Module2()
{
}

void
Module2::Init()
{
	message_sent=0;
    //fs = NULL;
    //initializeAccSaving();
}


void
Module2::DoYourDuty (int wc)
{
    //printf(".");
    //fflush(stdout);
	if (wc)  return;
    unsigned char data = 0;
    int result = 0;
    data = readSerial(result, data);
	if (var_getted)
	{
    aciSynchronizeVars();
    ETMessage *txMessage = new ETMessage(3*sizeof(int32_t)+3*sizeof(int16_t), SendType);
    *((int32_t*)txMessage->GetData())=angle_pitch;
    *((int32_t*)((char*)(txMessage->GetData())+sizeof(int32_t)))=angle_roll;
    *((int32_t*)((char*)(txMessage->GetData())+2*sizeof(int32_t)))=angle_yaw;
    *((int16_t*)((char*)(txMessage->GetData())+3*sizeof(int32_t)))=acc_x;
    *((int16_t*)((char*)(txMessage->GetData())+3*sizeof(int32_t)+sizeof(int16_t)))=acc_y;
    *((int16_t*)((char*)(txMessage->GetData())+3*sizeof(int32_t)+2*sizeof(int16_t)))=acc_z;
    ShareMsg(txMessage);
        //ofstream fs("accelerations_xyz.txt", std::ofstream::out | std::ofstream::app);
       // if(fs->is_open()){
        //if(fs.is_open()){
   // (*fs) << acc_x*9.81/10000 << "\t" << acc_y*9.81/10000 << "\t" << acc_z*9.81/10000 << "\t" << "\n";
        //fs << acc_x*9.81/10000 << "\t" << acc_y*9.81/10000 << "\t" << acc_z*9.81/10000 << "\t" << "\n";
        //} else
       // {
        //    cerr << "File not open!" << endl;
       // }
	}
    
    if ((initialize==1)&(ended==0)){
    aciUpdateCmdPacket(cmdCTRL);   
    }
	message_sent++;
	//fflush(stdout);
    aciEngine();
    //printf(":");
    //fflush(stdout);
}

void
Module2::Close()
{
 	printf("\nPeriodic N %d missed %ld  deadlines ",ExpertId, NMissedDeadLine);
 	printf("  and executed %ld times ",message_sent);
    fflush(stdout);
}
