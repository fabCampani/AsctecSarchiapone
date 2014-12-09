//    USER      //
// INTERFACE   //
/////////////////

#include "asctecCommIntf.h"

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
#include <fcntl.h>

#include "module1.h"
#include "module2.h"



using namespace std;

//// GLOBAL VARIABLES (defined in module2.cpp)
extern int16_t CTRL_pitch;
extern int16_t CTRL_roll;
extern int16_t CTRL_yaw;
extern u_int16_t CTRL_thrust;
extern int var_getted;
extern int32_t angle_pitch;
extern int32_t angle_roll;
extern int32_t angle_yaw;
extern int16_t acc_x;
extern int16_t acc_y;
extern int16_t acc_z;
extern int fd;
extern int initialize;
extern int ended;
extern u_int8_t rpm[6];
extern int wp;
//extern int height;
extern int landing;
extern int flag;
//////////////////////
extern int landing;
extern int flag;

extern int32_t GPS_latitude;
extern int32_t GPS_longitude;
extern int32_t GPS_height;
extern int32_t fusion_latitude;
extern int32_t fusion_longitude;
extern int32_t fusion_height;
extern int32_t GPS_heading;
extern u_int32_t position_accuracy;
extern u_int32_t height_accuracy;
extern u_int32_t GPS_num;
extern int32_t GPS_status;
extern int32_t fusion_speed_x;
extern int32_t fusion_speed_y;
extern int32_t fusion_speed_z;
extern int32_t GPS_speed_x;
extern int32_t GPS_speed_y;


u_int8_t ctrl_mode = 0;
u_int8_t ctrl_enabled = 0;
u_int8_t disable_motor_onoff_by_stick = 0;
int16_t CTRL_ctrl=15;
unsigned char m[6];
unsigned char cmd_ready = 0;
unsigned char motor_start=1;

bool motors = false;

void transmit(void* byte, unsigned short cnt);
void varListUpdateFinished();
void cmdListUpdateFinished();
void paramListUpdateFinished();
void versions(struct ACI_INFO);
void stopMotors();
void startMotors();
bool spinning();

void serialSetup() {
	fd = open("/dev/ttyUSB0", O_RDWR | O_NOCTTY | O_NDELAY | O_NONBLOCK);
	struct termios port_settings; // structure to store the port settings in
	cfsetispeed(&port_settings, B57600);
	port_settings.c_cflag = B57600 | CS8 | CREAD | CLOCAL;
	port_settings.c_iflag = IGNPAR;
	port_settings.c_oflag = 0;
	port_settings.c_lflag = 0;
	tcsetattr(fd, TCSANOW, &port_settings);
}

void asctecSetup() {
	//asctec communication
	aciInit();
	aciSetSendDataCallback(&transmit);
	aciInfoPacketReceivedCallback(&versions);
	aciSetVarListUpdateFinishedCallback(&varListUpdateFinished);
	aciSetCmdListUpdateFinishedCallback(&cmdListUpdateFinished);
	aciSetParamListUpdateFinishedCallback(&paramListUpdateFinished);
	aciSetEngineRate(100, 10);
}

void initializeAsctec() {
	aciCheckVerConf();
	aciGetDeviceVariablesList();
	aciGetDeviceCommandsList();
	printf("\n Waiting for command list...\n");
	while (!cmd_ready) {
        fflush(stdout);
        usleep(1);
        //printf(".");
	}
}

void transmit(void* byte, unsigned short cnt) {
    
    unsigned char *tbyte = (unsigned char *) byte;
    for (int i = 0; i < cnt; i++) {
        write(fd, &tbyte[i], 1);
    }
}


Module1::Module1 (ETDispatch* dis, long int period, int sm)
:ETExpert(dis,period, PRIORITY_AUTO,0,EXPERT_USER)
{
	SendType = sm;
}

Module1::~Module1()
{
}

void
Module1::Init()
{
    var_getted=0;
	message_sent=0;
    initialize=0;
    waypnum = 4; //number of waypoints. Default = 4.
    serialSetup();
	asctecSetup();
    ended=0;
    initialize3=0;
    landing=0;
    flag=0;
	ifstream fs("motors.txt");
	if (fs.is_open()){
		int m;
		fs >> m;
		if (m == 1){
			motors = true;
		}
	}
}

void
Module1::DoYourDuty (int wc)
{
    
    
	if (wc)  return;
    while (initialize3==0){
        
        initializeAsctec();
        aciSetEngineRate(100, 100);
        getchar();
		if (motors){
			startMotors();
		}
        CTRL_thrust = 0; //almost take off
        CTRL_pitch = 0;
        CTRL_roll = 0;
        CTRL_yaw = 0;
        printf("\n\nMotor starting \n\n");
        sleep(1);
        CTRL_thrust = 1000;  //1900
        aciUpdateCmdPacket(cmdCTRL);
        for (unsigned int i=0; i<20; i++)
        {
            CTRL_thrust = CTRL_thrust+20;  //1900
            aciUpdateCmdPacket(cmdCTRL);
            usleep(100000);
            printf("step %d, thrust %d \n", i, CTRL_thrust);
        }
        printf("\n\n\n\n\n ok! \n\n\n\n\n");
        sleep(1);
        initialize3=1;
        cout << "********************************************" << endl;
        cout << "*         press P to take a picture        *" << endl;
        cout << "* press K to change the control parameters *" << endl;
		cout << "*       press o to set position offset     *" << endl;
        cout << "*              press T to take off         *" << endl;
        cout << "*          press S to stop the motors      *" << endl;
        cout << "********************************************" << endl;
        fcntl(0, F_SETFL, O_NONBLOCK);
        
    }
    //printf("%d", var_getted);
    //printf("%d", ctrl_mode);
    fcntl(0, F_SETFL, O_NONBLOCK);
	char x=47;
    int w;
	scanf("%c", &x);
    w=int(x)-48;
    //printf("%d",w);
	fcntl(0, F_SETFL, O_NONBLOCK);
		if (x=='k')				//parameters
        {
            ETMessage *txMessage = new ETMessage(1, /*SendType*/ 1);
            *txMessage->GetData()=1;
            ShareMsg(txMessage);
        }
        else if (x=='p')			//picture
		{
            ETMessage *txMessage = new ETMessage(1, 5);
            *txMessage->GetData()=99;
            ShareMsg(txMessage);
        }
        else if (x=='s')			//Stop
        {
            ended=1;
            printf("\n Stopping \n");           
            //CTRL_thrust = 2300; //1300
            //aciUpdateCmdPacket(cmdCTRL);
            //sleep(1);      
            stopMotors();
            landing=0;
            flag=0;
            sleep(1);
            printf("\nPress CRTL+C to exit or r to fly again\n");
        }
        else if (x=='t')			//take off
        {
            printf("\n FLYING!! \n");
            ETMessage *txMessage = new ETMessage(1, 1);
            *txMessage->GetData()=96;
            ShareMsg(txMessage);
            initialize=1;
        }    
        else if (x=='a')			//landing
        {
            printf("\n LANDING!! \n");
            /*ETMessage *txMessage = new ETMessage(1, 5);
            *txMessage->GetData()=95;
            ShareMsg(txMessage);*/
			landing = 1;
        }
		else if (x == 'o')
		{
			ETMessage *txMessage = new ETMessage(1, /*SendType*/ 1);
			*txMessage->GetData() = 94;
			ShareMsg(txMessage);
		}
    
        else if ((x=='r')&(ended==1))		//restart
        {
            landing=0;
            initialize=0;
            //initialize3=0;
			if (motors){
				startMotors();
			}
            CTRL_thrust = 0; //almost take off
            CTRL_pitch = 0;
            CTRL_roll = 0;
            CTRL_yaw = 0;
            
            printf("\n\nMotor starting \n\n");
            sleep(1);
            CTRL_thrust = 1000;  //1900
            aciUpdateCmdPacket(cmdCTRL);
            for (unsigned int i=0; i<20; i++)
            {
                CTRL_thrust = CTRL_thrust+20;  //2200
                aciUpdateCmdPacket(cmdCTRL);
                usleep(100000);
                printf("step %d, thrust %d \n", i, CTRL_thrust);
            }
            printf("\n\n\n\n\n ok!!  \n\n\n\n\n");
            sleep(1);
            //initialize3=1;
            ended=0;
            cout << "********************************************" << endl;
            cout << "*         press P to take a picture        *" << endl;
            cout << "* press K to change the control parameters *" << endl;
			cout << "*       press o to set position offset     *" << endl;
            cout << "*              press T to take off         *" << endl;
            cout << "*          press S to stop the motors      *" << endl;
            cout << "********************************************" << endl;
            fcntl(0, F_SETFL, O_NONBLOCK);
        }

            
    
    //aciUpdateCmdPacket(cmdCTRL);
    //aciEngine();
    message_sent++;
    fflush(stdout);
}

void
Module1::Close()
{
    printf("\nPeriodic N %d missed %ld  deadlines ",ExpertId, NMissedDeadLine);
 	printf("  and executed %ld times ",message_sent);
    
	close(fd);
    printf("\n Exiting \n");
    fflush(stdout);
	//printParams();
 	}




void cmdListUpdateFinished() {
    printf("\n command list getted!\n");
    
    ctrl_mode=2;
    ctrl_enabled=1;
    disable_motor_onoff_by_stick=1;
    CTRL_ctrl = 15;
    aciAddContentToCmdPacket(Configuration, 0x0600, &ctrl_mode);
    aciAddContentToCmdPacket(Configuration, 0x0601, &ctrl_enabled);
    aciAddContentToCmdPacket(Configuration, 0x0602, &disable_motor_onoff_by_stick);
    aciSendCommandPacketConfiguration(Configuration, 1); //w/ ack
    aciUpdateCmdPacket(Configuration);
    
    aciAddContentToCmdPacket(cmdCTRL, 0x050A, &CTRL_pitch);
    aciAddContentToCmdPacket(cmdCTRL, 0x050B, &CTRL_roll);
    aciAddContentToCmdPacket(cmdCTRL, 0x050C, &CTRL_yaw);
    aciAddContentToCmdPacket(cmdCTRL, 0x050D, &CTRL_thrust);
    aciAddContentToCmdPacket(cmdCTRL, 0x050E, &CTRL_ctrl);
    aciSendCommandPacketConfiguration(cmdCTRL, 0); //w/o ack
    
    for(int i = 0; i <6; i++){
        m[i]=0;
        aciAddContentToCmdPacket(cmdDIMC, 0x0500+i, &m[i]);
    }
    aciSendCommandPacketConfiguration(cmdDIMC, 0); //w/o ack
    
    cmd_ready=1;
    //printf("qui\n");
}

void paramListUpdateFinished() {
    
}


void startMotors(){
    printf("\n Starting...\n");
    
    if(ctrl_mode!=2){
        ctrl_mode=2;
        aciUpdateCmdPacket(Configuration);
        printf("\n start: configuration re-sent!\n");
    }
    
    CTRL_yaw = 0;
    aciUpdateCmdPacket(cmdCTRL);
    
    CTRL_pitch = 0;
    CTRL_roll = 0;
    CTRL_yaw = -2047;
    CTRL_thrust = 0;
    int attempts=0;
    time_t beginning = time(NULL);
    do{
        aciUpdateCmdPacket(cmdCTRL);
        attempts++;
        usleep(10000);
    }while(!spinning());
    double duration = difftime(time(NULL),beginning);
    CTRL_yaw = 0;
    aciUpdateCmdPacket(cmdCTRL);
    cout << "Started!" << endl;
    printf("took %d attempts for a total %d seconds\n",attempts, (int)duration);
}

void stopMotors(){
    printf("Stopping!\n");
    
    if(ctrl_mode!=2){
        ctrl_mode=2;
        aciUpdateCmdPacket(Configuration);
        printf("stop: configuration re-sent!\n");
    }
    CTRL_yaw = 0;
    aciUpdateCmdPacket(cmdCTRL);
    
    CTRL_pitch = 0;
    CTRL_roll = 0;
    CTRL_yaw = 2047;
    CTRL_thrust = 0;
    
    int attempts=0;
    time_t beginning = time(NULL);
    do{
        aciUpdateCmdPacket(1);
        attempts++;
        usleep(10000);
    }while(spinning());
    double duration = difftime(time(NULL),beginning);
    
    printf("took %d attempts for a total %d seconds\n",attempts, (int)duration);
    
    CTRL_yaw = 0;
    aciUpdateCmdPacket(cmdCTRL);
}

void versions(struct ACI_INFO aciInfo) {
    printf("******************** Versions *******************\n");
    printf("* Type\t\t\tDevice\t\tRemote\t*\n");
    printf("* Major version\t\t%d\t=\t\%d\t*\n",aciInfo.verMajor,ACI_VER_MAJOR);
    printf("* Minor version\t\t%d\t=\t\%d\t*\n",aciInfo.verMinor,ACI_VER_MINOR);
    printf("* MAX_DESC_LENGTH\t%d\t=\t\%d\t*\n",aciInfo.maxDescLength,MAX_DESC_LENGTH);
    printf("* MAX_NAME_LENGTH\t%d\t=\t\%d\t*\n",aciInfo.maxNameLength,MAX_NAME_LENGTH);
    printf("* MAX_UNIT_LENGTH\t%d\t=\t\%d\t*\n",aciInfo.maxUnitLength,MAX_UNIT_LENGTH);
    printf("* MAX_VAR_PACKETS\t%d\t=\t\%d\t*\n",aciInfo.maxVarPackets,MAX_VAR_PACKETS);
    printf("*************************************************\n");
    fflush(stdout); //added to really print the table
}

void varListUpdateFinished() {
    printf("\n variables updated \n\n\n\n");
    aciAddContentToVarPacket(0,0x0300,&angle_pitch);
    aciAddContentToVarPacket(0,0x0301,&angle_roll);
    aciAddContentToVarPacket(0,0x0302,&angle_yaw);
    aciAddContentToVarPacket(0,0x0106,&GPS_latitude);
    aciAddContentToVarPacket(0,0x0107,&GPS_longitude);
    aciAddContentToVarPacket(0,0x0108,&GPS_height);
    
	//motor speed
    for(int i = 0; i < 6; i++){
		 aciAddContentToVarPacket(0,(0x0100)+i,&rpm[i]);
    }
    
    //aciAddContentToVarPacket(0,0x0200,&angvel_pitch);
    //aciAddContentToVarPacket(0,0x0201,&angvel_roll);
    //aciAddContentToVarPacket(0,0x0202,&angvel_yaw);
    
    aciAddContentToVarPacket(0,0x0203,&acc_x);
    aciAddContentToVarPacket(0,0x0204,&acc_y);
    aciAddContentToVarPacket(0,0x0205,&acc_z);
    //aciAddContentToVarPacket(0,0x0106,&latitude);
    //aciAddContentToVarPacket(0,0x0107,&longitude);
    //aciAddContentToVarPacket(0,0x0108,&height);
    aciAddContentToVarPacket(0,0x0303,&fusion_latitude);
    aciAddContentToVarPacket(0,0x0304,&fusion_longitude);
    aciAddContentToVarPacket(0,0x0305,&fusion_height);
    aciAddContentToVarPacket(0,0x010B,&GPS_heading);
    aciAddContentToVarPacket(0,0x010C,&position_accuracy);
    aciAddContentToVarPacket(0,0x010D,&height_accuracy);
    aciAddContentToVarPacket(0,0x010F,&GPS_num);
    aciAddContentToVarPacket(0,0x0110,&GPS_status);
    
    
    aciSetVarPacketTransmissionRate(0,10);
    aciVarPacketUpdateTransmissionRates();
    aciSendVariablePacketConfiguration(0);
    printf(" added\n\n\n\n");
    var_getted=1;
    printf ("\n\nPress a key To Start Session\n\n");
    fflush(stdout);
}

bool spinning(){
    bool sp = false;
    for(int i = 0; i < 6 && !sp; i++){
        if(rpm[i]>0){
            sp = true;
            return sp;
        }
    }
    return sp;
}









