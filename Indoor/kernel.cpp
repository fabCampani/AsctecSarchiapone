#include "const.h"
#include "etsubsched.h"
#include "module0.h"
#include "module1.h"
#include "module2.h"
#include "module3.h"
#include "module4.h"


main()
{
	//if (argc != 4)
	//	{
	//	printf("\nUsage: RMTest <number of periodic and sporadic experts> <minimum period in microseconds> <expert duration - cycles >  \n\n");
	//	exit(0);
	//	}
    	
    int nSpoEx 	= 1;
	int nPerEx 	= 4;
	
	long int period1 =100000; //10 hz for the user interface
	long int period2 =10000;  //100 hz for accelerometers
    long int period3 =33000;  //30 hz for camera
    long int period4 = 100000; //5hz for the waypoints manager
	
	int nTotEx 	= nSpoEx+nPerEx+1; 


	ETDispatch *kernel;
    Module0 *ModuleZero;
    Module1 *ModuleOne;
    Module2 *ModuleTwo;
    Module3 *ModuleThree;
    Module4 *ModuleFour;
    
	kernel = new ETDispatch (nTotEx,"127.0.0.1", 1800);

    ModuleZero = new Module0(kernel);
	ModuleOne = new Module1(kernel, period1, 1);  // user interface
	ModuleTwo = new Module2(kernel, period2, 2); // imu
	ModuleThree = new Module3(kernel, period3, 3); // positions
    ModuleFour = new Module4(kernel, period4,4); // waypoints

    printf ("\nExperts Built\n");
	fflush(stdout);

	int *expertId = new int[nTotEx];

	expertId[0] = kernel->AddExpert(ModuleOne);
	expertId[1] = kernel->AddExpert(ModuleTwo);
	expertId[2] = kernel->AddExpert(ModuleThree);
	expertId[3] = kernel->AddExpert(ModuleFour);
    expertId[4] = kernel->AddExpert(ModuleZero);
	
	for (int i =0; i < nTotEx-1; i++)
        	kernel->ActivateExpert(expertId[i]);

	//struct timeval t;
	//t.tv_sec = 10;
	//t.tv_usec = 0;
	//ETMsgBomb *b = new ETMsgBomb(ENDSESSION, t);
	//kernel->AddTimeBomb(b);
	
	kernel->DoYourDuty();
    
	for (int i =nTotEx-2; i >= 0; i--)
        	kernel->DeActivateExpert(expertId[i]);

    delete ModuleZero;
	delete ModuleOne;
	delete ModuleTwo;
	delete ModuleThree;
    delete ModuleFour;


	delete kernel;
	printf("\n\n Session Terminated.\n\n");
	return 0;
}


