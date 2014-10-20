#include "const.h"
#include <stdio.h>
#include <ettimer.h>


class Module1 : public ETExpert
{
	private:
                long int message_sent;
		int SendType;
        int waypnum;
    int initialize3;
        public:
		 Module1 (ETDispatch*, long int per, int sm);
		 ~Module1();
		 void Init();
		 void Close();
		 void DoYourDuty(int wc=0);

};
