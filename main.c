#include <unistd.h>
#include <pthread.h>
#include <stdio.h>

#include "igh.h"
#include "term.h"
int main(int argc, char **argv)
{
	//创建控制命令线程
    pthread_t cmdrecv_thread;
    pthread_create(&cmdrecv_thread, NULL, readCommands,NULL);
    
	//创建ethercat线程
    pthread_t ethercat_thread;
    pthread_create(&ethercat_thread, NULL, ethercatMaster,NULL);
	
	//等待线程结束
	pthread_join(cmdrecv_thread, NULL);
	pthread_join(ethercat_thread, NULL);

	while(1)
	{
		sleep(1);
	}
	return 0;
}