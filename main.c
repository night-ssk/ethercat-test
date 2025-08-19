#include <unistd.h>
#include <pthread.h>
#include <stdio.h>

#include "igh.h"
#include "term.h"
#include "mqtt_comm.h"
int main(int argc, char **argv)
{
	//创建ethercat线程
    pthread_t ethercat_thread;
    pthread_create(&ethercat_thread, NULL, ethercatMaster,NULL);
	pthread_detach(ethercat_thread);
	//初始化mqtt服务
	mqtt_comm_init("192.168.137.1", 1883);
	mqtt_comm_start();
	//初始化term服务
	term_init();
	while(1)
	{
		sleep(1);
	}
	return 0;
}

