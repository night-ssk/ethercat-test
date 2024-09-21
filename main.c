#include <unistd.h>
#include <pthread.h>
#include <stdio.h>

#include "igh.h"
#include "term.h"
int main(int argc, char **argv)
{
	if (argc < 2) {
        printf("使用方法: %s <命令文件路径>\n", argv[0]);
        return 1;
    }
	//创建控制命令线程
    pthread_t cmdrecv_thread;
    pthread_create(&cmdrecv_thread, NULL, readCommands,(void*)argv[1]);
    
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

