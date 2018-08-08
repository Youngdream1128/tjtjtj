#include <stdio.h>
#include <unistd.h>
#include <errno.h>
#include <string.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <stdlib.h>


#define TBOX_UPGRADE_SH       "/usr/bin/monitor.sh"



int main(int argc, char **argv)
{
	int ret;
	int status;
	pid_t pid;
	char buff[2];
	FILE *fp;
	
	while(1)
	{
		memset(buff, 0, sizeof(buff));
		fp = popen("ps -A|grep tbox |grep -v grep |wc -l","r");
		if(fread(buff, sizeof(char), sizeof(buff), fp) > 0)
		{
			ret = atoi(buff);
		}else{
			ret = 0;
		}
		
		//printf("ret:%d\n",ret);
		pclose(fp);
		
		if(ret == 0)
		{	
			printf("No tbox process!\n");
			pid = fork(); 
			if (pid == -1) {
				fprintf(stderr, "fork() error.errno:%d error:%s", errno, strerror(errno));
				break;
			}
			if (pid == 0) {
				ret = execlp(TBOX_UPGRADE_SH, TBOX_UPGRADE_SH, "start", NULL);
				if (ret < 0) {
					fprintf(stderr, "execv ret:%d errno:%d error:%s", ret, errno, strerror(errno));
					continue;
				}
				printf("execlp return :%d\n", ret);
				printf("exit child process");
				exit(0);
			}
			if (pid > 0) {
				pid = wait(&status);
				fprintf(stdout, "Child process id: %d\n", pid);
				printf("Wait child process return\n");
			}
		}else{
			//printf("tbox process exsit!\n");
		}
		
		sleep(1);
    }

	return 0;
}

