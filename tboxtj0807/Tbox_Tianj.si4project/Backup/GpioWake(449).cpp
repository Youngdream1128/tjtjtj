#include <stdint.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <getopt.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/types.h>
#include <sys/poll.h>
#include <sys/stat.h>
#include <string.h>
#include <errno.h>
#include <termios.h>
#include <linux/input.h>
#include <linux/ioctl.h>
#include <pthread.h>
#include "GpioWake.h"


#define MODEM_WAKEUP_MCU_GPIO  77 //sim7600.pin87
#define GPIO77_LOCKNAME "GPIO77_Wakeup_modem"
#define MCU_WAKEUP_MODEM_GPIO  74 //sim7600.pin72
#define GPIO74_LOCKNAME "GPIO74_Wakeup_modem"
#define GPIO_SEELP_MODEM_MS    100

#define MAX_BUF 128


#define PIN72_ISR_LOCK  system_power_lock(GPIO74_LOCKNAME)
#define PIN72_ISR_UNLOCK system_power_unlock(GPIO74_LOCKNAME)

#define FC_WAKE  system_power_lock("FC_LOCK")
#define FC_SLEEP system_power_unlock("FC_LOCK")

#define PIN87_LOCK  system_power_lock(GPIO77_LOCKNAME)
#define PIN87_UNLOCK system_power_unlock(GPIO77_LOCKNAME)

static int openOrCloseWifi = -1;


int lowPowerMode(int isSleep, int isCloseWifi);


void system_power_lock(const char *lock_id)
{
	int fd;

	fd = open("/sys/power/wake_lock", O_WRONLY);
	if (fd < 0) {
		printf("wake_lock,error %d\n",fd);
		return;
	}

	write(fd, lock_id, strlen(lock_id));

	close(fd);
}
void system_power_unlock(const char *lock_id)
{
	int fd;
	fd = open("/sys/power/wake_unlock", O_WRONLY);
	if (fd < 0) {
		printf("wake_unlock,error %d\n",fd);
		return;
	}

	write(fd, lock_id, strlen(lock_id));

	close(fd);
}

int gpio_file_create(int gpio)
{
	int sfd = -1;
	char checkstr[50] = {0};
	char configstr[10] = {0};
	int reto,len;

	sprintf(checkstr,"/sys/class/gpio/gpio%d/value",gpio);
	if(0 == access(checkstr, F_OK)){
		return 0;
	}

	sfd = open("/sys/class/gpio/export",O_WRONLY);
	if(sfd < 0){
		printf("%s:%d,open file error,%d\n",__FUNCTION__,__LINE__,sfd);
		return sfd;
	}

	len = sprintf(configstr,"%d",gpio);
	reto = write(sfd,configstr,len);

	if(reto != len){
		printf("create gpio(%d) files:%d,%d,%d\n",gpio,__LINE__,len,reto);
		return reto;
	}
	usleep(1000);
	reto = access(checkstr, F_OK);
	if(0 > reto){
		printf("%s:%d,gpio file(%s)not exist\n",__FUNCTION__,__LINE__,checkstr);
	}

	close(sfd);
	return reto;
}
int gpio_set_dir(unsigned int gpio, unsigned int out_flag)
{
	int fd;
	char buf[MAX_BUF];
 
	snprintf(buf, sizeof(buf), "/sys/class/gpio/gpio%d/direction", gpio);
 
	fd = open(buf, O_WRONLY);
	if (fd < 0) {
		printf("gpio(%d)/direction\n",gpio);
		return fd;
	}
 
	if (out_flag)
		write(fd, "out", 4);
	else
		write(fd, "in", 3);
 
	close(fd);
	return 0;
}
int gpio_set_value(unsigned int gpio, unsigned int value)
{
	int fd;
	char buf[MAX_BUF];
 
	snprintf(buf, sizeof(buf), "/sys/class/gpio/gpio%d/value", gpio);
 
	fd = open(buf, O_WRONLY);
	if (fd < 0) {
		perror("gpio/set-value\n");
		return fd;
	}
 
	if (value)
		write(fd, "1", 2);
	else
		write(fd, "0", 2);
 
	close(fd);
	return 0;
}
static int gpio_set_isr(unsigned gpio)
{
	char fvalue_path[MAX_BUF];
	int fd = -1,ret = -1;

	sprintf(fvalue_path,"/sys/class/gpio/gpio%d/edge",gpio);
	fd = open(fvalue_path,O_RDWR);
	if(fd < 0){
		printf("gpio_set_isr write %s error %d\n",fvalue_path,ret);
		return fd;
	}
	
	if((ret = write(fd,"both",5)) < 1){
		close(fd);
		printf("gpio_set_isr write %s error,%d\n",fvalue_path,ret);
		return ret;
	}
	close(fd);

	return (ret < 0)?ret:0;
}
#define MODEM_WAKEUP_MCU_TIME 100
int modem_ri_notify_mcu(void)
{
	PIN87_LOCK;
	gpio_set_value(MODEM_WAKEUP_MCU_GPIO,1);
	poll(0,0,MODEM_WAKEUP_MCU_TIME);
	gpio_set_value(MODEM_WAKEUP_MCU_GPIO,0);
	PIN87_UNLOCK;
}
int gpio_can_wakeup(unsigned int gpio, unsigned int value)
{
	int fd;
	char buf[MAX_BUF];

	snprintf(buf, sizeof(buf), "/sys/class/gpio/gpio%d/can_wakeup", gpio);

	fd = open(buf, O_WRONLY);
	if (fd < 0) {
		perror("gpio/set-value");
		return fd;
	}

	if (value)
		write(fd, "1", 2);
	else
		write(fd, "0", 2);

	close(fd);
	return 0;
}
int uart_wakeup(unsigned char bwakeup)
{
	int ufd = open(UART_SLEEP_CTL, O_WRONLY);
	if (ufd < 0) {
		printf(" %s error %d\n",UART_SLEEP_CTL, ufd);
		return -1;
	}
	if(bwakeup){
		write(ufd, "on", 2);
	}else{
		write(ufd, "auto", 4);
	}
	close(ufd);
	return 0;
}

int gpio_init()
{
    int ret;
    ret = gpio_file_create(MODEM_WAKEUP_MCU_GPIO);
    ret += gpio_set_dir(MODEM_WAKEUP_MCU_GPIO, 1);
    ret += gpio_set_value(MODEM_WAKEUP_MCU_GPIO, 0);

    ret += gpio_file_create(MCU_WAKEUP_MODEM_GPIO);
    ret += gpio_set_dir(MCU_WAKEUP_MODEM_GPIO, 0);
    ret += gpio_set_isr(MCU_WAKEUP_MODEM_GPIO);
    ret += gpio_can_wakeup(MCU_WAKEUP_MODEM_GPIO, 1);
    return ret;
}

int lowPowerMode(int isSleep, int isCloseWifi)
{
	// 0:go to sleep, unlock system
	if(isSleep == 0)
	{
		lte_led_off(0);
		wifi_led_off(0);
		uart_wakeup(0);
		//关闭wifi
		DEBUGLOG("############### isSleep:%d, isCloseWifi:%d, openOrCloseWifi:%d tboxInfo.operateionStatus.wifiStartStatus %d\n",isSleep, isCloseWifi, openOrCloseWifi, tboxInfo.operateionStatus.wifiStartStatus);
		if(((isCloseWifi == 0) && (openOrCloseWifi == 1)) || ((isCloseWifi == 0) && (tboxInfo.operateionStatus.wifiStartStatus == 0)))
		{
			DEBUGLOG("############### 1111 \n");
			//close wifi success
			if(wifi_OpenOrClose(0) == 0){
				openOrCloseWifi = 0;
				tboxInfo.operateionStatus.wifiStartStatus = -1;
				DEBUGLOG("############### 2222 \n");
			}
		}
		if(tboxInfo.operateionStatus.isGoToSleep != 0)
			tboxInfo.operateionStatus.isGoToSleep = 0;
		
		FC_SLEEP;
	}
	else    // wake up sys, lock system
	{
		FC_WAKE;
		DEBUGLOG("=========== isSleep:%d, isCloseWifi:%d, openOrCloseWifi:%d tboxInfo.operateionStatus.wifiStartStatus %d\n",isSleep, isCloseWifi, openOrCloseWifi, tboxInfo.operateionStatus.wifiStartStatus);
		if(tboxInfo.operateionStatus.isGoToSleep != 1)
			tboxInfo.operateionStatus.isGoToSleep = 1;
		if(lteLedStatus == 1)
		{
			lte_led_on();
		}
		else if(lteLedStatus == 0)
		{
			lte_led_off(0);
		}
		else
		{
			lte_led_blink(500, 500);
		}

		uint8_t wifi_RemindStatus = -1;
		dataPool->getPara(WIFI_REMINDSTATUS_TD, &wifi_RemindStatus, sizeof(wifi_RemindStatus));
		
			
		if(wifi_RemindStatus == 0){
			wifi_led_off(1);
		}else{
			if(wifiLedStatus == 1)
			{
				wifi_led_on();
			}
			else if(wifiLedStatus == 0)
			{
				wifi_led_off(0);
			}
			else if(wifiLedStatus == 2)
			{
				wifi_led_blink(500, 500);
			}
		}
		uart_wakeup(1);

		//打开wifi
		if( wifi_RemindStatus == 1 )
		{
			//open wifi success
			wifi_OpenOrClose(1);
			openOrCloseWifi = 1;
			tboxInfo.operateionStatus.wifiStartStatus = 0;
		}
		else{	//关闭WIFI
			wifi_OpenOrClose(0);
            wifi_led_off(0);
			openOrCloseWifi = -1;
			tboxInfo.operateionStatus.wifiStartStatus = -1;
		}
	}

	return 0;
}

int gipo_wakeup_init()
{
	int ret = 0;
	int fd;
	char fvalue_path[MAX_BUF]={0};
	struct pollfd read_pollfd;
	int ch_time = -1;

	if(gpio_init() != 0){
		printf("gpio_init error\n");
		return -1;
	}

	sprintf(fvalue_path,"/sys/class/gpio/gpio%d/value",MCU_WAKEUP_MODEM_GPIO);
	fd = open(fvalue_path,O_RDONLY|O_NONBLOCK);
	if(fd< 0){
		printf("open %s,fd error %d\n",fvalue_path,fd);
		return -1;
	}

	memset(&read_pollfd,0,sizeof(read_pollfd));
	read_pollfd.fd=fd;
	read_pollfd.events = (POLLERR |POLLPRI);

	while(1){
		printf("polling,time = %d\n",ch_time);
		ret = poll(&read_pollfd, 1, ch_time);
		if(ret < 0){
			break;
		}
		if((ret == 0) && (ch_time == GPIO_SEELP_MODEM_MS)){
			char rbuff[3]={0};
			int lvalue = 0;
			read_pollfd.revents = 0;
			ret = lseek(fd,(off_t)0, SEEK_SET);
			ret += read(fd,rbuff,3);
			printf("Timeout GPIO is %s\n", rbuff);
			lvalue = atoi(rbuff);
			if(!lvalue){
				PIN72_ISR_UNLOCK;
				tboxInfo.operateionStatus.wakeupSource = -1;
				lowPowerMode(0, 0);
				printf("goto sleep\n");
			}else{
				tboxInfo.operateionStatus.wakeupSource = 0;
				lowPowerMode(1, 1);
				printf("wakeup on\n");
			}
			ch_time = -1;
		}
		if((read_pollfd.revents & read_pollfd.events) == read_pollfd.events){
			char rbuff[3]={0};
			int lvalue = 0;
			read_pollfd.revents = 0;
			PIN72_ISR_LOCK;
			ret = lseek(fd,(off_t)0, SEEK_SET);
			ret += read(fd,rbuff,3);
			printf("GPIO is %s\n", rbuff);
			lvalue = atoi(rbuff);
			if(!lvalue){
				ch_time = GPIO_SEELP_MODEM_MS;//wait 100ms
			}else{
				tboxInfo.operateionStatus.wakeupSource = 0;
				lowPowerMode(1, 1);
				ch_time = -1;
			}
		}
	}
	return ret;
}





#if 0

int main(int argc, char *argv[])
{
	int ret = 0;
	int fd;
	char fvalue_path[MAX_BUF]={0};
	struct pollfd read_pollfd;
	int ch_time = -1;

	if(gpio_init() != 0){
		printf("gpio_init error\n");
		return -1;
	}

	sprintf(fvalue_path,"/sys/class/gpio/gpio%d/value",MCU_WAKEUP_MODEM_GPIO);
	fd = open(fvalue_path,O_RDONLY|O_NONBLOCK);
	if(fd< 0){
		printf("open %s,fd error %d\n",fvalue_path,fd);
		return -1;
	}

	memset(&read_pollfd,0,sizeof(read_pollfd));
	read_pollfd.fd=fd;
	read_pollfd.events = (POLLERR |POLLPRI);

	while(1){
		printf("polling,time = %d\n",ch_time);
		ret = poll(&read_pollfd, 1, ch_time);
		if(ret < 0){
			break;
		}
		if((ret == 0) && (ch_time == GPIO_SEELP_MODEM_MS)){
			char rbuff[3]={0};
			int lvalue = 0;
			read_pollfd.revents = 0;
			ret = lseek(fd,(off_t)0, SEEK_SET);
			ret += read(fd,rbuff,3);
			printf("Timeout GPIO is %s\n", rbuff);
			lvalue = atoi(rbuff);
			if(!lvalue){
				PIN72_ISR_UNLOCK;
				printf("goto sleep\n");
			}else{
				printf("wakeup on\n");
			}
			ch_time = -1;
		}
		if((read_pollfd.revents & read_pollfd.events) == read_pollfd.events){
			char rbuff[3]={0};
			int lvalue = 0;
			read_pollfd.revents = 0;
			PIN72_ISR_LOCK;
			ret = lseek(fd,(off_t)0, SEEK_SET);
			ret += read(fd,rbuff,3);
			printf("GPIO is %s\n", rbuff);
			lvalue = atoi(rbuff);
			if(!lvalue){
				ch_time = GPIO_SEELP_MODEM_MS;//wait 100ms
			}else{
				ch_time = -1;
			}
		}
	}
	return ret;
}
#endif

