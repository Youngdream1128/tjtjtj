#include <stdio.h>
#include <time.h>
#include <string.h>
#include <stdint.h>
#include <pthread.h>
#include "common.h"
#include "mcuUart.h"
#include "TBoxDataPool.h"
#include "LTEModuleAtCtrl.h"
#include "GBT32960.h"
#include "OTAUpgrade.h"
#include "LedControl.h"
#include "AdcVoltageCheck.h"
#include "FTPSClient.h"
#include "DnsResolv.h"
#include "VoiceCall.h"
#include "simcom_common.h"
#include "WiFiControl.h"
#include "Message.h"
#include "DataCall.h"
#include "WDSControl.h"
#include "NASControl.h"
#include "GpioWake.h"
#include "dsi_netctrl.h"
#include "FAWACP.h"
#include "OTAWiFi.h"
#include "IVI_Communication.h"
#include <sys/syslog.h>
#include "encryption_init.h"
#include "libnts_crypt_parse.h"
/*
#include "nts_crypt.h"

#define LIB_CACULATE_PATH  "\data\libnts_crypt.so"


//typedef int (*NTS_CRYPT_KEY)(unsigned char *, int, unsigned char *);

int (*ntsEncryptKey)(unsigned char *root_key, int input_len, unsigned char *encrypted_key);
int (*ntsDecryptKey)(unsigned char *encrypted_key, int input_len, unsigned char *root_key);


void LoadntsCrypt()
{
	printf(">>>>>>>>>>>>>>>>>>>>>>>>>>LoadntsCrypt>>>>>>>>\n");

	void * libm_handle = NULL;
	libm_handle = dlopen(LIB_CACULATE_PATH, RTLD_LAZY);
	if (!libm_handle)
	{
		//如果返回 NULL 句柄,通过dlerror方法可以取得无法访问对象的原因
		printf("Open Error:%s.\n",dlerror());
		return 0;
	}

	// 使用 dlsym 函数，尝试解析新打开的对象文件中的符号。您将会得到一个有效的指向该符号的指针，或者是得到一个 NULL 并返回一个错误
	//NTS_CRYPT_KEY ntsEncryptKey = dlsym(libm_handle,"nts_encrypt_key");//call dlsym function
	ntsEncryptKey = dlsym(libm_handle,"nts_encrypt_key");//call dlsym function
	ntsDecryptKey = dlsym(libm_handle,"nts_decrypt_key");//call dlsym function

	char * errorInfo = dlerror();// 调用dlerror方法，返回错误信息的同时，内存中的错误信息被清空
	if (errorInfo != NULL)
		{
	    printf("Dlsym Error:%s.\n",errorInfo);
	    return 0;
	}

	uint8_t pRootKey[] = {0x7b, 0xde, 0x18, 0x70, 0x1a, 0xc2, 0x52, 0xb9, 0xa9, 0xb6, 0x8a, 0x08, 0x8c, 0x9a, 0x8f, 0x82};
	uint8_t encrypted_key[] = {0x7b, 0xde, 0x18, 0x70, 0x1a, 0xc2, 0x52, 0xb9, 0xa9, 0xb6, 0x8a, 0x08, 0x8c, 0x9a, 0x8f, 0x82};;
	int input_len = 16;
	int result = 0;
	
	result = ntsEncryptKey(pRootKey, input_len, encrypted_key);
	printf("ntsEncryptKey result:%s.\n",result);

	/*unsigned char * root_key;
	uint8_t *root_key = (uint8_t *)malloc(256);
	if(root_key == NULL)
	{
		printf("malloc root_key error!");
	}
	result = ntsDecryptKey(encrypted_key, input_len, pRootKey);
	if(result < 0){
		printf("decode root_key error \n");
		free(root_key);
		root_key = NULL;
		return;
		}
	printf("ntsDecryptKey result:%s.\n",result);
	free(root_key);
	root_key = NULL;
	*/
	dlclose(libm_handle);
}

*/
#if 1
//天津一汽APN
//生产测试用APN
#define APN1  "zwzgyq02.clfu.njm2mapn"
#define APN2  "UNIM2M.NJM2MAPN"
//量产APN
//#define APN1  "zwzgyq02.clfu.njm2mapn"
//#define APN2  "zwzgyq03.clfu.njm2mapn"
#endif


TBoxDataPool *dataPool = NULL;
LTEModuleAtCtrl *LteAtCtrl = NULL;
GBT32960 *p_GBT32960 = NULL;
IVI_Communication iviCommunication;


char target_ip[16] ={0};


/*****************************************************************************
* Function Name : apn_init
* Description   : apn 初始化,及设置apn
* Input			: None
* Output        : None
* Return        : 0:success, -1:failed
* Auther        : ygg
* Date          : 2018.03.19
*****************************************************************************/
int apn_init()
{
	int ret;
	int apn_index = 4;
	char apn[30] = {0};
	char username[64] = {0};
	char password[64] = {0};
	int pdp_type;

	//apn 初始化
	if(wds_qmi_init() == 0)
	   DEBUGLOG("wds_qmi_init success!\n");

	//profile_index: 4-->Private network; 6-->Public Network
	ret = wds_GetAPNInfo(apn_index, &pdp_type, apn, username, password); 
	if(ret == FALSE)
	{
		printf("wds_GetAPNInfo Fail\n");
	}
	else
	{
		printf(">>>>>> apn[%d]=%s, pdp_type = %d, username=%s, password=%s\n", apn_index, apn, pdp_type, username, password);
	}

	if(strcmp(apn, APN1) != 0)
		wds_SetAPNInfo(4, 0, APN1, NULL, NULL);

	apn_index = 6;
	memset(apn, 0, sizeof(apn));
	memset(username, 0, sizeof(username));
	memset(password, 0, sizeof(password));
	ret = wds_GetAPNInfo(apn_index, &pdp_type, apn, username, password); 
	if(ret == FALSE)
	{
		printf("wds_GetAPNInfo Fail\n");
	}
	else
	{
		printf(">>>>>> apn[%d]=%s, pdp_type = %d, username=%s, password=%s\n", apn_index, apn, pdp_type, username, password);
	}

    if(APN2 != NULL)
    {
        if(strcmp(apn, APN2) != 0)
        wds_SetAPNInfo(6, 0, APN2, NULL, NULL);
    }
    else
    {
        wds_SetAPNInfo(6, 0, NULL, NULL, NULL);
    }
    
	return 0;
}

/*****************************************************************************
* Function Name : mcuInitThread
* Description   : mcu 线程初始化
* Input			: void *args 
* Output        : None
* Return        : NULL
* Auther        : ygg
* Date          : 2018.01.18
*****************************************************************************/
void *mcuInitThread(void * args) 
{
	pthread_detach(pthread_self());
	mcuUart *p_mcuUart = new mcuUart();
	
	return NULL;
}

/*****************************************************************************
* Function Name : dataCallDailCheck
* Description   : 私有网络断开重复拨号
* Input			: void *args 
* Output        : None
* Return        : NULL
* Auther        : ygg
* Date          : 2018.01.18
*****************************************************************************/
void *dataCallDailCheck(void * args) 
{
    datacall_info_type datacall_info;
	pthread_detach(pthread_self());
	while(1)
	{
	    get_datacall_info(&datacall_info);
		if(datacall_info.status == DATACALL_DISCONNECTED)
		{
			if(tboxInfo.networkStatus.isLteNetworkAvailable != NETWORK_NULL)
				tboxInfo.networkStatus.isLteNetworkAvailable = NETWORK_NULL;
			dataCallDail();
			sleep(5);
		}
		else
		{
			if(tboxInfo.networkStatus.isLteNetworkAvailable != NETWORK_LTE)
				tboxInfo.networkStatus.isLteNetworkAvailable = NETWORK_LTE;
			sleep(5);
			#if 0
			int ret;
		    char ip[16]={0};
			ret = query_ip_from_dns("www.baidu.com",
    			                   datacall_info.pri_dns_str,
    			                   datacall_info.sec_dns_str,
    			                   ip);
    	    printf("baidu ip=%s len=%d\n",ip, strlen(ip));
    	    memset(ip,0,sizeof(ip));
			ret = query_ip_from_dns("www.yahoo.com",
    			                   datacall_info.pri_dns_str,
    			                   datacall_info.sec_dns_str,
    			                   ip);
    	    printf("yahoo ip=%s len=%d\n",ip, strlen(ip)); 
			#endif  
	    }
	}
	
	return NULL;
}

/*****************************************************************************
* Function Name : gpioWakeThread
* Description   : gpio 唤醒脚检测
* Input			: void *args 
* Output        : None
* Return        : 0:success
* Auther        : ygg
* Date          : 2018.01.18
*****************************************************************************/
void *gpioWakeThread(void *args)
{
	pthread_detach(pthread_self());
	gipo_wakeup_init();
	
	return (void *)0;
}

/*****************************************************************************
* Function Name : GBT32960Thread
* Description   : gb32960线程初始化
* Input			: void *args 
* Output        : None
* Return        : 0:success
* Auther        : ygg
* Date          : 2018.01.18
*****************************************************************************/
void *GBT32960Thread(void *args)
{
	pthread_detach(pthread_self());
    p_GBT32960 = new GBT32960();
	return (void *)0;
}

void *FAWACPThread(void *args)
{
  pthread_detach(pthread_self());
  CFAWACP  cFAWACP;

  return (void *)0;
}

void *IVIThread(void *args)
{
	pthread_detach(pthread_self());
	while(1)
	{
		iviCommunication.IVI_Communication_Init();
		sleep(2);
	}
	return (void *)0;
}


void process_simcom_ind_message(simcom_event_e event,void *cb_usr_data)
{
    int i;

    switch(event)
    {
        case SIMCOM_EVENT_VOICE_CALL_IND:
            {
            	//唤醒系统
				if(tboxInfo.operateionStatus.isGoToSleep == 0)
				{
					printf("0000000000 voice call\n");
					tboxInfo.operateionStatus.isGoToSleep = 1;
					tboxInfo.operateionStatus.wakeupSource = 1;
					lowPowerMode(1, -1);
				modem_ri_notify_mcu();
				}

				iviCommunication.TBOX_Voicall_State(cb_usr_data);
				#if 0
                call_info_type call_list[QMI_VOICE_CALL_INFO_MAX_V02];
                memcpy((void *)call_list, cb_usr_data, sizeof(call_list));
				
                for(i = 0; i < QMI_VOICE_CALL_INFO_MAX_V02; i++)
                {
					if(call_list[i].call_id != 0)
                    {
                        printf(">>>>>>voice callback:\n");
                        printf("index[%d], call_id=%d, state=%d, direction = %d, number=%s\n",
                                          i,
                                          call_list[i].call_id,
                                          call_list[i].call_state,
                                          call_list[i].direction,
                                          strlen(call_list[i].phone_number) >= 1?call_list[i].phone_number:"unkonw"
                                          );
						
                        printf("<<<<<<<<<<< \n");

						if(call_list[i].call_state == CALL_STATE_END_V02)
							tboxInfo.operateionStatus.phoneType = 0;
                    }
                }  
                #endif
            }
            break;

        case SIMCOM_EVENT_SMS_PP_IND:
            {
                //唤醒系统
				if(tboxInfo.operateionStatus.isGoToSleep == 0)
				{
					printf("11111111111111 sms \n");
					tboxInfo.operateionStatus.isGoToSleep = 1;
					tboxInfo.operateionStatus.wakeupSource = 2;
					lowPowerMode(1, -1);
					modem_ri_notify_mcu();
				}
				
                sms_info_type sms_info;
                memcpy((void *)&sms_info, cb_usr_data, sizeof(sms_info));
                
                printf("\n-----------receive message --------------------------\n");
                printf("address=%s\n",sms_info.source_address);
                for(i = 0; i < strlen(sms_info.message_content); i++)
                {
                    printf("0x%02X ", sms_info.message_content[i]);
                }
                printf("\n");
            }
            break;

        case SIMCOM_EVENT_NETWORK_IND:
            {
                network_info_type network_info;
                memcpy((void *)&network_info, cb_usr_data, sizeof(network_info));
                //printf("\n---------network info---------------------------\n");
                /*printf("network_info: register=%d, cs=%d, ps=%d,radio_if=%d\n",
                        network_info.registration_state,
                        network_info.cs_attach_state,
                        network_info.ps_attach_state,
                        network_info.radio_if_type);*/
				if(network_info.registration_state == NAS_REGISTERED_V01)
				{
					if(tboxInfo.networkStatus.networkRegSts != 1)
						tboxInfo.networkStatus.networkRegSts = 1;

					if(lteLedStatus != 2)
						lte_led_blink(500,500);
				}
				if(network_info.registration_state != NAS_REGISTERED_V01)
				{
					if(tboxInfo.networkStatus.networkRegSts == 1)
						tboxInfo.networkStatus.networkRegSts = 0;

					if(lteLedStatus != 1)
						lte_led_on();
				}
            }
            break;
         case SIMCOM_EVENT_DATACALL_IND:
            {
            	int ret;
            	char target_ip_new[16] = {0};
                datacall_info_type datacall_info;
                get_datacall_info(&datacall_info);
                                   
				if(datacall_info.status == DATACALL_CONNECTED)
				{
				    int use_dns = 1;
                    printf("datacall_ind1: if_name=%s,ip=%s,mask=%d\n", 
                                          datacall_info.if_name,
                                          datacall_info.ip_str,
                                          datacall_info.mask);
                    printf("datacall_ind2: dns1=%s,dns2=%s,gw=%s\n", 
                                          datacall_info.pri_dns_str,
                                          datacall_info.sec_dns_str,
                                          datacall_info.gw_str); 
                    if(use_dns)
                    {
                    	//天津一汽:"znwl-uat-cartj.faw.cn"
    					ret = query_ip_from_dns("znwl-uat-cartj.faw.cn", datacall_info.pri_dns_str ,datacall_info.pri_dns_str , target_ip_new);
    					if(ret != 0)
    					{
    						printf("query ip fail\n");
    						break;
    					}

    				  	printf("target_ip:%s\n",target_ip_new);

    				  	set_host_route(target_ip, target_ip_new, datacall_info.if_name);
    				  	strncpy(target_ip, target_ip_new, sizeof(target_ip_new));
				  	}
				  	else
				  	{
				  	    set_host_route(target_ip, target_ip, datacall_info.if_name);
				  	}
				}
            }
            break;
         default:
            break;
    }
}

extern int uart_debug();

void do_enter_recovery_reset(void)
{
    sleep(2);
    syscall(SYS_reboot, LINUX_REBOOT_MAGIC1, LINUX_REBOOT_MAGIC2, 
        LINUX_REBOOT_CMD_RESTART2, "recovery");
}

void do_enter_backup_reset(void)
{
    sleep(2);
    system("reboot");
}

int enter_recovery_mode_delay(void)
{
	pthread_t id;
	int ret;
	ret = pthread_create(&id, NULL, (void *)do_enter_recovery_reset, NULL);
	return 0;
}

/* call for user, update module*/
void system_update()
{
	printf("system_update in\n");
	//set flag to send DM session(non fota update)
	FILE  *fp = NULL;
	system("mkdir -p /data/dme");
	system("touch /data/dme/start_DM_session");
	fp = fopen("/data/dme/start_DM_session", "w+");
	if(fp == NULL)
	{
		return -1;
	}	
	else
	{
		fclose(fp);
		system("sync");
		/*modify to not delete temp files
		system("rm /data/dme/DMS.tre");
		system("rm /data/dme/DevDetail.tre");
		system("rm /data/dme/DevInfo.tre");
		system("rm /data/dme/aclData.acl");
		system("rm /data/dme/eventlist.cfg");
		system("rm /data/dme/init_active_date_file");
		system("rm /data/dme/rs_log.txt");
		system("rm /data/dme/session.log");*/	
	}

	system("echo off > /sys/power/autosleep"); 
	(void)enter_recovery_mode_delay();
	return 0;
}

	
	
	

int main(int argc,char* argv[]) 
{
	int ret;
	uint32_t count = 0;
	char sysVersion[128] = {0};
	pthread_t mcuInitThreadId;
	pthread_t GBT32960ThreadId;
	pthread_t gpioWakeId;
	pthread_t FAWACPThreadId;
	pthread_t IVIThreadId;

	pthread_attr_t IVIAttr;
	struct sched_param sched;

	ret = pthread_attr_init(&IVIAttr);
	if(ret != 0)
		printf("pthread_attr_init error!\n");
	
	sched.sched_priority = 99;
    pthread_attr_setschedpolicy(&IVIAttr, SCHED_FIFO); //SCHED_RR;
    pthread_attr_setschedparam(&IVIAttr, &sched);
    pthread_attr_setinheritsched(&IVIAttr, PTHREAD_EXPLICIT_SCHED);

	//uart_debug();
	openlog("SIMCOM_DEMO", LOG_PID, LOG_USER);
	
	getSoftwareVerion(sysVersion, sizeof(sysVersion));
	DEBUGLOG("System version:%s", sysVersion);

	//encryption_init();//加密IC

	DEBUGLOG("1111111111111111111111111111");
	encrypt_key();
	DEBUGLOG("2222222222222222222222222");

	lte_led_on();

	dataPool  = new TBoxDataPool();
	LteAtCtrl = new LTEModuleAtCtrl();
	LteAtCtrl->atCtrlInit();

	if(apn_init() == 0)
	   DEBUGLOG("apn_init success!");

	if(dataCall_init() == 0)
		DEBUGLOG("dataCall_init success!");
	
	if(voiceCall_init() == 0)
	   DEBUGLOG("voiceCall_init success!");
	 
	if(message_init() == 0)
	   DEBUGLOG("message_init success!");

	if(nas_init() == 0)
		DEBUGLOG("nas_init success!");

	if(wifi_startState_check() == -1)
	{
		DEBUGLOG("wifi init incompletely!");
	}
    
//修改WIFI默认的用户名和密码。老版本需要升级system
	check_wifi_ssid_pwd_default_value();



#if 1
	ret = pthread_create(&mcuInitThreadId, NULL, mcuInitThread, NULL);
	if(0 != ret) 
	{
		printf("can't create thread: %s\n",strerror(ret)); 
		exit(-1);
	}
#endif

#if 0
	ret = pthread_create(&dataCallDailId, NULL, dataCallDailCheck, NULL);
	if(0 != ret)
	{
		printf("can't create thread dataCallDailCheck : %s\n",strerror(ret));
		exit(-1);
	}
#endif

#if 1
	ret = pthread_create(&gpioWakeId, NULL, gpioWakeThread, NULL);
	if(0 != ret)
	{
		printf("can't create thread gpioWakeThread : %s\n",strerror(ret));
		exit(-1);
	}
#endif

#if 0
	ret = pthread_create(&GBT32960ThreadId, NULL, GBT32960Thread, NULL);
	if(0 != ret)
	{
		printf("can't create thread GBT32960Thread : %s\n",strerror(ret));
		exit(-1);
	}
#endif

#if 1
	ret = pthread_create(&FAWACPThreadId, NULL, FAWACPThread, NULL);
	if(0 != ret)
	{
		printf("can't create FAWACPThread:%s\n",strerror(ret));
		exit(-1);
	}
#endif

#if 1
	ret = pthread_create(&IVIThreadId, &IVIAttr, IVIThread, NULL);
	if(0 != ret) 
	{
		printf("can't create thread: %s\n",strerror(ret)); 
		exit(-1);
	}
#endif

	while(1)
	{
		count++;
		if(count%2 == 0)
		{
			nas_get_SignalStrength((int *)&tboxInfo.networkStatus.signalStrength, &ret);
			//printf("tboxInfo.networkStatus.signalStrength = %d\n", tboxInfo.networkStatus.signalStrength);

			nas_serving_system_type_v01 nas_status;
			nas_get_NetworkType(&nas_status);
			if(nas_status.registration_state == NAS_REGISTERED_V01)
			{
				if(tboxInfo.networkStatus.networkRegSts != 1)
					tboxInfo.networkStatus.networkRegSts = 1;

				if(lteLedStatus != 2)
					lte_led_blink(500,500);
			}
			if(nas_status.registration_state != NAS_REGISTERED_V01)
			{
				if(tboxInfo.networkStatus.networkRegSts == 1)
					tboxInfo.networkStatus.networkRegSts = 0;

				if(lteLedStatus != 1)
					lte_led_on();
			}

			if(tboxInfo.operateionStatus.wifiStartStatus == 0)
				wifi_connected_dataExchange();
		}

		if(count%5 == 0)
		{
			ret = wifi_startState_check();
			if(ret == 0)
			{
				if(tboxInfo.operateionStatus.wifiStartStatus != 0)
					tboxInfo.operateionStatus.wifiStartStatus = 0;
			}
			else
			{
				if(tboxInfo.operateionStatus.wifiStartStatus != -1)
					tboxInfo.operateionStatus.wifiStartStatus = -1;
				wifi_led_off(1);
			}

			datacall_info_type datacall_info;
			get_datacall_info(&datacall_info);
			if(datacall_info.status == DATACALL_DISCONNECTED)
			{
				if(tboxInfo.networkStatus.isLteNetworkAvailable != NETWORK_NULL)
					tboxInfo.networkStatus.isLteNetworkAvailable = NETWORK_NULL;
				dataCallDail();
			}else{
				if(tboxInfo.networkStatus.isLteNetworkAvailable != NETWORK_LTE)
					tboxInfo.networkStatus.isLteNetworkAvailable = NETWORK_LTE;
			}
		}
		sleep(1);
	}
	
	return 0;
}


