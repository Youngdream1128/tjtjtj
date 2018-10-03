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

#include "FactoryPattern_Communication.h"

#if 1
//天津一汽APN
//生产测试用APN
//#define APN1  "zwzgyq02.clfu.njm2mapn"
//#define APN2  "UNIM2M.NJM2MAPN"
//量产APN
#define APN1  "zwzgyq02.clfu.njm2mapn"
#define APN2  "zwzgyq03.clfu.njm2mapn"
#endif


TBoxDataPool *dataPool = NULL;
LTEModuleAtCtrl *LteAtCtrl = NULL;
GBT32960 *p_GBT32960 = NULL;
IVI_Communication iviCommunication;
FactoryPattern_Communication  factorypattern;

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

void *factoryThread(void *args)
{
	pthread_detach(pthread_self());
	while(1)
	{
		factorypattern.FactoryPattern_Communication_Init();
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
					lowPowerMode(1, 1);
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
					lowPowerMode(1, 1);
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

//组件库加密
int encrypttest(unsigned char *pRootKey, int input_len, int num)
{
	int result = 0;
	printf("RootKey_%d is:\n", num);
    for (int i = 0; i < input_len; i++) {
        printf("%02x",pRootKey[i]);
    }
	printf("\n");
	
	uint8_t *encryptedKey;
	encryptedKey = (uint8_t *)malloc(256);
	if(encryptedKey == NULL)
		printf("malloc encryptedKey error!");
	else
	{	
		result = encrypt_key(pRootKey, input_len, encryptedKey);
		printf("ntsEncryptKey result:%d.\n",result);

		free(encryptedKey);
		encryptedKey = NULL;
	}

	return result;
}
//组件库解密
int decrypttest(unsigned char *pEncryptedKey, int input_len, int num)
{
	int result = 0;
	printf("encryptedKey_%d is:\n", num);
	for (int i = 0; i < input_len; i++) {
		printf("%02x",pEncryptedKey[i]);
	}
	printf("\n");	

	uint8_t *decryptkey;
	decryptkey = (uint8_t *)malloc(256);
	if(decryptkey == NULL)
		printf("malloc decryptkey error!");
	else
	{
		printf("malloc decrypt_key success!\n");
		result = decrypt_key(pEncryptedKey, input_len, decryptkey);
		printf("ntsDecryptKey result:%d.\n",result);

		free(decryptkey);
		decryptkey = NULL;		
	}
	return result;
}


int main(int argc,char* argv[]) 
{
	int ret;
	uint32_t count = 0;
	char sysVersion[128] = {0};
	pthread_t mcuInitThreadId;
	//pthread_t GBT32960ThreadId;
	pthread_t gpioWakeId;
	pthread_t FAWACPThreadId;
	pthread_t IVIThreadId;

	pthread_t factoryThreadId;

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
	
/*	//加密IC
	int nFlagCfg_Encryption = -1;//-1.失败,0.已配置未锁,1.已锁,2.配置成功
	int retEncryInit = encryption_init(nFlagCfg_Encryption);//加密IC
*/
//加密
/*	int input_len = 16;
	int result = 0;
	printf(">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>    Encrypted_key start:    >>>>>>>>>>>>>>>>>>>\n");
	uint8_t pRootKey_1[16] = {0xC4, 0xF3, 0x05, 0x1F, 0xF5, 0x59, 0x48, 0xC9, 0x79, 0xED, 0xB5, 0x62, 0xA7, 0x44, 0xAA, 0x1C};
	uint8_t pRootKey_2[16] = {0x93, 0xA6, 0x8D, 0x01, 0xF6, 0x8E, 0xB2, 0x7F, 0xE4, 0xE2, 0x13, 0x24, 0x26, 0xCF, 0x82, 0x7B};
	uint8_t pRootKey_3[16] = {0xF0, 0x10, 0x47, 0x78, 0x84, 0xFE, 0xEA, 0xAC, 0x23, 0x65, 0xA2, 0x82, 0x96, 0xF2, 0x5B, 0x2E};
	uint8_t pRootKey_4[16] = {0x77, 0x58, 0xEE, 0x4D, 0xF3, 0xDB, 0xE6, 0x8F, 0x46, 0x28, 0xF3, 0x41, 0xE0, 0x9C, 0xEB, 0x5A};
	uint8_t pRootKey_5[16] = {0x9F, 0x30, 0x7D, 0x3B, 0xBC, 0x41, 0x7D, 0xB6, 0x12, 0xE2, 0x46, 0xB1, 0xA8, 0xA5, 0x1D, 0x93};
	//uint8_t pRootKey_6[16] = {0x75, 0xfb, 0xac, 0xee, 0x02, 0x97, 0x63, 0x97, 0x7d, 0x03, 0xe3, 0x27, 0x5a, 0x9f, 0x3c, 0x08};	
	uint8_t pRootKey_6[16] = {0x7b, 0xde, 0x18, 0x70, 0x1a, 0xc2, 0x52, 0xb9, 0xa9, 0xb6, 0x8a, 0x08, 0x8c, 0x9a, 0x8f, 0x82};
	
	result = encrypttest(pRootKey_1, input_len, 1);
	result = encrypttest(pRootKey_2, input_len, 2);
	result = encrypttest(pRootKey_3, input_len, 3);
	result = encrypttest(pRootKey_4, input_len, 4);
	result = encrypttest(pRootKey_5, input_len, 5);
	result = encrypttest(pRootKey_6, input_len, 6);
*/
//解密	
/*	printf(">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>    Decrypted_key start:    >>>>>>>>>>>>>>>>>>>\n");
	uint8_t encryptedKey_1[128] = {0xB0, 0xCA, 0x05, 0x0A, 0x08, 0x87, 0x99, 0x62, 0xC6, 0x46, 0xBB, 0x6A, 0xAC, 0x45, 0x8F, 0xF0, 0xFB, 0x2C, 0xD6, 0x77, 0x94, 0x84, 0xAC, 0x36, 0xF3, 0xFD, 0xCA, 0xB9, 0x09, 0x50, 0xC7, 0x79};
	uint8_t encryptedKey_2[128] = {0x4A, 0x3C, 0x93, 0x89, 0xF9, 0xA9, 0xF8, 0x1D, 0xD0, 0x19, 0x46, 0x7E, 0x77, 0x17, 0xBB, 0xF5, 0xFB, 0x2C, 0xD6, 0x77, 0x94, 0x84, 0xAC, 0x36, 0xF3, 0xFD, 0xCA, 0xB9, 0x09, 0x50, 0xC7, 0x79};
	uint8_t encryptedKey_3[128] = {0xB2, 0xC4, 0xC4, 0xC2, 0x65, 0x27, 0xA3, 0xC9, 0xCD, 0xF1, 0xB7, 0xCA, 0xD8, 0xE3, 0x6D, 0xA2, 0xFB, 0x2C, 0xD6, 0x77, 0x94, 0x84, 0xAC, 0x36, 0xF3, 0xFD, 0xCA, 0xB9, 0x09, 0x50, 0xC7, 0x79};
	uint8_t encryptedKey_4[128] = {0xCF, 0xCB, 0x58, 0x35, 0x83, 0x90, 0x47, 0x1E, 0x1B, 0x28, 0x8D, 0x1C, 0x8F, 0x76, 0x49, 0xB3, 0xFB, 0x2C, 0xD6, 0x77, 0x94, 0x84, 0xAC, 0x36, 0xF3, 0xFD, 0xCA, 0xB9, 0x09, 0x50, 0xC7, 0x79};
	uint8_t encryptedKey_5[128] = {0xAF, 0x1E, 0xFF, 0x58, 0xFA, 0x40, 0x3D, 0x49, 0xCA, 0x0F, 0x12, 0x93, 0x2D, 0x1D, 0x10, 0x40, 0xFB, 0x2C, 0xD6, 0x77, 0x94, 0x84, 0xAC, 0x36, 0xF3, 0xFD, 0xCA, 0xB9, 0x09, 0x50, 0xC7, 0x79};
	//uint8_t encryptedKey_6[128] = {0x5f, 0x1f, 0x2f, 0x06, 0x0c, 0xa7, 0x04, 0x94, 0x10, 0xed, 0xb9, 0x71, 0x96, 0x3f, 0x81, 0x92, 0x8c, 0x08, 0xe9, 0x96, 0x9b, 0xff, 0x63, 0xad, 0x2b, 0xbb, 0x18, 0x1c, 0x10, 0x86, 0x49, 0xb1};

	input_len = 32;
	result = decrypttest(encryptedKey_1, input_len, 1);
	result = decrypttest(encryptedKey_2, input_len, 2);
	result = decrypttest(encryptedKey_3, input_len, 3);
	result = decrypttest(encryptedKey_4, input_len, 4);
	result = decrypttest(encryptedKey_5, input_len, 5);
	//result = decrypttest(encryptedKey_6, input_len, 6);
*/	
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

	
	if(1)
	{
		DEBUGLOG(" *****************************************************************wifi init incompletely!");
		//重上电默认需要关闭Wifi
		sleep(2);
		uint8_t  wifi_RemindStatus = 0;
		dataPool->setPara(WIFI_REMINDSTATUS_TD, &wifi_RemindStatus, sizeof(wifi_RemindStatus));
		wifi_OpenOrClose(0);//关闭
		tboxInfo.operateionStatus.wifiStartStatus = -1;
	}
	

#if 0
	ret = pthread_create(&IVIThreadId, &IVIAttr, IVIThread, NULL);
	if(0 != ret) 
	{
		printf("can't create thread: %s\n",strerror(ret)); 
		exit(-1);
	}
#endif

#if 1
	ret = pthread_create(&factoryThreadId, NULL, factoryThread, NULL);
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


