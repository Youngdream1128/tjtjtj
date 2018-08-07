#include "LTEModuleAtCtrl.h"


/*****************************************************************************
* Function Name : networkConnectionStatus
* Description   : 网络连接状态
* Input			: int result
* Output        : None
* Return        : None
* Auther        : ygg
* Date          : 2018.01.18
*****************************************************************************/
void networkConnectionStatus(int result)
{
	//printf("########### result=%d ###########\n", result);
	if (1 == result)
	{
		//printf("Lte tcp open success!\n");
		tboxInfo.networkStatus.isLteNetworkAvailable = NETWORK_LTE;
	}
	else
	{
		//printf("Fail to retrieve IP, please check your module.\n");
		tboxInfo.networkStatus.isLteNetworkAvailable = NETWORK_NULL;
	}
}

/*****************************************************************************
* Function Name : LTEModuleAtCtrl
* Description   : 构造函数
* Input			: None
* Output        : None
* Return        : None
* Auther        : ygg
* Date          : 2018.01.18
*****************************************************************************/
LTEModuleAtCtrl::LTEModuleAtCtrl()
{
	//if (pthread_create(&timingThreadId, NULL, timingTaskThread, this) != 0)
	{
		//printf("Can not creat timingThread:%s\n", strerror(errno));
	}
}

int LTEModuleAtCtrl::atCtrlInit()
{
	NetworkInit();
        switchVoiceChannel();
	uint8_t emmcPartition;	
	
	if((emmcDeviceCheck() == 0) && (emmcPartitionCheck() == -1)) //0))
	{
		if(checkEmmcMountFolderCanBeWrite() == -1) //0)
		{
			dataPool->getTboxConfigInfo(emmcStatusID, &emmcPartition, 1);

			if(emmcPartition == 0)
			{
			printf("\n#################### fdisk emmc start #############################\n");
			printf("\n#################### fdisk emmc start #############################\n");
			//cfdisk(2, 1048576);
			lte_led_blink(250, 250);
			cfdisk(1, 0);
				emmcPartition = 1;
				dataPool->setTboxConfigInfo(emmcStatusID, &emmcPartition, 1);
			switch9011();
			atReset();
			lte_led_on();
			printf("\n#################### fdisk emmc over #############################\n");
			printf("\n#################### fdisk emmc over #############################\n");
		}
	}
	}
	
}

/*****************************************************************************
* Function Name : ~LTEModuleAtCtrl
* Description   : 析构函数
* Input			: None
* Output        : None
* Return        : None
* Auther        : ygg
* Date          : 2018.01.18
*****************************************************************************/
LTEModuleAtCtrl::~LTEModuleAtCtrl()
{
	close(fd);
}

/*****************************************************************************
* Function Name : NetworkInit
* Description   : 网络初始化
* Input			: None
* Output        : None
* Return        : 0:success
* Auther        : ygg
* Date          : 2018.01.18
*****************************************************************************/
int LTEModuleAtCtrl::NetworkInit()
{
	char buff[30];
	char imei[30];
	char cimi[30];
	char iccid[30];
	
	fd = open(QUEC_AT_PORT, O_RDWR | O_NONBLOCK | O_NOCTTY);
	printf(" open(\"%s\") fd:%d \n", QUEC_AT_PORT, fd);

	// get IMEI
	memset(imei, 0, sizeof(imei));
	dataPool->getPara(IMEI_INFO, (void *)imei, sizeof(imei));
	printf("imei:%s\n",imei);

	memset(buff, 0, sizeof(buff));
	if(getIMEI(buff, sizeof(buff)) == 0)
	{
		printf("IMEI:%s\n",buff);

		if(memcmp(imei, buff,(strlen(buff))) != 0)
		{
			dataPool->setPara(IMEI_INFO, (void *)buff, strlen(buff));
		}
	}

	memset(cimi, 0, sizeof(cimi));
	dataPool->getPara(CIMI_INFO, (void *)cimi, sizeof(cimi));
	printf("cimi:%s\n",cimi);

	memset(buff, 0, sizeof(buff));
	if(getCIMI(buff, sizeof(buff)) == 0)
	{
		printf("CIMI:%s\n",buff);

		if(memcmp(cimi, buff,(strlen(buff))) != 0)
		{
			dataPool->setPara(CIMI_INFO, (void *)buff, strlen(buff));
		}
	}

	if(queryIsInsertSIM() > 0)
	{
		tboxInfo.networkStatus.isSIMCardPulledOut = 1;
		//enableMobileNetwork(1*2, networkConnectionStatus);

		memset(iccid, 0, sizeof(iccid));
		dataPool->getPara(SIM_ICCID_INFO, (void *)iccid, sizeof(iccid));

		// get ICCID
		memset(buff, 0, sizeof(buff));
		if(getICCID(buff, sizeof(buff)) == 0)
		{
			printf("ICCID:%s\n",buff);
			if(memcmp(iccid, buff,(strlen(buff))) != 0)
			{
			dataPool->setPara(SIM_ICCID_INFO, (void *)buff, strlen(buff));
		}
	}
	}
	else
	{
		printf("no sim card\n");
		tboxInfo.networkStatus.isSIMCardPulledOut = 0;
	}

	return 0;
}

/*****************************************************************************
* Function Name : timingTaskThread
* Description   : 定时任务
* Input			: None
* Output        : None
* Return        : None
* Auther        : ygg
* Date          : 2018.01.18
*****************************************************************************/
void *LTEModuleAtCtrl::timingTaskThread(void *args)
{
	LTEModuleAtCtrl *p_LTENetwork = (LTEModuleAtCtrl *)args;

	while(1)
	{
		//printf("\n\n66666666666666\n\n");
		//enableMobileNetwork(1, networkConnectionStatus);
		//p_LTENetwork->audioPlayTTS("123abcd");
		sleep(2);
		break;
	}
}

/***************************************************************************
* Function Name: sendATCmd 					
* Function Introduction: send at command to LTE module
* Parameter description:
*     atCmd    : at command
*     finalRsp : at command's response result.
*     buff     : if need function return result, you should pass the buff,
*                then the function will store the result into buff 
*     buffSize : buff size, At least 100bits.
*     timeout_ms: time out
* Function return value:  -1: failed; 0: return error,
*                         other: return data length.
* Auther        : ygg
* Date          : 2017.09.08								
****************************************************************************/
int LTEModuleAtCtrl::sendATCmd(char* atCmd, char* finalRsp, char *buff, uint32_t buffSize, long timeout_ms)
{
    int retval;
	int len;
    int readLen;
	uint8_t count = 0;
    bool recvFlag = false;
    char strAT[100];
    char strFinalRsp[100];
    char strResponse[100];
	
    fd_set fds;
    struct timeval timeout = {0, 0};

    memset(strAT, 0x0, sizeof(strAT));
	
    sprintf(strFinalRsp, "\r\n%s", finalRsp);

    timeout.tv_sec = timeout_ms / 1000;
    //timeout.tv_usec = timeout_ms % 1000;
	
    len = strlen(atCmd);
    if ((atCmd[len - 1] != '\r') && (atCmd[len - 1] != '\n'))
    {
        len = sprintf(strAT, "%s\r\n", atCmd);
        strAT[len] = 0;
		//printf("strAT:%s\n", strAT);
    }

    if((retval = write(fd, strAT, len)) > 0)
		//printf("send at success, retval:%d\n", retval);

    if ((buff != NULL) &&(buffSize != 0))
    {
		memset(buff, 0, buffSize);
    }
 	
    while (count++ <= 3)
    {
        FD_ZERO(&fds);
        FD_SET(fd, &fds);

        switch (select(fd + 1, &fds, NULL, NULL, &timeout))
        {
            case -1:
				printf("select error:%d\n", errno);
				if (errno == EINTR) 
					continue;
				return -1;

            case 0:
                printf("select time out.\n");
                return -1;

            default:
                if (FD_ISSET(fd, &fds))
                {
                    do
                    {
                        memset(strResponse, 0x0, sizeof(strResponse));
                        readLen = read(fd, strResponse, sizeof(strResponse));
                   		/*printf("\n111 strResponse:%s, 222  strFinalRsp:%s, 333  readLen:%d \n", strResponse,strFinalRsp,readLen);
						for(int i = 0; i<readLen; i++)
							printf("%02x ",*(strResponse+i));
						printf("\n\n");*/

						if (readLen > 0)
						{
							if(strstr(strResponse, strFinalRsp))
							{
								//printf("\nAAAA\n");
								retval = readLen;
								recvFlag = true;
							}
							else if(strstr(strResponse, "+CME ERROR:")
								 || strstr(strResponse, "+CMS ERROR:") 
								 || strstr(strResponse, "ERROR"))
							{
								//printf("\nBBBB\n");
								retval = 0;
								recvFlag = true;
							}
							else
							{
								//printf("\nCCCCC\n");
								retval = -1;
								printf("No final rsp\n");
							}
					   }

						if((buff != NULL) && (buffSize != 0) && (buffSize >=  sizeof(strResponse)))
						{
							memcpy(buff, strResponse, readLen);
						}
                    }
                    while ((readLen > 0) && (sizeof(strResponse) == readLen));
                }
                else
                {
                    printf("FD is missed\n");
                }
                break;
        }

        if (recvFlag)
        {
            break;
        }
	}
	
    return retval;
}

/*****************************************************************************
* Function Name : getLTEModuleRevision
* Description   : 获取模块版本
* Input			: char *pBuff;
*                 int size;
* Output        : None
* Return        : -1: failed; 0: return error,
*                 other: return data length.
* Auther        : ygg
* Date          : 2018.01.18
*****************************************************************************/
int LTEModuleAtCtrl::getLTEModuleRevision(char *pBuff, int size)
{
	int retval;
	uint8_t i;
	char strResult[128];
	char *pos = NULL;

	for (i = 0; i < 3; i++)
	{
		memset(strResult, 0, sizeof(strResult));
		retval = sendATCmd((char*)"ATI", (char*)"OK", strResult, sizeof(strResult), 2000);
		if (retval > 0)
		{
			pos = strstr(strResult, "Revision: ");
			if (pos != NULL)
			{
				pos += strlen("Revision: ");
				for (i = 0; i < size; i++)
				{
					if (*pos == '\r')
						break;
					*pBuff++ =  *pos++;
				}
			}
			
			break;
		}
	}

	return retval;
}

/*****************************************************************************
* Function Name : queryIsInsertSIM
* Description   : 查询是否插入SIM卡
* Input			: None
* Output        : None
* Return        : -1: failed; 0: return error,
*                 other: return data length.
* Auther        : ygg
* Date          : 2018.01.18
*****************************************************************************/
int LTEModuleAtCtrl::queryIsInsertSIM()
{
	int i;
	int retval = -1;
	char strResult[100];

	memset(strResult, 0, sizeof(strResult));

	for (i = 0; i < 5; i++)
	{
		memset(strResult, 0, sizeof(strResult));
		retval = sendATCmd((char*)"AT+CPIN?", (char*)"+CPIN: READY", strResult, sizeof(strResult), 5000);
		printf("111 %d\n", retval);
		if (retval > 0)
		{
			printf("strResult:%s,  retval:%d \n",strResult, retval);
			break;
		}
	}

	//printf("strResult:%s,  retval:%d \n",strResult, retval);
	//for(int i = 0; i<(int)sizeof(strResult); i++)
		//printf("%02x ",*(strResult+i));
	//printf("\n");

	return retval;
}

/*****************************************************************************
* Function Name : queryCSQ
* Description   : 查询信号函数
* Input			: uint8_t *rssi
*                 uint8_t *ber
* Output        : None
* Return        : -1: failed; 0: return error,
*                 other: return data length.
* Auther        : ygg
* Date          : 2018.01.18
*****************************************************************************/
int LTEModuleAtCtrl::queryCSQ(uint8_t *rssi, uint8_t *ber)
{
	int retval = -1;
	char *pos = NULL;
	uint8_t i, j;
	char rspStr[32];
	char result[4];

	memset(rspStr, 0, sizeof(rspStr));
	retval = sendATCmd((char*)"AT+CSQ", (char*)"OK", rspStr, sizeof(rspStr), 2000);
	if(retval > 0)
	{
		pos = strstr(rspStr, "+CSQ: ");
		if (pos != NULL)
		{
			pos += strlen("+CSQ: ");

			for (j = 0; j < 2; j++)
			{
				memset(result, 0, sizeof(result));
				for (i = 0; i < sizeof(result) - 1; i++)
				{
					if ((*pos != ',') && (*pos != '\0') && (*pos != '\r'))
					{
						result[i] =  *pos;
					}
					else
					{
						pos++;
						break;
					}
					pos++;
				}
				if (j == 0)
					*rssi = atoi(result);
				else
					*ber = atoi(result);
			}
		}
	}

	return retval;
}

/*****************************************************************************
* Function Name : queryReg
* Description   : 查询网络注册函数
* Input			: None
* Output        : None
* Return        : regState
* Auther        : ygg
* Date          : 2018.01.18
*****************************************************************************/
int LTEModuleAtCtrl::queryReg()
{
	int retval;
	char strResult[100];
	char *pos = NULL;
	uint8_t i,j;
	
	char state[4];
	uint32_t regState = 4;

	memset(strResult, 0, sizeof(strResult));
	retval = sendATCmd((char*)"AT+CREG?", (char*)"OK", strResult, sizeof(strResult), 2000);
	if (retval > 0)
	{
		pos = strstr(strResult, "+CREG: ");
		if (pos != NULL)
		{
			pos += strlen("+CREG: ");

			for (i = 0; i < 2; i++)
			{
				memset(state, 0, sizeof(state));
				for (j = 0; j < sizeof(state) - 1; j++)
				{
					if ((*pos != ',') && (*pos != '\0') && (*pos != '\r'))
					{
						state[j] =  *pos;
					}
					else
					{
						pos++;
						break;
					}
					pos++;
				}
				if (i == 1)
					regState = atoi(state);
			}
		}
	}

	return regState;
}

/*****************************************************************************
* Function Name : switch9011
* Description   : 切换9011端口
* Input			: None
* Output        : None
* Return        : 0:success, -1:faild
* Auther        : ygg
* Date          : 2018.04.24
*****************************************************************************/
int LTEModuleAtCtrl::switch9011()
{
	int retval = -1;
	char strResult[100];

	DEBUGLOG("9999999999999999999\n");
	memset(strResult, 0, sizeof(strResult));
	retval = sendATCmd((char*)"AT+CUSBPIDSWITCH=9011,1,1", (char*)"OK", strResult, sizeof(strResult), 20000);
	if (retval > 0)
	{
		printf("888888888888888888888 strResult:%s,  retval:%d \n",strResult, retval);
		retval = 0;
	}

	return retval;
}

/*****************************************************************************
* Function Name : atReset
* Description   : 重启
* Input			: None
* Output        : None
* Return        : 0:success, -1:faild
* Auther        : ygg
* Date          : 2018.04.24
*****************************************************************************/
int LTEModuleAtCtrl::atReset()
{
	int retval = -1;
	char strResult[100];

	DEBUGLOG("777777777777777\n");
	memset(strResult, 0, sizeof(strResult));
	retval = sendATCmd((char*)"AT+CRESET", (char*)"OK", strResult, sizeof(strResult), 20000);
	if (retval > 0)
	{
		printf("7777777777777777 strResult:%s,  retval:%d \n",strResult, retval);
		retval = 0;
	}

	return retval;
}

/*****************************************************************************
* Function Name : emmcDeviceCheck
* Description   : emmc驱动检测
* Input			: None
* Output        : None
* Return        : 0:success, -1:faild
* Auther        : ygg
* Date          : 2018.04.24
*****************************************************************************/
int LTEModuleAtCtrl::emmcDeviceCheck()
{
	int ret;
	FILE *fp;
	char buff[100] = {0};
	char *ptr1 = NULL;
	char *ptr2 = NULL;
	
	fp = popen("ls /dev | grep mmc*","r");
	if(fread(buff, sizeof(char), sizeof(buff), fp) > 0)
	{
		DEBUGLOG("00000000000000000 emmcDeviceCheck, buff:%s\n", buff);
		ptr1 = strstr(buff, "mmcblk0");
		ptr2 = strstr(buff, "mmcblk0rpmb");
		if((ptr1 != NULL) && (ptr2 != NULL))
			ret = 0;
		else
			ret = -1;
	}
	else
	{
		ret = -1;
    }
	pclose(fp);

	DEBUGLOG("00000000111111 emmcDeviceCheck, buff:%s, ret:%d\n", buff, ret);

	return ret;
}

/*****************************************************************************
* Function Name : emmcPartitionCheck
* Description   : emmc分区检测
* Input			: None
* Output        : None
* Return        : 0:success, -1:faild
* Auther        : ygg
* Date          : 2018.04.24
*****************************************************************************/
int LTEModuleAtCtrl::emmcPartitionCheck()
{
	int ret = -1;
	FILE *fp;
	char buff[10] = {0};
		
	//fp = popen("ls /dev | grep mmcblk0p1 |grep -v \"grep\" |wc -l","r");
	fp = popen("ls /dev | grep mmcblk0p1","r");
	if(fread(buff, sizeof(char), sizeof(buff), fp) > 0)
	{
		DEBUGLOG("22222222222\n");
		ret = 0;
	}
	pclose(fp);

	DEBUGLOG("22222222222	emmcPartitionCheck, buff:%s, ret:%d\n", buff, ret);

	return ret;
}

/*****************************************************************************
* Function Name : checkEmmcMountFolderCanBeWrite
* Description   : 判断emmc挂载目录是否可写
* Input			: None
* Output        : None
* Return        : 0:success, -1:faild
* Auther        : ygg
* Date          : 2018.04.24
*****************************************************************************/
int LTEModuleAtCtrl::checkEmmcMountFolderCanBeWrite()
{
#define MOUNT_FOLDER  "/media/card"
	int ret = -1;

	DEBUGLOG("aaaaaaaaaaaaaaaaaaaaaaaa\n");

    if(access(MOUNT_FOLDER, W_OK) == 0){
        DEBUGLOG("folder %s can be write\n", MOUNT_FOLDER);
        ret = 0;
    }else{
        DEBUGLOG("folder %s cannot be write\n", MOUNT_FOLDER);
        ret = -1;
	}
	DEBUGLOG("bbbbbbbbbbbbbbbbbbbbbbbbbbbbb\n");

	return ret;
}

/*****************************************************************************
* Function Name : cfdisk
* Description   : 对emmc进行分区
* Input			: int partitionNum:分区个数(支持2个分区)
                  int partitionSize 分区大小
                  partitionNum：1,partitionSize不需要设置
                                2,partitionSize需指定第一个分区的大小默认kb
                                  第二个分区大小不需要设置大小
* Output        : None
* Return        : 0:success, -1:faild
* Auther        : ygg
* Date          : 2018.04.24
*****************************************************************************/
int LTEModuleAtCtrl::cfdisk(int partitionNum, int partitionSize)
{
	int retval = -1;
	char strResult[100];
	char fdiskCMD[100] = {0};
	
	memset(strResult, 0, sizeof(strResult));

	if(partitionNum == 1)
	{
		DEBUGLOG("22222222222	cfdisk,\n");
		retval = sendATCmd((char*)"AT+CFDISK=1", (char*)"OK", strResult, sizeof(strResult), 50000000);
		if (retval > 0)
		{
			DEBUGLOG(" 3333333333333  strResult:%s,  retval:%d \n",strResult, retval);
			retval = 0;
		}
		sleep(8);
	}
	else if(partitionNum == 2)
	{
		snprintf(fdiskCMD, sizeof(fdiskCMD), "AT+CFDISK=2,%s", partitionSize);
		printf("fdiskCMD:%s\n",fdiskCMD);
		retval = sendATCmd((char*)fdiskCMD, (char*)"OK", strResult, sizeof(strResult), 50000000);
		//retval = sendATCmd((char*)"AT+CFDISK=2,1048576", (char*)"OK", strResult, sizeof(strResult), 20000);
		if (retval > 0)
		{
			DEBUGLOG("strResult:%s,  retval:%d \n",strResult, retval);
			retval = 0;
		}
	}

	if(retval == 0){
		retval = sendATCmd((char*)"AT+CFDISK", (char*)"OK", strResult, sizeof(strResult), 50000000);
		if (retval > 0)
		{
			DEBUGLOG(" 444444444444444 strResult:%s,  retval:%d \n",strResult, retval);
			retval = 0;
		}
	}
	
	return retval;
}

/*****************************************************************************
* Function Name : voiceLoopBack
* Description   : 切换音频芯片回环测试,
                  AT+CLOOPBACK=3,0 关闭回环测试,
                  AT+CLOOPBACK=3,1 打开回环测试,
* Input			: None
* Output        : None
* Return        : 0:success, -1:faild
* Auther        : ygg
* Date          : 2018.01.18
*****************************************************************************/
int LTEModuleAtCtrl::voiceLoopBack()
{
	int i;
	int retval = -1;

	for (i = 0; i < 3; i++)
	{
		retval = sendATCmd((char*)"AT+CLOOPBACK=3,1", (char*)"OK", NULL, 0, 5000);
		if (retval > 0)
		{
			break;
		}
	}

	return retval;
}

/*****************************************************************************
* Function Name : switchCwiic
* Description   : 解决MIC_IN+ 输入，MIC_IN- 信号接地,音频输出到4G,4G没有
                  音频输出
* Input			: None
* Output        : None
* Return        : 0:success, -1:faild
* Auther        : ygg
* Date          : 2018.01.18
*****************************************************************************/
int LTEModuleAtCtrl::switchCwiic()
{
	int i;
	int retval = -1;

	for (i = 0; i < 3; i++)
	{
		retval = sendATCmd((char*)"at+cwiic=0x34,0x58,0x1,1", (char*)"OK", NULL, 0, 5000);
		if (retval > 0)
		{
			break;
		}
	}
	
	return retval;
}

/*****************************************************************************
* Function Name : switchVoiceChannel
* Description   : 切换音频芯片输出模式,默认:AT+CSDVC=1 为line out输出,
                  AT+CSDVC=3 为SPK OUT输出,
* Input			: None
* Output        : None
* Return        : 0:success, -1:faild
* Auther        : ygg
* Date          : 2018.01.18
*****************************************************************************/
int LTEModuleAtCtrl::switchVoiceChannel()
{
	int i;
	int retval = -1;
	//char strResult[100];

	for (i = 0; i < 3; i++)
	{
		//memset(strResult, 0, sizeof(strResult));
		retval = sendATCmd((char*)"AT+CSDVC=3", (char*)"OK", NULL, 0, 5000);
		if (retval > 0)
		{
			//printf("strResult:%s,  retval:%d \n",strResult, retval);
			break;
		}
	}
	//printf("strResult:%s,  retval:%d \n",strResult, retval);

	return retval;
}

/*****************************************************************************
* Function Name : getICCID
* Description   : 查询ICCID函数
* Input			: char *pBuff
*                 int size
* Output        : None
* Return        : 0:success, -1:faild
* Auther        : ygg
* Date          : 2018.01.18
*****************************************************************************/
int LTEModuleAtCtrl::getICCID(char *pBuff, int size)
{
	uint8_t i;
	int retval;
	char strResult[100];
	char *pos = NULL;

	if(size < 20)
	{
		printf("passed the memory space is too small!\n ");
		return -1;
	}
	
	for (i = 0; i < 3; i++)
	{
		memset(strResult, 0, sizeof(strResult));
		retval = sendATCmd((char*)"AT+CICCID", (char*)"OK", strResult, sizeof(strResult), 2000); //AT+QCCID
		if (retval > 0)
		{
			pos = strstr(strResult, "+ICCID: ");
			
			if (pos != NULL)
			{
				pos += strlen("+ICCID: ");
				for (i = 0; i < size; i++)
				{
					if (*pos == '\r')
						break;
					*pBuff++ =  *pos++;
				}
			}
		
			break;
		}
	}

	return 0;
}

/*****************************************************************************
* Function Name : getIMEI
* Description   : 查询IMEI
* Input			: char *pBuff
*                 int size
* Output        : None
* Return        : 0:success, -1:faild
* Auther        : ygg
* Date          : 2018.01.18
*****************************************************************************/
int LTEModuleAtCtrl::getIMEI(char *pBuff, int size)
{
	uint8_t i;
	int retval;
	char strResult[100];
	char *pos = NULL;

	for (i = 0; i < 3; i++)
	{
		memset(strResult, 0, sizeof(strResult));
		retval = sendATCmd((char*)"AT+CGSN", (char*)"OK", strResult, sizeof(strResult), 3000);  //AT+GSN
		if (retval > 0)
		{
			pos = strstr(strResult, "\r\n");
			
			if (pos != NULL)
			{
				pos += strlen("\r\n");
				for (i = 0; i < size; i++)
				{
					if (*pos == '\r')
						break;
					*pBuff++ =  *pos++;
				}
			}
		
			break;
		}
	}

	return 0;
}

/*****************************************************************************
* Function Name : getCIMI
* Description   : 查询CIMI
* Input			: char *pBuff
*                 int size
* Output        : None
* Return        : 0:success, -1:faild
* Auther        : ygg
* Date          : 2018.01.18
*****************************************************************************/
int LTEModuleAtCtrl::getCIMI(char *pBuff, int size)
{
	uint8_t i;
	int retval;
	char strResult[100];
	char *pos = NULL;

	if(pBuff == NULL)
	{
		printf("pBuff error!\n ");
		return -1;
	}

	for (i = 0; i < 3; i++)
	{
		memset(strResult, 0, sizeof(strResult));
		retval = sendATCmd((char*)"AT+CIMI", (char*)"OK", strResult, sizeof(strResult), 3000);  //AT+GSN
		if (retval > 0)
		{
			pos = strstr(strResult, "\r\n");
			
			if (pos != NULL)
			{
				pos += strlen("\r\n");
				for (i = 0; i < size; i++)
				{
					if (*pos == '\r')
						break;
					*pBuff++ =  *pos++;
				}
			}
		
			break;
		}
	}

	return 0;
}

/*****************************************************************************
* Function Name : callPhone
* Description   : 打电话
* Input			: char *phoneNumber
* Output        : None
* Return        : 0:success, -1:faild
* Auther        : ygg
* Date          : 2018.01.18
*****************************************************************************/
int LTEModuleAtCtrl::callPhone(char *phoneNumber)
{
	int retval;
	char rspStr[ARRAY_SIZE];
	char callPhoneCmd[32];

	memset(callPhoneCmd, 0, sizeof(callPhoneCmd));
	snprintf(callPhoneCmd, sizeof(callPhoneCmd), "ATD%s;", phoneNumber);
	
	memset(rspStr, 0, sizeof(rspStr));
	retval = sendATCmd(callPhoneCmd, (char*)"OK", rspStr, sizeof(rspStr), 10000);

	return retval;
}

/*****************************************************************************
* Function Name : answerOrHangUpPhone
* Description   : 接电话或挂起电话
* Input			: bool isConnect
* Output        : None
* Return        : 0:success, -1:faild
* Auther        : ygg
* Date          : 2018.01.18
*****************************************************************************/
int LTEModuleAtCtrl::answerOrHangUpPhone(bool isConnect)
{
	int retval;
	uint8_t i;

	for (i = 0; i < 3; i++)
	{
		if (isConnect == true) //answer phone
			retval = sendATCmd((char*)"ATA", (char*)"OK", NULL, 0, 2000);
		else //hang up phone
			retval = sendATCmd((char*)"ATH", (char*)"OK", NULL, 0, 2000);
		if (retval > 0)
			break;
	}

	return retval;
}

/*****************************************************************************
* Function Name : answerOrHangUpPhone
* Description   : 发送短信
* Input			: char *phoneNum, 
*                 uint8_t *pData, 
*                 uint32_t datalen, 
*                 unsigned char SMSMode
* Output        : None
* Return        : 0:success, -1:faild
* Auther        : ygg
* Date          : 2018.01.18
*****************************************************************************/
int LTEModuleAtCtrl::sendSMS(char *phoneNum, uint8_t *pData, uint32_t datalen, unsigned char SMSMode)
{
	int retval;
	char rspStr[ARRAY_SIZE];
	char cmdStr[32];

	if ((phoneNum == NULL) || (pData == NULL) || (datalen == 0))
		return -1;

	memset(cmdStr, 0, sizeof(cmdStr));
	snprintf(cmdStr, sizeof(cmdStr), "AT+CMGF=%d", SMSMode);
	sendATCmd(cmdStr, (char*)"OK", rspStr, sizeof(rspStr), 2000);
	
	memset(rspStr, 0, sizeof(rspStr));
	sendATCmd((char*)"AT+CSCS=\"GSM\"", (char*)"OK", rspStr, sizeof(rspStr), 2000);

	memset(cmdStr, 0, sizeof(cmdStr));
	snprintf(cmdStr, sizeof(cmdStr), "AT+CMGS=\"%s\"", phoneNum);
	
	memset(rspStr, 0, sizeof(rspStr));
	retval = sendATCmd(cmdStr, (char*)">", rspStr, sizeof(rspStr), 2000);
	if (retval > 0)
	{
		write(fd, pData, datalen);
		memset(cmdStr, 0, sizeof(cmdStr));
		cmdStr[0] = 0x1a;
		
		retval = sendATCmd(cmdStr, (char*)"+CMGS:", rspStr, sizeof(rspStr), 2000);
	}

	return retval;
}

/*****************************************************************************
* Function Name : audioPlayTTS
* Description   : 语音播报
* Input			: char *tts
* Output        : None
* Return        : 0:success, -1:faild
* Auther        : ygg
* Date          : 2018.01.18
*****************************************************************************/
int LTEModuleAtCtrl::audioPlayTTS(char *tts)
{
    int retval;

    if (strlen(tts) > 2000)
        return  -1;

    char *ttsCmd = (char*)malloc(2 *1024);
    if (ttsCmd == NULL)
    {
        printf("malloc ttsCmd failed.\n");
        return  -1;
    }
    memset(ttsCmd, 0, 2*1024);

    snprintf(ttsCmd, sizeof(char) *1024, "AT+CTTS=2,\"%s\"", tts);  //AT+QTTS=2
	
    printf("ttsCmd:%s",ttsCmd);
    retval = sendATCmd(ttsCmd, (char*)"OK", NULL, 0, 5000);

    if (ttsCmd != NULL)
        free(ttsCmd);

    return retval;
}

/*****************************************************************************
* Function Name : getNetworkingType
* Description   : 获得网络类型
* Input			: uint8_t isOpen
* Output        : None
* Return        : 0:success, -1:faild
* Auther        : ygg
* Date          : 2018.01.18
*****************************************************************************/
int LTEModuleAtCtrl::getNetworkingType()
{
	FILE *fp;
	char buff[512];
	
	memset(buff, 0, sizeof(buff));
	
	fp = popen("route | sed -n '/default/p'","r");
	if(fread(buff, sizeof(char), sizeof(buff), fp) > 0)
		;//printf("buff:%s\n",buff);
	else
	{
		tboxInfo.networkStatus.NetworkConnectionType = NETWORK_NULL;
	}
	
	pclose(fp);
	
	if((strstr(buff, "rmnet_data0") != NULL) || (strstr(buff, "rmnet_data1") != NULL))
	{
		tboxInfo.networkStatus.NetworkConnectionType = NETWORK_LTE;
	}
	else if(strstr(buff, "wlan0") != NULL)
	{
		tboxInfo.networkStatus.NetworkConnectionType = NETWORK_WIFI;
	}
	else
	{
		tboxInfo.networkStatus.NetworkConnectionType = NETWORK_NULL;
	}

}


