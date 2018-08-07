#include "mcuUart.h"

mcuUart * mcuUart::m_mcuUart =NULL;
extern GBT32960 *p_GBT32960;

/*****************************************************************************
* Function Name : mcuUart
* Description   : 构造函数
* Input			: None
* Output        : None
* Return        : None
* Auther        : ygg
* Date          : 2018.01.18
*****************************************************************************/
mcuUart::mcuUart()
{
	fd = -1;
	memset(&SigEvent, 0, sizeof(SIG_Event_t));
	m_mcuUart = this;
	
	mcuUartInit();

	if(pthread_create(&CheckEventThreadId, NULL, CheckEventThread, this) != 0)
		MCULOG("Cannot creat CheckEventThread:%s\n", strerror(errno));
	
	while(1)
	{
		mcuUartReceiveData();
	}
}

/*****************************************************************************
* Function Name : close_uart
* Description   : 关闭打开的串口用于升级完重启系统
* Input			: None
* Output        : None
* Return        : None
* Auther        : ygg
* Date          : 2018.01.18
*****************************************************************************/
void mcuUart::close_uart()
{
	close(fd);
}

void *mcuUart::CheckEventThread(void *arg)
{
	pthread_detach(pthread_self()); 
	mcuUart *pmcuUart = (mcuUart*)arg;
	int iRet = -1;
	int iLoop = 0;
	while(1)
	{
		sleep(1);

		if(CFAWACP::cfawacp->m_loginState == 2 && CFAWACP::cfawacp->m_ConnectedState == true)
		{
			if(pmcuUart->SigEvent.s_VehCollideAlarm == 1)	//车辆碰撞
			{
				pmcuUart->reportEventCmd(CFAWACP::cfawacp->timingReportingData, 1, ACPApp_EmergencyDataID);
				
				for(iLoop = 0; iLoop < 3; iLoop++)
	 			{
					printf("@@@@@@@@@ login CheckEventThread voiceCall  @@@@@@@@@@@@@:%d---PhoneNum: %s\n",iLoop,p_FAWACPInfo_Handle->RemoteDeviceConfigInfo[0].EmergedCall);
	 				iRet = voiceCall(p_FAWACPInfo_Handle->RemoteDeviceConfigInfo[0].EmergedCall);
					if(iRet == 0){
						pmcuUart->SigEvent.s_VehCollideAlarm = -1;
						break;
						}
	 		 	}
			}
			usleep(200);
			if(pmcuUart->SigEvent.s_DoorIntrusAlarm == 1)	//车门入侵
			{
				pmcuUart->reportEventCmd(CFAWACP::cfawacp->timingReportingData, 1, ACPApp_VehicleAlarmID);
			}
			usleep(200);
			//保养提醒报警	暂时定剩余保养里程小于500时为保养提醒
			if(p_FAWACPInfo_Handle->VehicleCondData.RemainUpkeepMileage <= 500)
			{
				pmcuUart->reportEventCmd(CFAWACP::cfawacp->timingReportingData, 3, ACPApp_VehicleAlarmID);
			}
			usleep(200);
			if(pmcuUart->SigEvent.s_LowOilAlarm == 1)		//低油量报警
			{
				pmcuUart->reportEventCmd(CFAWACP::cfawacp->timingReportingData, 4, ACPApp_VehicleAlarmID);
			}
			usleep(200);
			if(pmcuUart->SigEvent.s_VehOutFileAlarm == 1)			//紧急熄火报警
			{
				pmcuUart->reportEventCmd(CFAWACP::cfawacp->timingReportingData, 5, ACPApp_VehicleAlarmID);
			}
			usleep(200);
			//故障事件上报
			if(  p_FAWACPInfo_Handle->AcpCodeFault.ACPCODEFAULTEMSState == 1 || p_FAWACPInfo_Handle->AcpCodeFault.ACPCODEFAULTTCUState == 1 || 
				 p_FAWACPInfo_Handle->AcpCodeFault.ACPCODEFAULTSRSState == 1 || p_FAWACPInfo_Handle->AcpCodeFault.ACPCODEFAULTESPState == 1 || 
				 p_FAWACPInfo_Handle->AcpCodeFault.ACPCODEFAULTABSState == 1 || p_FAWACPInfo_Handle->AcpCodeFault.ACPCODEFAULTABSState == 1 || 
				 p_FAWACPInfo_Handle->AcpCodeFault.ACPCODEFAULTEPASState == 1|| p_FAWACPInfo_Handle->AcpCodeFault.ACPCODEFAULTOilPressureState == 1 || 
				 p_FAWACPInfo_Handle->AcpCodeFault.ACPCODEFAULTLowOilIDState == 1 || p_FAWACPInfo_Handle->AcpCodeFault.ACPCODEFAULTBrakeFluidLevelState == 1 || 
				 p_FAWACPInfo_Handle->AcpCodeFault.ACPCODEFAULTBBWState == 1 || p_FAWACPInfo_Handle->AcpCodeFault.ACPCODEFAULTTPMSState == 1 || 
				 p_FAWACPInfo_Handle->AcpCodeFault.ACPCODEFAULTESCLState == 1||  p_FAWACPInfo_Handle->AcpCodeFault.ACPCODEFAULTElecParkUnitState == 1 || 
				 p_FAWACPInfo_Handle->AcpCodeFault.ACPCODEFAULTBatteryChargeState == 1 || p_FAWACPInfo_Handle->AcpCodeFault.ACPCODEFAULTEmissionState == 1 || 
				 p_FAWACPInfo_Handle->AcpCodeFault.ACPCODEFAULTSTTState == 1 || p_FAWACPInfo_Handle->AcpCodeFault.ACPCODEFAULTACCState == 1 || 
				 p_FAWACPInfo_Handle->AcpCodeFault.ACPCODEFAULTEngineOverwaterState == 1)
			{
				pmcuUart->reportEventCmd(CFAWACP::cfawacp->timingReportingData, 18, ACPApp_VehicleCondUploadID);
			}
			//道路救援报警

			//车辆异动报警
		}
		else
		{
			if(pmcuUart->SigEvent.s_VehCollideAlarm == 1)
			{
	 			for(iLoop = 0; iLoop < 3; iLoop++)
	 			{
	 			
					printf("@@@@@@@@@ CheckEventThread voiceCall  @@@@@@@@@@@@@:%d---PhoneNum: %s\n",iLoop,p_FAWACPInfo_Handle->RemoteDeviceConfigInfo[0].EmergedCall);
					iRet = voiceCall(p_FAWACPInfo_Handle->RemoteDeviceConfigInfo[0].EmergedCall);
					if(iRet == 0){
						pmcuUart->SigEvent.s_VehCollideAlarm = -1;
						break;
						}
	 		 	}
			}
		}
			
		memset(&pmcuUart->SigEvent, 0, sizeof(SIG_Event_t));
	}

	pthread_exit(0);
}


//TSP下发命令->TBOX解析发送到MCU(上锁)->MCU下发至8090->8090执行动作反馈结果至MCU->MCU将数据设置TBOX结构(解锁)->TBOX反馈结果TSP
void mcuUart::reportEventCmd(callback_EventCmd cb_EventCmd, uint8_t MsgType, AcpAppID_E AppID)
{
	//回调函数控制MCU发送远程控制指定
	cb_EventCmd(MsgType, AppID);
}

/*****************************************************************************
* Function Name : mcuUartInit
* Description   : mcu串口初始化
* Input			: None
* Output        : None
* Return        : 0:success
* Auther        : ygg
* Date          : 2018.01.18
*****************************************************************************/
int mcuUart::mcuUartInit()
{
    unsigned int ui32count = 0;

    while (ui32count++ < 10)
    {
        if ((fd = open(MCU_UART_DEVICE, O_RDWR | O_NOCTTY)) == -1)
        {
			usleep(200);
            MCULOG("Can't Open Serial Port!,ui32count = %d\n", ui32count);
			if(ui32count == 9)
				exit(-1);
        }
        else
        {
            MCULOG("fd:%d, pid:%d\n", fd, getpid());
            break;
        }
    }

    ui32count = 0;
    while (ui32count++ < 10)
    {
        if (0 > setUartSpeed(fd, MCU_UART_SPEED))
        {
            MCULOG("set mcu uart speed failed!ui32count = %d\n", ui32count);
			if(ui32count == 9)
				exit(-1);
            sleep(1);
        }
        else
        {
            MCULOG("set uart speed sucessed!");
            break;
        }
    }

    ui32count = 0;
    while (ui32count++ < 10)
    {
        if (setUartParity(fd, MCU_UART_DATA_BITS, MCU_UART_STOP_BITS, MCU_UART_CHECK_BIT) == -1)
        {
            MCULOG("set mcu uart speed failed!,ui32count=%d", ui32count);
			if(ui32count == 9)
				exit(-1);
            sleep(1);
        }
        else
        {
            MCULOG("set uart parity sucessed!");
            break;
        }
    }

    return 0;
}

/*****************************************************************************
* Function Name : setUartSpeed
* Description   : 设置串口波特率
* Input			: int fd,
*                 int speed
* Output        : None
* Return        : 0:success, -1:faild
* Auther        : ygg
* Date          : 2018.01.18
*****************************************************************************/
int mcuUart::setUartSpeed(int fd, int speed)
{
	int speed_arr[] = { B921600, B460800, B230400, B115200, B38400, B19200,
						B9600,	 B4800,   B2400,   B1200,	B300,	B38400,
						B19200,  B9600,   B4800,   B2400,	B1200,	B300};
	
	int name_arr[] = { 921600, 460800, 230400, 115200, 38400, 19200,
					   9600,   4800,   2400,   1200,   300,   38400,
					   19200,  9600,   4800,   2400,   1200,  300};
    int status;
    struct termios Opt;
    tcgetattr(fd, &Opt);
	
    for (unsigned int i = 0; i < sizeof(speed_arr) / sizeof(int); i++)
    {
        if (speed == name_arr[i])
        {
            tcflush(fd, TCIOFLUSH);
            cfsetispeed(&Opt, speed_arr[i]);
            cfsetospeed(&Opt, speed_arr[i]);
            if ((status = tcsetattr(fd, TCSANOW, &Opt)) != 0)
            {
                perror("tcsetattr fd!");
                return -1;
            }
            tcflush(fd, TCIOFLUSH);
        }
    }
	MCULOG("SET SPEED OK!!!!!\n");
	
    return 0;
}

/*****************************************************************************
* Function Name : setUartParity
* Description   : 设置串奇偶性、数据位和停止位
* Input			: int fd,
*                 int databits,
*                 int stopbits,
*                 int parity
* Output        : None
* Return        : 0:success, -1:faild
* Auther        : ygg
* Date          : 2018.01.18
*****************************************************************************/
int mcuUart::setUartParity(int fd, int databits, int stopbits, int parity)
{
    struct termios options;
    if (tcgetattr(fd, &options) != 0)
    {
        perror("Setup Serial!");
        return -1;
    } 
	options.c_cflag &= ~CSIZE;

    switch (databits)
    {
        case 7:
            options.c_cflag |= CS7;
            break;
        case 8:
            options.c_cflag |= CS8;
            break;
        default:
            fprintf(stderr, "Unsupported data size.\n");
            return -1;
    }

    switch (parity)
    {
        case 'n':
        case 'N':
            options.c_cflag &= ~PARENB; /* Clear parity enable */
            options.c_iflag &= ~INPCK;  /* Enable parity checking */
            break;
        case 'o':
        case 'O':
            options.c_cflag |= (PARODD | PARENB);/* set to odd parity check */
            options.c_iflag |= INPCK;            /* Disnable parity checking */
            break;
        case 'e':
        case 'E':
            options.c_cflag |= PARENB;  /* Enable parity */
            options.c_cflag &= ~PARODD; /* convert to event parity check */
            options.c_iflag |= INPCK;   /* Disnable parity checking */
            break;
        case 's':
        case 'S':
            /* as no parity */
            options.c_cflag &= ~PARENB;
            options.c_cflag &= ~CSTOPB;
            break;
        default:
            fprintf(stderr, "Unsupported parity.\n");
            return -1;
    }

    switch (stopbits)
    {
        case 1:
            options.c_cflag &= ~CSTOPB;
            break;
        case 2:
            options.c_cflag |= CSTOPB;
            break;
        default:
            fprintf(stderr, "Unsupported stop bits.\n");
            return -1;
    }

    /* Set input parity option */
    if (parity != 'n')
        options.c_iflag |= INPCK;
    tcflush(fd, TCIFLUSH);
    options.c_cc[VTIME] = MCU_UART_TIMEOUT_MSECONDS; /* set timeout for 50 mseconds */
    options.c_cc[VMIN] = 0;
    options.c_cflag |= (CLOCAL | CREAD);

    /* Select the line input */
    options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    /* Select the line output */
    options.c_oflag &= ~OPOST;
    /* prevant 0d mandatory to 0a */
    options.c_iflag &= ~ICRNL;

    /* Cancellation of software flow control
     * options.c_iflag &=~(IXON | IXOFF | IXANY);
     */
    options.c_iflag &= ~IXON;

    if (tcsetattr(fd, TCSANOW, &options) != 0)
    {
        perror("Activate Serial failed.\n");
        return -1;
    }
	
    return 0;
}

/*****************************************************************************
* Function Name : mcuUartReceiveData
* Description   : 设置串奇偶性、数据位和停止位
* Input			: None
* Output        : None
* Return        : 0:success, -1:faild
* Auther        : ygg
* Date          : 2018.01.18
*****************************************************************************/
int mcuUart::mcuUartReceiveData(void)
{
    int nread;
    unsigned int len;

    unsigned char *p_mcuBuffer = (unsigned char*)malloc(BUFF_LEN);
	if(p_mcuBuffer == NULL)
	{
		MCULOG("malloc p_mcuBuffer failed.\n");
		return -1;
	}
    memset(p_mcuBuffer, 0, BUFF_LEN);

  //  MCULOG("start to read mcu data\n");

    len = checkMcuUartData(p_mcuBuffer, BUFF_LEN);
    if (len > 0)
    {
        nread = unpackMcuUartData(p_mcuBuffer, len);
        if (nread == -1)
        {
            MCULOG("unpack data failed!");
        }
    }
	if (p_mcuBuffer != NULL)
		memset(p_mcuBuffer, 0, BUFF_LEN);

	return 0;
}

/*
int mcuUart::registerCallback_reportDate(callBack_reportDate func)
{
	if (func == NULL)
		return -1;
	
	reportDataFunc = func;

	return 0;
}*/

/*****************************************************************************
* Function Name : checkMcuUartData
* Description   : 数据检查
* Input			: unsigned char *pData
*                 unsigned int size
* Output        : None
* Return        : retval:数据长度, -1:faild
* Auther        : ygg
* Date          : 2018.01.18
*****************************************************************************/
uint32_t mcuUart::checkMcuUartData(unsigned char *pData, unsigned int size)
{
	bool startToRead = false;
	unsigned char tempData = 0;
	unsigned int retval;
	unsigned char *pos = pData;
	unsigned char runState = 0;
	bool escapeCode = false;
	while (!startToRead)
	{
		if ((uint32_t)(pos-pData) >= size)
			break;
		if (read(fd, &tempData, 1) > 0)
		{
			//printf("%02x ",tempData);
			if (runState == 0)
			{
				if (tempData == 0x7e)
				{
					pos = pData;
					*pos = tempData;
					runState = 1;
				}
			}
			else if (runState == 1)
			{
				if (tempData == 0x7e)
				{
					if (pos == pData)
					{
						pos = pData;
						*pos = tempData;
						runState = 1;
					}
					else
					{
						*++pos = tempData;
						startToRead = true;
						break;
					}
				}
				else
				{
					if (escapeCode == true)
					{
						if (tempData == 0x01)
							*pos = 0x7D;
						else if (tempData == 0x02)
							*pos = 0x7E;
						else
							runState = 0;
						escapeCode = false;
					}
					else
					{
						if (tempData == 0x7d)
							escapeCode = true;
						*++pos = tempData;
					}
				}
			}
			else
			{
				runState = 0;
			}
		}
	}

	if ((startToRead == true) && (pos > pData + 1))
		retval = pos - pData + 1;
	else
		retval = 0;
	
	//MCULOG(" @@@@@@@@@@@@@@@@@@@@@@@@@ MCU received data! retval=%d", retval);
	//for(int i=0; i<(int)retval; i++)
		//MCU_NO("%02x ",*(pData+i));
	//MCU_NO("\n\n");
	
	return retval;
}

/*****************************************************************************
* Function Name : unpackMcuUartData
* Description   : 解包mcu数据
* Input			: uint8_t *pData
*                 unsigned int datalen
* Output        : None
* Return        : 0:success, -1:faild
* Auther        : ygg
* Date          : 2018.01.18
*****************************************************************************/
int mcuUart::unpackMcuUartData(uint8_t *pData, unsigned int datalen)
{
	//static int count = 0;
    uint16_t checkCrc;
	uint16_t serialNumber = 0;
    uint16_t bodylen = 0;
    uint8_t cmdId;

    if ((pData == NULL) || (datalen <= 4))
        return -1;

    bodylen = (pData[1] << 8) + pData[2];
    if ((uint32_t)(bodylen+11)!= datalen)
        return -1;

    checkCrc = (pData[1+7+bodylen] << 8) + pData[1+7+bodylen+1];
    if (checkCrc != Crc16Check(&pData[1], datalen-4))
    {
        MCULOG("check crc16 failed!\n");
        return -1;
    }
	else
	{
//		MCULOG("check crc16 success!\n");
	}

    cmdId = pData[3];
//	MCULOG("cmdId:%02x \n",cmdId);
	
    switch (cmdId)
    {
		case START_SYNC_CMD:
			unpack_syncParameter(pData, datalen);
			break;
		case HEART_BEAT_CMD:
		{
			serialNumber = (pData[4] << 8) + pData[5];
			packDataWithRespone(TBOX_REPORT_4G_STATE, 0x01, NULL, 0, serialNumber);
			//reportDataFunc(pData, datalen);
			unpack_updatePositionInfo(pData, datalen);
			
//			printf("====== GYL unpack_updateTimingInfo DATA Begin======\n");
//			MCULOG("====== GYL unpack_updateTimingInfo DATA Begin======\n");
			
			datalen -= 30;
			
			unpack_updateTimingInfo(pData + 30, datalen);
//			printf("====== GYL unpack_updateTimingInfo DATA end ======\n");
//			MCULOG("====== GYL unpack_updateTimingInfo DATA end ======\n");
		}
			break;
		case MCU_SND_MESSAGES_ID:
			unpack_text_messages(pData, datalen);
			break;
		case TEXT_TO_SPEECH_ID:
			unpack_tts_voice(pData, datalen);
			break;
		case MCU_SND_UPGRADE_INFO:
			unpack_MCU_SND_Upgrade_Info(pData, datalen);
			break;
		case MCU_SND_UPGRADE_DATA:
			unpack_MCU_SND_Upgrade_Data(pData, datalen);
			break;
		case MCU_SND_UPGRADE_CMPL:
			unpack_MCU_SND_Upgrade_CMPL();
			break;
		case TBOX_REMOTECTRL_CMD:	//0x04
			MCULOG();
			serialNumber = (pData[4] << 8) + pData[5];
			packProtocolData(TBOX_REMOTECTRL_CMD, 0x04, NULL, 0, serialNumber);
			unpack_RemoteCtrl(pData, datalen);
			break;
		case 0x06:
			mcu_apply_for_data(pData, datalen);
		default:
            MCULOG("cmdid error!\n");
            break;
    }
	
    return 0;
}

/*****************************************************************************
* Function Name : Crc16Check
* Description   : 数据检验
* Input			: uint8_t *pData
*                 uint32_t len
* Output        : None
* Return        : ui16Crc:校验码
* Auther        : ygg
* Date          : 2018.01.18
*****************************************************************************/
unsigned int mcuUart::Crc16Check(unsigned char* pData, uint32_t len)
{
	unsigned int ui16InitCrc = 0xffff;
	unsigned int ui16Crc = 0;
	unsigned int ui16i;
	unsigned char ui8j;
	unsigned char ui8ShiftBit;
	
	for(ui16i = 0;ui16i<len;ui16i++)
	{		
		ui16InitCrc ^= pData[ui16i];
		for(ui8j=0;ui8j<8;ui8j++)
		{
			ui8ShiftBit = ui16InitCrc&0x01;
			ui16InitCrc >>= 1;
			if(ui8ShiftBit != 0)
			{
				ui16InitCrc ^= 0xa001;
			}		
		}
	}
	
	ui16Crc = ui16InitCrc;
	return ui16Crc;
}

/*****************************************************************************
* Function Name : unpack_syncParameter
* Description   : 解包同步参数
* Input			: unsigned char *pData
*                 unsigned int len
* Output        : None
* Return        : 0:success, -1:faild
* Auther        : ygg
* Date          : 2018.01.18
*****************************************************************************/
int mcuUart::unpack_syncParameter(unsigned char *pData, unsigned int len)
{
	int i;
	unsigned short serialNumber;
	unsigned char attr[2];
	unsigned char subDataLen;
	unsigned char *pos = NULL;
	unsigned int tempData = 0;
	unsigned char u8Array[32];
	memset(u8Array,0,sizeof(u8Array));

	serialNumber = (pData[4] << 8) + pData[5];
	MCULOG("serialNumber = %d\n",serialNumber);

	memset(attr, 0, sizeof(attr));
	attr[0] = pData[6];
	attr[1] = pData[7];

	pos = pData;

	if(*(pos+8) == 0x01)
		//setSystemTime(pData);
	subDataLen = *(pos+8+1);
	pos = pos+8+1+subDataLen+1;

	if(*pos == 0x02)
		;
	subDataLen = *(pos+1);
	pos = pos+1+subDataLen+1;

	if(*pos == 0x03)
	{
		tempData = (*(pos+2)<<24) + (*(pos+3)<<16) + (*(pos+4)<<8) + (*(pos+5)<<0);
		tempData = tempData/3600;
		MCULOG("The lock car time show the terminal didn't connect to the server:%d\n",tempData);
		//dataPool->setPara(TBOX_DATA_NO_GPRS_SIGNAL_HOURS_INFO, &tempData, sizeof(tempData));	
	}
	subDataLen = *(pos+1);
	pos = pos+1+subDataLen+1;

	MCU_NO("*pos = %02x\n",*pos);
	if(*pos == 0x04)
	{
		;
	}
	subDataLen = *(pos+1);
	pos = pos+1+subDataLen+1;
	
	MCU_NO("*pos = %02x\n",*pos);
	if(*pos == 0x05)
	{
		;
	}
	subDataLen = *(pos+1);
	pos = pos+1+subDataLen+1;
	
	MCU_NO("*pos = %02x\n",*pos);
	if(*pos == 0x06)
	{
		;
	}
	subDataLen = *(pos+1);
	pos = pos+1+subDataLen+1;
	
	MCU_NO("*pos = %02x\n",*pos);
	if(*pos == 0x07)
	{
		;
	}
	subDataLen = *(pos+1);
	pos = pos+1+subDataLen+1;
	
	MCU_NO("*pos = %02x\n",*pos);
	if(*pos == 0x08)
	{
		tempData = (*(pos+2)<<24) + (*(pos+3)<<16) + (*(pos+4)<<8) + (*(pos+5)<<0);
		MCULOG("Can baud Rate:%d\n",tempData);
		//dataPool->setPara(TBOX_CANBAUDRATE_PARAID, &tempData, sizeof(tempData));
	}
	subDataLen = *(pos+1);
	pos = pos+1+subDataLen+1;
	
	MCU_NO("*pos = %02x\n",*pos);
	if(*pos == 0x09)
	{
		;	
	}
	subDataLen = *(pos+1);
	pos = pos+1+subDataLen+1;
	
	MCU_NO("*pos = %02x\n",*pos);
	if(*pos == 0x0A)
	{
		;	
	}
	subDataLen = *(pos+1);
	pos = pos+1+subDataLen+1;

	MCU_NO("*pos = %02x\n",*pos);
	if(*pos == 0x0B)
	{
		;	
	}
	subDataLen = *(pos+1);
	pos = pos+1+subDataLen+1;

	MCU_NO("*pos = %02x\n",*pos);
	if(*pos == 0x0C)
	{
		tempData = *(pos+2);
		//dataPool->setPara(TBOX_REMOTE_UPGRADE_PARAID, &tempData, sizeof(tempData));
	}
	subDataLen = *(pos+1);
	pos = pos+1+subDataLen+1;
	
	MCU_NO("*pos = %02x\n",*pos);
	if(*pos == 0x0D)
	{
		tempData = *(pos+2);
		//dataPool->setPara(TBOX_POWER_SAVING_MODE_PARAID, &tempData, sizeof(tempData));
	}
	subDataLen = *(pos+1);
	pos = pos+1+subDataLen+1;

	MCU_NO("*pos = %02x\n",*pos);
	if(*pos == 0x0E)
	{
		;	
	}
	subDataLen = *(pos+1);
	pos = pos+1+subDataLen+1;
	
	MCU_NO("*pos = %02x\n",*pos);
	if(*pos == 0x0F)
	{
		;	
	}
	subDataLen = *(pos+1);
	pos = pos+1+subDataLen+1;

	MCU_NO("*pos = %02x\n",*pos);
	if(*pos == 0x10)
	{
		;	
	}
	subDataLen = *(pos+1);
	pos = pos+1+subDataLen+1;
	
	MCU_NO("*pos = %02x\n",*pos);
	if(*pos == 0x11)
	{
		;	
	}
	subDataLen = *(pos+1);
	pos = pos+1+subDataLen+1;

	MCU_NO("*pos = %02x\n",*pos);
	if(*pos == 0x12 || *pos == 0x13)
	{
		tempData = (*(pos+2)<<8) + *(pos+3);
		//dataPool->setPara(TBOX_MAINPWR_LOW_THRES_PARAID, &tempData, sizeof(tempData));
		MCU_NO("tempDatas = %02x\n",tempData);
	}
	subDataLen = *(pos+1);
	pos = pos+1+subDataLen+1;
	
	MCU_NO("*pos = %02x\n",*pos);
	if(*pos == 0x14)
	{
		;	
	}
	subDataLen = *(pos+1);
	pos = pos+1+subDataLen+1;

	MCU_NO("*pos = %02x\n",*pos);
	if(*pos == 0x15)
	{
		tempData = (*(pos+2)<<24) + (*(pos+3)<<16) + (*(pos+4)<<8) + (*(pos+5)<<0);
		//dataPool->setPara(TBOX_SLEEP_REPORTPOSI_INTERVAL_PARAID, &tempData, sizeof(tempData));
		MCU_NO("back power interval = %d\n",tempData);
	}
	subDataLen = *(pos+1);
	pos = pos+1+subDataLen+1;

	MCU_NO("*pos = %02x\n",*pos);
	if(*pos == 0x16)
	{
		tempData = *(pos+2);
		//dataPool->setPara(TBOX_PLC_PWRUP_CHECKLEVEL_PARAID, &tempData, sizeof(tempData));	
	}
	subDataLen = *(pos+1);
	pos = pos+1+subDataLen+1;

	MCU_NO("*pos = %02x\n",*pos);
	if(*pos == 0x17)
	{
		;	
	}
	subDataLen = *(pos+1);
	pos = pos+1+subDataLen+1;

	MCU_NO("*pos = %02x\n",*pos);
	if(*pos == 0x18)
	{
		tempData = (*(pos+2)<<24) + (*(pos+3)<<16) + (*(pos+4)<<8) + (*(pos+5)<<0);
		//dataPool->setPara(TBOX_BACKPWRUP_REPORTPOSI_INTERVAL_PARAID, &tempData, sizeof(tempData));
		MCU_NO("back power interval = %d\n",tempData);
	}
	subDataLen = *(pos+1);
	pos = pos+1+subDataLen+1;

	MCU_NO("*pos = %02x\n",*pos);
	if(*pos == 0x19)
	{
		;	
	}
	subDataLen = *(pos+1);
	pos = pos+1+subDataLen+1;

	MCU_NO("*pos = %02x\n",*pos);
	if(*pos == 0x1A)
	{
		subDataLen = *(pos+1);
		memset(mcuVerNumber, 0, sizeof(mcuVerNumber));
		//MCULOG("Set mcu version & ver length:%d", subDataLen);
		//for (i = 0; i < subDataLen; i++)
		//	MCU_NO("%c",*(pos+1+1+i));
			//MCU_NO("%02x ",*(pos+1+1+i));
		//MCU_NO("\n\n");
		/*********************************************************************
		 *  mcu version example:V1.0.0.0_Build2017041714:00:00,
		 *  It need to get the version number form the mcu version's 2bit,
		 *  4bit,6bit and 8bit,then convert to number and stored it in u8Array
		 *  array.
		 *********************************************************************/
		mcuVerNumber[0] = *(pos+1+1+1)-'0';
		mcuVerNumber[1] = *(pos+1+1+3)-'0';
		mcuVerNumber[2] = *(pos+1+1+5)-'0';
		//mcuVerNumber[3] = *(pos+1+1+7)-'0';
		MCU_NO("%02x %02x %02x %02x ",mcuVerNumber[0],mcuVerNumber[1],mcuVerNumber[2]); //,u8Array[3]);
		//dataPool->setPara(TBOX_STA8090_VERSION_PARAID, u8Array, 4);
	}
	subDataLen = *(pos+1);
	pos = pos+1+subDataLen+1;
	
	MCU_NO("*pos = %02x\n",*pos);
	if(*pos == 0x1B)
	{
		subDataLen = *(pos+1);
		memset(u8Array, 0, sizeof(u8Array));
		MCULOG("Set plc version & ver length:%d", subDataLen);
		for (i = 0; i < subDataLen; i++)
			MCU_NO("%02x ",*(pos+1+1+i));
		MCU_NO("\n\n");
		memcpy(u8Array, pos+1+1, 20);
		//dataPool->setPara(TBOX_PLC_VERSION_PARAID, u8Array, subDataLen);
	}
	subDataLen = *(pos+1);
	pos = pos+1+subDataLen+1;

	MCU_NO("*pos = %02x\n",*pos);
	if(*pos == 0x1C)
	{
		;	
	}
	subDataLen = *(pos+1);
	pos = pos+1+subDataLen+1;

	MCU_NO("*pos = %02x\n",*pos);
	if(*pos == 0x1D)
	{
		;	
	}
	subDataLen = *(pos+1);
	pos = pos+1+subDataLen+1;

	MCU_NO("*pos = %02x\n",*pos);
	if(*pos == 0x1E)
	{
		tempData = (*(pos+2)<<24) + (*(pos+3)<<16) + (*(pos+4)<<8) + (*(pos+5)<<0);
		MCULOG("Can bus minimum filtering value:%d\n",tempData);
		//dataPool->setPara(TBOX_CAN_MINID_PARAID, &tempData, sizeof(tempData));
	}
	subDataLen = *(pos+1);
	pos = pos+1+subDataLen+1;

	MCU_NO("*pos = %02x\n",*pos);
	if(*pos == 0x1F)
	{
		tempData = (*(pos+2)<<24) + (*(pos+3)<<16) + (*(pos+4)<<8) + (*(pos+5)<<0);
		MCULOG("Can bus maximun filtering value:%d\n",tempData);
		//dataPool->setPara(TBOX_CAN_MAXID_PARAID, &tempData, sizeof(tempData));
	}
	subDataLen = *(pos+1);
	pos = pos+1+subDataLen+1;
	
	MCU_NO("*pos = %02x\n",*pos);
	if(*pos == 0x20)
	{
		//dataPool->getInfo(TBOX_DATA_EXT_STATE_INFO,&tempData,sizeof(tempData));
		if(*(pos+2) == 0)
			tempData &= (~(0x01 << 0));
		else if(*(pos+2) == 1)
			tempData &= (0x01 << 0);
		MCULOG("Lock car time, *(pos+2):%02x,temData:%d\n",*(pos+2),tempData);
		//dataPool->setInfo(TBOX_DATA_EXT_STATE_INFO,&tempData,sizeof(tempData));
	}
	subDataLen = *(pos+1);
	pos = pos+1+subDataLen+1;

	packProtocolData(TBOX_GENERAL_RESP, 0x00, NULL, 0, serialNumber);
	
	return 0;
}

/*****************************************************************************
* Function Name : unpack_updatePositionInfo
* Description   : 解包位置信息
* Input			: unsigned char *pData
*                 unsigned int len
* Output        : None
* Return        : 0:success, -1:faild
* Auther        : ygg
* Date          : 2018.01.18
*****************************************************************************/
int mcuUart::unpack_updatePositionInfo(unsigned char *pData, unsigned int len)
{
	struct tm _tm;
	uint32_t u32Status = (pData[8]<<24) + (pData[9]<<16) + (pData[10]<<8) +pData[11];

	//定位状态 positioning status
	if(u32Status & (0x01 <<18))
	{
		//MCULOG("GPS positioning successful!\n");
		//设置系统时间
		_tm.tm_year = pData[24]+100; /* 年份，其值等于实际年份减去1900 */
		//8090已处理月份为当月，不需+1
		_tm.tm_mon = pData[25];    /* 月份（从一月开始，0代表一月）-取值区间为[0,11] */
		_tm.tm_mday = pData[26];     /* 一个月中的日期 - 取值区间为[1,31] */
		_tm.tm_hour = pData[27];     /* 时 - 取值区间为[0,23] */
		_tm.tm_min = pData[28];      /* 分 - 取值区间为[0,59] */
		_tm.tm_sec = pData[29];      /* 秒 – 取值区间为[0,59] */

		setSystemTime(&_tm);
	}
	else
	{
		//MCULOG("GPS positioning failed!\n");
	}

	unpack_updateGBT32960PositionInfo(pData, len);
	unpack_updateFAWACPPositionInfo(pData, len);
	unpack_updateFAWACPVehCondInfo(pData, len);

	return 0;
}

/*****************************************************************************
* Function Name : setSystemTime
* Description   : 设置系统时间
* Input			: struct tm *pTm
* Output        : None
* Return        : 0:success, -1:faild
* Auther        : ygg
* Date          : 2018.01.18
*****************************************************************************/
int mcuUart::setSystemTime(struct tm *pTm)
{
    time_t timep;
    struct timeval tv;

    timep = mktime(pTm);
    tv.tv_sec = timep;
    tv.tv_usec = 0;
    if (settimeofday(&tv, (struct timezone*)0) < 0)
    {
        printf("Set system datatime error!\n");
        return -1;
    }

    return 0;
}

/*****************************************************************************
* Function Name : unpack_updateGBT32960PositionInfo
* Description   : 更新位置信息
* Input			: unsigned char *pData
*                 unsigned int len
* Output        : None
* Return        : 0:success, -1:faild
* Auther        : ygg
* Date          : 2018.01.18
*****************************************************************************/
int mcuUart::unpack_updateGBT32960PositionInfo(unsigned char *pData, unsigned int len)
{
	uint16_t i;
	
	//for (i = 0; i < len; i++)
	//	MCU_NO("%02x ",*(pData+i));
	//MCU_NO("\n\n");
	
	uint32_t u32PositionStatus = (pData[8]<<24) + (pData[9]<<16) + (pData[10]<<8) +pData[11];
//	MCULOG("u32PositionStatus:%u\n",u32PositionStatus);

	//定位状态 positioning status
	if(u32PositionStatus & (0x01 <<18)){
		//p_GBT32960_handle->dataInfo.positionInfo.positionValid.positionState &= (0<<0);
		p_GBT32960_handle->dataInfo.positionInfo.positionValid.bitState.valid = 0;
//		MCULOG("111 positionState:%d\n",p_GBT32960_handle->dataInfo.positionInfo.positionValid.positionState);
	}
	else{
		//p_GBT32960_handle->dataInfo.positionInfo.positionValid.positionState |= (1<<0);
		p_GBT32960_handle->dataInfo.positionInfo.positionValid.bitState.valid = 1;
//		MCULOG("222 positionState:%d\n",p_GBT32960_handle->dataInfo.positionInfo.positionValid.positionState);
	}
	
	//南纬或北纬 South latitude or north latitude
	if(u32PositionStatus & (0x01 <<0)){
		//p_GBT32960_handle->dataInfo.positionInfo.positionValid.positionState |= (1<<1);
		p_GBT32960_handle->dataInfo.positionInfo.positionValid.bitState.lat = 1;
//		MCULOG("333 positionState:%d\n",p_GBT32960_handle->dataInfo.positionInfo.positionValid.positionState);
	}
	else{
		//p_GBT32960_handle->dataInfo.positionInfo.positionValid.positionState &= (0<<1);
		p_GBT32960_handle->dataInfo.positionInfo.positionValid.bitState.lat = 0;
//		MCULOG("444 positionState:%d\n",p_GBT32960_handle->dataInfo.positionInfo.positionValid.positionState);
	}

	//东经或西经 East longitude or west longitude
	if(u32PositionStatus & (0x01 <<1)){
		//p_GBT32960_handle->dataInfo.positionInfo.positionValid.positionState |= (1<<2);
		p_GBT32960_handle->dataInfo.positionInfo.positionValid.bitState.lon = 1;
//		MCULOG("555 positionState:%d\n",p_GBT32960_handle->dataInfo.positionInfo.positionValid.positionState);
	}
	else{
		//p_GBT32960_handle->dataInfo.positionInfo.positionValid.positionState &= (0<<2);
		p_GBT32960_handle->dataInfo.positionInfo.positionValid.bitState.lon = 0;
//		MCULOG("666 positionState:%d\n",p_GBT32960_handle->dataInfo.positionInfo.positionValid.positionState);
	}

	/****************************************************************
	* 定位状态说明:
	* pData[12]: 纬度
	* pData[13]: 纬度分
	* ((pData[14] << 8) + pData[15]): 纬度分小数
	* pData[16]: 经度
	* pData[17]: 经度分
	* ((pData[18] << 8) + pData[19]): 经度分小数
	*****************************************************************/
	p_GBT32960_handle->dataInfo.positionInfo.Lat = (uint32_t)pData[12]*1000000+
												(((float)pData[13]+((float)((pData[14]<<8)+pData[15])/100000))/60)*1000000;

	p_GBT32960_handle->dataInfo.positionInfo.Lon = (uint32_t)pData[16]*1000000+
												(((float)pData[17]+((float)((pData[18]<<8)+pData[19])/100000))/60)*1000000;

//	MCULOG("Lat:%lu\n",p_GBT32960_handle->dataInfo.positionInfo.Lat);
//	MCULOG("Lon:%lu\n",p_GBT32960_handle->dataInfo.positionInfo.Lon);

	return 0;
}

int mcuUart::unpack_updateFAWACPPositionInfo(unsigned char *pData, unsigned int len)
{
	uint16_t i;

	//for (i = 0; i < len; i++)
	//	MCU_NO("%02x ",*(pData+i));
	//MCU_NO("\n\n");

	uint32_t u32PositionStatus = (pData[8]<<24) + (pData[9]<<16) + (pData[10]<<8) +pData[11];
//	MCULOG("u32PositionStatus:%u\n",u32PositionStatus);

	//定位状态 positioning status
	if(u32PositionStatus & (0x01 <<18)){
		p_FAWACPInfo_Handle->VehicleCondData.GPSData.GPSState = 0;
	//	MCULOG("111 positionState:%d\n",p_FAWACPInfo_Handle->VehicleCondData.GPSData.GPSState);
	}
	else{
		p_FAWACPInfo_Handle->VehicleCondData.GPSData.GPSState = 1;
	//	MCULOG("222 positionState:%d\n",p_FAWACPInfo_Handle->VehicleCondData.GPSData.GPSState);
	}

	//南纬或北纬 South latitude or north latitude
	if(u32PositionStatus & (0x01 <<0)){
		p_FAWACPInfo_Handle->VehicleCondData.GPSData.latitudeState = 1;
	//	MCULOG("333 positionState:%d\n",p_FAWACPInfo_Handle->VehicleCondData.GPSData.latitudeState);
	}
	else{
		p_FAWACPInfo_Handle->VehicleCondData.GPSData.latitudeState = 0;
	//	MCULOG("444 positionState:%d\n",p_FAWACPInfo_Handle->VehicleCondData.GPSData.latitudeState);
	}

	//东经或西经 East longitude or west longitude
	if(u32PositionStatus & (0x01 <<1)){
		p_FAWACPInfo_Handle->VehicleCondData.GPSData.longitudeState = 1;
	//	MCULOG("555 positionState:%d\n",p_FAWACPInfo_Handle->VehicleCondData.GPSData.longitudeState);
	}
	else{
		p_FAWACPInfo_Handle->VehicleCondData.GPSData.longitudeState = 0;
	//	MCULOG("666 positionState:%d\n",p_FAWACPInfo_Handle->VehicleCondData.GPSData.longitudeState);
	}

	/****************************************************************
	* 定位状态说明:
	* pData[12]: 纬度
	* pData[13]: 纬度分
	* ((pData[14] << 8) + pData[15]): 纬度分小数
	* pData[16]: 经度
	* pData[17]: 经度分
	* ((pData[18] << 8) + pData[19]): 经度分小数
	*****************************************************************/
	p_FAWACPInfo_Handle->VehicleCondData.GPSData.degree = (pData[20] << 8) + pData[21];

	p_FAWACPInfo_Handle->VehicleCondData.CurrentSpeed = (pData[22] << 8) + pData[23];
	
//	MCULOG("Degree:%lu\n",p_FAWACPInfo_Handle->VehicleCondData.GPSData.degree);
//	MCULOG("CurrentSpeed:%lu\n",p_FAWACPInfo_Handle->VehicleCondData.CurrentSpeed);

	p_FAWACPInfo_Handle->VehicleCondData.GPSData.latitude = (uint32_t)pData[12]*1000000+
												(((float)pData[13]+((float)((pData[14]<<8)+pData[15])/100000))/60)*1000000;

	p_FAWACPInfo_Handle->VehicleCondData.GPSData.longitude = (uint32_t)pData[16]*1000000+
												(((float)pData[17]+((float)((pData[18]<<8)+pData[19])/100000))/60)*1000000;

	//MCULOG("Lat:%lu\n",p_FAWACPInfo_Handle->VehicleCondData.GPSData.latitude);
	//MCULOG("Lon:%lu\n",p_FAWACPInfo_Handle->VehicleCondData.GPSData.longitude);
	
	p_FAWACPInfo_Handle->VehicleCondData.GPSData.year = pData[24]-10;
	p_FAWACPInfo_Handle->VehicleCondData.GPSData.month = pData[25];
	p_FAWACPInfo_Handle->VehicleCondData.GPSData.day = pData[26];
	p_FAWACPInfo_Handle->VehicleCondData.GPSData.hour = pData[27];
	p_FAWACPInfo_Handle->VehicleCondData.GPSData.minute = pData[28];
	p_FAWACPInfo_Handle->VehicleCondData.GPSData.second = pData[29];

	return 0;
}

int mcuUart::unpack_updateFAWACPVehCondInfo(unsigned char* pData, unsigned int len)
{
	p_FAWACPInfo_Handle->VehicleCondData.RemainedOil = pData[30];//剩余油量
	p_FAWACPInfo_Handle->VehicleCondData.Odometer = (pData[31] << 24) + (pData[32] << 16) + (pData[33] << 8) + pData[34];//总里程Last Value
	p_FAWACPInfo_Handle->VehicleCondData.Battery = ((pData[35] << 8) & 0xFF00) | (pData[36] & 0xFF);//蓄电池电量
	p_FAWACPInfo_Handle->VehicleCondData.LTAverageOil = ((pData[37] << 8) & 0xFF00) | (pData[38] & 0xFF);//长时平均油耗
	p_FAWACPInfo_Handle->VehicleCondData.STAverageOil = ((pData[39] << 8) & 0xFF00) | (pData[40] & 0xFF);//短时平均油耗
	
//	MCULOG("RemainedOil == %d\n",p_FAWACPInfo_Handle->VehicleCondData.RemainedOil);
//	MCULOG("Odometer == %d\n",p_FAWACPInfo_Handle->VehicleCondData.Odometer);
//	MCULOG("Battery == %d\n",p_FAWACPInfo_Handle->VehicleCondData.Battery);
//	MCULOG("LTAverageOil == %d\n",p_FAWACPInfo_Handle->VehicleCondData.LTAverageOil);
//	MCULOG("STAverageOil == %d\n",p_FAWACPInfo_Handle->VehicleCondData.STAverageOil);
	//车门状态
	p_FAWACPInfo_Handle->VehicleCondData.CarDoorState.bitState.drivingDoor = pData[41] & 0x01;
	p_FAWACPInfo_Handle->VehicleCondData.CarDoorState.bitState.copilotDoor = (pData[41] >> 1) & 0x01;
	p_FAWACPInfo_Handle->VehicleCondData.CarDoorState.bitState.leftRearDoor = (pData[41] >> 2) & 0x01;
	p_FAWACPInfo_Handle->VehicleCondData.CarDoorState.bitState.rightRearDoor = (pData[41] >> 3) & 0x01;
	p_FAWACPInfo_Handle->VehicleCondData.CarDoorState.bitState.rearCanopy = (pData[41] >> 4) & 0x01;
	p_FAWACPInfo_Handle->VehicleCondData.CarDoorState.bitState.engineCover = (pData[41] >> 5) & 0x01;
	
//	MCULOG("drivingDoor == %d\n",p_FAWACPInfo_Handle->VehicleCondData.CarDoorState.bitState.drivingDoor);
//	MCULOG("copilotDoor == %d\n",p_FAWACPInfo_Handle->VehicleCondData.CarDoorState.bitState.copilotDoor);
//	MCULOG("leftRearDoor == %d\n",p_FAWACPInfo_Handle->VehicleCondData.CarDoorState.bitState.leftRearDoor);
//	MCULOG("rightRearDoor == %d\n",p_FAWACPInfo_Handle->VehicleCondData.CarDoorState.bitState.rightRearDoor);
//	MCULOG("rearCanopy == %d\n",p_FAWACPInfo_Handle->VehicleCondData.CarDoorState.bitState.rearCanopy);
//	MCULOG("engineCover == %d\n",p_FAWACPInfo_Handle->VehicleCondData.CarDoorState.bitState.engineCover);
	//车锁状态
	p_FAWACPInfo_Handle->VehicleCondData.CarLockState.bitState.rightRearLock = pData[42] & 0x03;
	p_FAWACPInfo_Handle->VehicleCondData.CarLockState.bitState.leftRearLock = (pData[42] >> 2) & 0x03;
	p_FAWACPInfo_Handle->VehicleCondData.CarLockState.bitState.copilotLock = (pData[42] >> 4) & 0x03;
	p_FAWACPInfo_Handle->VehicleCondData.CarLockState.bitState.drivingLock = (pData[42] >> 6) & 0x03;
	
//	MCULOG("rightRearLock == %d\n",p_FAWACPInfo_Handle->VehicleCondData.CarLockState.bitState.rightRearLock);
//	MCULOG("leftRearLock == %d\n",p_FAWACPInfo_Handle->VehicleCondData.CarLockState.bitState.leftRearLock);
//	MCULOG("copilotLock == %d\n",p_FAWACPInfo_Handle->VehicleCondData.CarLockState.bitState.copilotLock);
//	MCULOG("drivingLock == %d\n",p_FAWACPInfo_Handle->VehicleCondData.CarLockState.bitState.drivingLock);
	//天窗状态
	p_FAWACPInfo_Handle->VehicleCondData.sunroofState = pData[43];
	
//	MCULOG("sunroofState == %d\n",p_FAWACPInfo_Handle->VehicleCondData.sunroofState);
	//车窗状态数据
	p_FAWACPInfo_Handle->VehicleCondData.WindowState.bitState.leftFrontWindow = pData[45] & 0x07;
	p_FAWACPInfo_Handle->VehicleCondData.WindowState.bitState.rightFrontWindow = (pData[45] >> 3) & 0x07;
	p_FAWACPInfo_Handle->VehicleCondData.WindowState.bitState.leftRearWindow = ((pData[45] >> 6) & 0x07) + ((pData[44] << 2) & 0x04);
	p_FAWACPInfo_Handle->VehicleCondData.WindowState.bitState.rightRearWindow = ((pData[44] >> 1) & 0x07);
	
//	MCULOG("leftFrontWindow == %d\n",p_FAWACPInfo_Handle->VehicleCondData.WindowState.bitState.leftFrontWindow );
//	MCULOG("rightFrontWindow == %d\n",p_FAWACPInfo_Handle->VehicleCondData.WindowState.bitState.rightFrontWindow );
//	MCULOG("leftRearWindow == %d\n",p_FAWACPInfo_Handle->VehicleCondData.WindowState.bitState.leftRearWindow );
//	MCULOG("rightRearWindow == %d\n",p_FAWACPInfo_Handle->VehicleCondData.WindowState.bitState.rightRearWindow );
	//车灯状态数据
	p_FAWACPInfo_Handle->VehicleCondData.CarlampState.bitState.headlights = pData[46] & 0x01;
	p_FAWACPInfo_Handle->VehicleCondData.CarlampState.bitState.positionlights = (pData[46] >> 1) & 0x01;
	p_FAWACPInfo_Handle->VehicleCondData.CarlampState.bitState.nearlights = (pData[46] >> 2) & 0x01;
	p_FAWACPInfo_Handle->VehicleCondData.CarlampState.bitState.rearfoglights = (pData[46] >> 3) & 0x01;
	p_FAWACPInfo_Handle->VehicleCondData.CarlampState.bitState.frontfoglights = (pData[46] >> 4) & 0x01;
	
//	MCULOG("headlights == %d\n",p_FAWACPInfo_Handle->VehicleCondData.CarlampState.bitState.headlights );
//	MCULOG("positionlights == %d\n",p_FAWACPInfo_Handle->VehicleCondData.CarlampState.bitState.positionlights );
//	MCULOG("nearlights == %d\n",p_FAWACPInfo_Handle->VehicleCondData.CarlampState.bitState.nearlights );
//	MCULOG("rearfoglights == %d\n",p_FAWACPInfo_Handle->VehicleCondData.CarlampState.bitState.rearfoglights );
//	MCULOG("frontfoglights == %d\n",p_FAWACPInfo_Handle->VehicleCondData.CarlampState.bitState.frontfoglights );
	//轮胎信息数据
	p_FAWACPInfo_Handle->VehicleCondData.TyreState.rightrearTyrePress = pData[54];
	p_FAWACPInfo_Handle->VehicleCondData.TyreState.leftrearTyrePress = pData[53];
	p_FAWACPInfo_Handle->VehicleCondData.TyreState.rightfrontTyrePress = pData[52];
	p_FAWACPInfo_Handle->VehicleCondData.TyreState.leftfrontTyrePress = pData[51];
	p_FAWACPInfo_Handle->VehicleCondData.TyreState.rightrearTemperature = pData[50];
	p_FAWACPInfo_Handle->VehicleCondData.TyreState.leftrearTemperature = pData[49];
	p_FAWACPInfo_Handle->VehicleCondData.TyreState.rightfrontTemperature = pData[48];
	p_FAWACPInfo_Handle->VehicleCondData.TyreState.leftfrontTemperature = pData[47];
	//TBox_MCU版本
	memcpy(p_FAWACPInfo_Handle->VehicleCondData.VerTboxMCU, &pData[55], 12);
	//发动机状态
	p_FAWACPInfo_Handle->VehicleCondData.EngineState = pData[67] & 0x07;
	//实时方向盘转角数据
	p_FAWACPInfo_Handle->VehicleCondData.WheelState.bitState.wheeldegree = ((pData[68] & 0x7F) << 8) + pData[69];
	p_FAWACPInfo_Handle->VehicleCondData.WheelState.bitState.wheeldirection = (pData[68] >> 7) & 0x01;
	//发动机转速
	p_FAWACPInfo_Handle->VehicleCondData.EngineSpeed = (pData[70] << 8) + pData[71];
	//档位信息
	p_FAWACPInfo_Handle->VehicleCondData.Gearstate = pData[72] & 0x0F;
	//手刹状态
	p_FAWACPInfo_Handle->VehicleCondData.HandbrakeState = pData[73] & 0x03;
	//驻车状态
	p_FAWACPInfo_Handle->VehicleCondData.ParkingState = pData[74] & 0x03;
	//安全带状态
	p_FAWACPInfo_Handle->VehicleCondData.Safetybeltstate = pData[75] & 0x03;
	//剩余保养里程
	p_FAWACPInfo_Handle->VehicleCondData.RemainUpkeepMileage = (pData[76] << 8) + pData[77];
	//空调相关信息
	p_FAWACPInfo_Handle->VehicleCondData.AirconditionerInfo.airconditionerState = pData[80] & 0x01;
	p_FAWACPInfo_Handle->VehicleCondData.AirconditionerInfo.compressorState = (pData[80] >> 1) & 0x01;
	p_FAWACPInfo_Handle->VehicleCondData.AirconditionerInfo.autoState = (pData[80] >> 2) & 0x01;
	p_FAWACPInfo_Handle->VehicleCondData.AirconditionerInfo.defrostState = (pData[80] >> 3) & 0x01;
	p_FAWACPInfo_Handle->VehicleCondData.AirconditionerInfo.inOutCirculateState = (pData[80] >> 4) & 0x01;
	p_FAWACPInfo_Handle->VehicleCondData.AirconditionerInfo.blowingLevel = (pData[80] >> 5) & 0x07;
	p_FAWACPInfo_Handle->VehicleCondData.AirconditionerInfo.blowingMode = pData[79] & 0x07;
	p_FAWACPInfo_Handle->VehicleCondData.AirconditionerInfo.Temperature = ((pData[79] >> 3) & 0x1F) + ((pData[78] & 0x03) << 5);
	//持续时间信息
	p_FAWACPInfo_Handle->VehicleCondData.KeepingstateTime.accelerateTime = pData[84];
	p_FAWACPInfo_Handle->VehicleCondData.KeepingstateTime.decelerateTime = pData[83];
	p_FAWACPInfo_Handle->VehicleCondData.KeepingstateTime.wheelTime = pData[82];
	p_FAWACPInfo_Handle->VehicleCondData.KeepingstateTime.overspeedTime = pData[81];
	
	//动力电池
	/*预留*/

	//充电状态数据
	p_FAWACPInfo_Handle->VehicleCondData.ChargeState.chargeState = pData[86];
	p_FAWACPInfo_Handle->VehicleCondData.ChargeState.remainChargeTime = pData[85];
	
	//TBOX-OS版本
	/*预留*/

	//故障信息
	p_FAWACPInfo_Handle->AcpCodeFault.ACPCODEFAULTEMSState = pData[87];		//发动机管理系统故障
	p_FAWACPInfo_Handle->AcpCodeFault.ACPCODEFAULTTCUState = pData[88];		//变速箱控制单元故障
	p_FAWACPInfo_Handle->AcpCodeFault.ACPCODEFAULTSRSState = pData[89];		//安全气囊系统故障
	p_FAWACPInfo_Handle->AcpCodeFault.ACPCODEFAULTESPState = pData[90];		//电子稳定系统故障
	p_FAWACPInfo_Handle->AcpCodeFault.ACPCODEFAULTABSState = pData[91];		//防抱死刹车系统
	p_FAWACPInfo_Handle->AcpCodeFault.ACPCODEFAULTEPASState = pData[92];		//电子助力转向系统
	p_FAWACPInfo_Handle->AcpCodeFault.ACPCODEFAULTOilPressureState = pData[93];	//机油压力报警
	p_FAWACPInfo_Handle->AcpCodeFault.ACPCODEFAULTLowOilIDState = pData[94];	//油量低报警
	p_FAWACPInfo_Handle->AcpCodeFault.ACPCODEFAULTBrakeFluidLevelState = pData[95];	//制动液位报警
	p_FAWACPInfo_Handle->AcpCodeFault.ACPCODEFAULTBBWState = pData[96];		//制动系统故障
	p_FAWACPInfo_Handle->AcpCodeFault.ACPCODEFAULTTPMSState = pData[97];		//胎压系统故障
	p_FAWACPInfo_Handle->AcpCodeFault.ACPCODEFAULTESCLState = pData[98];		//电子转向柱锁故障
	p_FAWACPInfo_Handle->AcpCodeFault.ACPCODEFAULTElecParkUnitState = pData[99];	//电子驻车系统故障
	p_FAWACPInfo_Handle->AcpCodeFault.ACPCODEFAULTBatteryChargeState = pData[102];	//蓄电池充电故障
	p_FAWACPInfo_Handle->AcpCodeFault.ACPCODEFAULTEmissionState = pData[103];	//排放系统故障
	p_FAWACPInfo_Handle->AcpCodeFault.ACPCODEFAULTSTTState = pData[104];		//启停系统故障
	p_FAWACPInfo_Handle->AcpCodeFault.ACPCODEFAULTACCState = pData[105];		//自适应巡航系统故障
	p_FAWACPInfo_Handle->AcpCodeFault.ACPCODEFAULTEngineOverwaterState = pData[106];//发动机水温过高报警
	//硬线或总线碰撞报警	
	SigEvent.s_VehCollideAlarm = pData[100];
printf("####################  SigEvent.s_VehCollideAlarm #################### :%d\n",SigEvent.s_VehCollideAlarm);
	if(SigEvent.s_VehCollideAlarm == 1)
	{	
		struct tm *p_tm = NULL;
		time_t tmp_time;
		tmp_time = time(NULL);
		p_tm = gmtime(&tmp_time);
		printf(">>>>>>>>>>>>>> s_VehCollideAlarm == 1 >>>>>>>>>>>>>>\n");
		printf(" %0d-%d-%d-%d-%d-%d\n",p_tm->tm_year+1900, p_tm->tm_mon, p_tm->tm_mday,p_tm->tm_hour,p_tm->tm_min,p_tm->tm_sec);
	}
	else
	{
		printf(">>>>>>>>>>>>>> s_VehCollideAlarm == 0 >>>>>>>>>>>>>>\n");
	}
	//车身防盗报警
	SigEvent.s_DoorIntrusAlarm = pData[101];
	//低油量报警
	SigEvent.s_LowOilAlarm = p_FAWACPInfo_Handle->AcpCodeFault.ACPCODEFAULTLowOilIDState;
	//定时熄火信号
	SigEvent.s_TimeOutFileAlarm = pData[107];
	//紧急熄火信号
	SigEvent.s_VehOutFileAlarm = pData[108];
	//高度
	p_FAWACPInfo_Handle->VehicleCondData.GPSData.altitude = (pData[109] << 8) + pData[110];
	//制动踏板开关（天津一汽无）
	
	//环境温度（天津一汽无）

	//空调人为操作
	p_FAWACPInfo_Handle->AcpCodeFault.ACPCODEFAULTAirconManualState = pData[114];

	//长时平均速度
	p_FAWACPInfo_Handle->VehicleCondData.LTAverageSpeed = (pData[115] << 8) + pData[116];
	//短时平均速度
	p_FAWACPInfo_Handle->VehicleCondData.STAverageSpeed = (pData[117] << 8) + pData[118];
}

/*****************************************************************************
* Function Name : unpack_fault_info
* Description   : 解包故障信息
* Input			: unsigned char *pData
*                 unsigned int len
* Output        : None
* Return        : 0:success
* Auther        : ygg
* Date          : 2018.04.27
*****************************************************************************/
int mcuUart::unpack_fault_info(unsigned char* pData, unsigned int len)
{
	MCULOG("set fault ========================\n");
/*	p_FAWACPInfo_Handle->AcpCodeFault.ACPCODEFAULTEMSState = pData[8];		//发动机管理系统故障
	p_FAWACPInfo_Handle->AcpCodeFault.ACPCODEFAULTTCUState = pData[9];		//变速箱控制单元故障
	p_FAWACPInfo_Handle->AcpCodeFault.ACPCODEFAULTEmissionState = pData[10]; //排放系统故障
	p_FAWACPInfo_Handle->AcpCodeFault.ACPCODEFAULTSRSState = pData[11];		//安全气囊系统故障
	p_FAWACPInfo_Handle->AcpCodeFault.ACPCODEFAULTESPState = pData[12];		//电子稳定系统故障
	p_FAWACPInfo_Handle->AcpCodeFault.ACPCODEFAULTABSState = pData[13];		//防抱死刹车系统
	p_FAWACPInfo_Handle->AcpCodeFault.ACPCODEFAULTEPASState = pData[14];		//电子助力转向系统
	p_FAWACPInfo_Handle->AcpCodeFault.ACPCODEFAULTOilPressureState = pData[15];	//机油压力报警
	p_FAWACPInfo_Handle->AcpCodeFault.ACPCODEFAULTLowOilIDState = pData[16];	//油量低报警
	p_FAWACPInfo_Handle->AcpCodeFault.ACPCODEFAULTBrakeFluidLevelState = pData[17];	//制动液位报警
	//p_FAWACPInfo_Handle->AcpCodeFault.ACPCODEFAULTBatteryChargeState  :2; //蓄电池充电故障
	p_FAWACPInfo_Handle->AcpCodeFault.ACPCODEFAULTBBWState = pData[18];		//制动系统故障
	p_FAWACPInfo_Handle->AcpCodeFault.ACPCODEFAULTTPMSState = pData[19];		//胎压系统故障
	p_FAWACPInfo_Handle->AcpCodeFault.ACPCODEFAULTESCLState = pData[20];		//电子转向柱锁故障
	p_FAWACPInfo_Handle->AcpCodeFault.ACPCODEFAULTElecParkUnitState = pData[99];	//电子驻车系统故障
	p_FAWACPInfo_Handle->AcpCodeFault.ACPCODEFAULTBatteryChargeState = pData[102];	//蓄电池充电故障
	p_FAWACPInfo_Handle->AcpCodeFault.ACPCODEFAULTEmissionState = pData[103];	//排放系统故障
	p_FAWACPInfo_Handle->AcpCodeFault.ACPCODEFAULTSTTState = pData[104];		//启停系统故障
	p_FAWACPInfo_Handle->AcpCodeFault.ACPCODEFAULTACCState = pData[105];		//自适应巡航系统故障
	p_FAWACPInfo_Handle->AcpCodeFault.ACPCODEFAULTEngineOverwaterState = pData[106];//发动机水温过高报警

*/

	MCULOG("set fault ========================\n");

	return 0;
}

/*****************************************************************************
* Function Name : unpack_text_messages
* Description   : 短信内容
* Input			: unsigned char *pData
*                 unsigned int len
* Output        : None
* Return        : 0:success
* Auther        : ygg
* Date          : 2018.01.18
*****************************************************************************/
int mcuUart::unpack_text_messages(unsigned char* pData, unsigned int len)
{
	unsigned char *pos = pData;
	uint8_t retval;
	unsigned char phoneNumberLen;
	unsigned char messageLen;
	char phoneNumber[CONTENT_MAX_LEN];
	char message[CONTENT_MAX_LEN];
	
	memset(phoneNumber, 0, CONTENT_MAX_LEN);
	memset(message, 0, CONTENT_MAX_LEN);

	phoneNumberLen = *(pos+8);
	MCULOG("phoneNumber length:%02x\n", phoneNumberLen);
	memcpy(phoneNumber, (pos+8+1), phoneNumberLen);
	MCULOG("phoneNumber:%s\n", phoneNumber);

	messageLen = *(pos+8+1+phoneNumberLen);
	MCULOG("message length:%02x\n", messageLen);
	memcpy(message, (pos+8+1+phoneNumberLen+1), messageLen);
	MCULOG("message content:%s\n", message);

	if(phoneNumberLen > 0 && messageLen > 0)
	{
		retval = 0;
		packDataWithRespone(TBOX_REPLY_MESSAGES_ID, 0, &retval, 1, 0);
	}
	else
	{
		retval = 1;
		packDataWithRespone(TBOX_REPLY_MESSAGES_ID, 0, &retval, 1, 0);
	}
	
	return 0;
}

/*****************************************************************************
* Function Name : unpack_text_messages
* Description   : tts语音内容
* Input			: unsigned char *pData
*                 unsigned int len
* Output        : None
* Return        : 0:success
* Auther        : ygg
* Date          : 2018.01.18
*****************************************************************************/
int mcuUart::unpack_tts_voice(unsigned char* pData, unsigned int len)
{
	unsigned char *pos = pData;
	unsigned char ttsLen;
	char ttsContent[CONTENT_MAX_LEN];
	memset(ttsContent, 0, CONTENT_MAX_LEN);

	ttsLen = *(pos+8);
	MCULOG("tts length:%02x\n", ttsLen);

	memcpy(ttsContent, (pos+8+1), ttsLen);
	MCULOG("ttsContent:%s\n", ttsContent);

	LteAtCtrl->audioPlayTTS(ttsContent);

	return 0;
}

/*****************************************************************************
* Function Name : packDataWithRespone
* Description   : 应答函数
* Input			: uint8_t responeCmd
*                 uint8_t subCmd
*                 uint8_t *needToPackData
*                 uint16_t len
* Output        : None
* Return        : 0:success
* Auther        : ygg
* Date          : 2018.01.18
*****************************************************************************/
int mcuUart::packDataWithRespone(uint8_t responeCmd, uint8_t subCmd, uint8_t *needToPackData, uint16_t len, uint16_t serialNumber)
{
	int i,headLen;
	int totalDataLen;
	unsigned char *pos = NULL;
	unsigned short int dataLen;
	unsigned int checkCode;
	
	static unsigned short int serialNO = 0;
	unsigned char *pBuff = (unsigned char *)malloc(BUFF_LEN);
	if(pBuff == NULL)
		return -1;
    memset(pBuff, 0, BUFF_LEN);
	
	pos = pBuff;
	
	*pos++ = 0x7e;
	//data length
	*pos++ = 0;
	*pos++ = 0;
	//cmd
	*pos++ = responeCmd;
	//serial NO.
	*pos++ = (serialNO & 0xff00) >> 8;
	*pos++ = (serialNO & 0xff) >> 0;

	//data property: the data which be send has been encrypted
	*pos++ = 0;
	*pos++ = 0;
	*pos++ = (serialNumber & 0xff00) >> 8;
	*pos++ = (serialNumber & 0xff) >> 0;

	headLen = pos-pBuff;
	MCULOG("headLen = %d \t subCmd:%02x\n", headLen, subCmd);

	switch (responeCmd)
	{
		case TBOX_REPORT_4G_STATE:
			dataLen = pack_report4GState(pos, BUFF_LEN-headLen);
			
			break;
		case TBOX_REPLY_MESSAGES_ID:
			dataLen = pack_successMark(pos, needToPackData, len);

			break;
		default:
			MCULOG("The cmd error!\n");
			break;
	}

	//MCULOG("dataLen = %d\n",dataLen);
	if(subCmd == 0x01)
	{
	pBuff[1] = (dataLen & 0xff00)>>8;
	pBuff[2] = (dataLen & 0xff)>>0;
	}
	pos += dataLen;
	
	//Calculated check code
	checkCode = Crc16Check(&pBuff[1], pos-pBuff-1);
	MCULOG("checkCode = %0x\n", checkCode);
	
	*pos++ = (checkCode & 0xff00)>>8;
	*pos++ = (checkCode & 0xff)>>0;

	*pos++ = 0x7e;

	totalDataLen = pos-pBuff;
	MCULOG("totalDataLen =%d", totalDataLen);

	MCULOG("\n\n\nBefore escape Data:");
	for(i = 0; i < totalDataLen; i++)
		MCU_NO("%02x ", *(pBuff+i));
	MCU_NO("\n");

	escape_mcuUart_data(pBuff, totalDataLen);

	serialNO++;
	if (serialNO > 10000)
		serialNO = 0;

	if (pBuff != NULL)
	{
		free(pBuff);
		pBuff = NULL;
	}

    return 0;
}

/*****************************************************************************
* Function Name : packProtocolData
* Description   : 打包协议函数
* Input			: uint8_t responeCmd
*                 uint8_t subCmd
*                 uint8_t *needToPackData
*                 uint16_t len
*                 uint16_t serialNum
* Output        : None
* Return        : 0:success
* Auther        : ygg
* Date          : 2018.01.18
*****************************************************************************/
int mcuUart::packProtocolData(uint8_t responeCmd, uint8_t subCmd, uint8_t *needToPackData, uint16_t len, uint16_t serialNum)
{
	int i, totalDataLen;
	unsigned char headLen;
	unsigned char *pos = NULL;
	unsigned short int dataLen;
	unsigned int checkCode;
	unsigned short int attribution = 0;
	static unsigned short int serialNO = 0;
	
	unsigned char *pData = (unsigned char *)malloc(BUFF_LEN);
	if(pData == NULL)
		return -1;
	memset(pData, 0, BUFF_LEN);

	pos = pData;
	
	*pos++ = 0x7e;
	//data length
	*pos++ = 0;
	*pos++ = 0;
	//cmd
	*pos++ = responeCmd;
	//serial NO.
	*pos++ = (serialNO & 0xff00) >> 8;
	*pos++ = (serialNO & 0xff) >> 0;

	//data property: the data which be send has been encrypted
	*pos++ = 0;
	*pos++ = 0;

	headLen = pos-pData;
	MCULOG("headLen = %d\n", headLen);

	switch (responeCmd)
	{
		case TBOX_GENERAL_RESP:
			//Response command
			*pos++ = subCmd;
			//Response serial number
			*pos++ = (serialNum & 0xff00) >> 8;
			*pos++ = (serialNum & 0xff) >> 0;
			*pos++ = 0x00;
			dataLen = pos-pData-headLen;
			break;
		case TBOX_REPLY_ID:
			*pos++ = *needToPackData;
			dataLen = pos-pData-headLen;
		default:
			break;
	}
	
	MCULOG("dataLen = %d\n",dataLen);
	pData[1] = (dataLen & 0xff00)>>8;
	pData[2] = (dataLen & 0xff)>>0;

	//pos += dataLen;
	
	//Calculated check code
	checkCode = Crc16Check(&pData[1], pos-pData-1);
	MCULOG("checkCode = %0x\n", checkCode);
	
	*pos++ = (checkCode & 0xff00)>>8;
	*pos++ = (checkCode & 0xff)>>0;

	*pos++ = 0x7e;

	totalDataLen = pos-pData;
	MCULOG("totalDataLen =%d", totalDataLen);

	MCULOG("Before escape Data:");
	for(i = 0; i < totalDataLen; i++)
		MCU_NO("%02x ", *(pData+i));
	MCU_NO("\n");

	serialNO++;
	if (serialNO > 10000)
		serialNO = 0;

	escape_mcuUart_data(pData, totalDataLen);

	if (pData != NULL)
	{
		free(pData);
		pData = NULL;
	}

	return 0;
}

/*****************************************************************************
* Function Name : escape_mcuUart_data
* Description   : 数据转义函数
* Input			: unsigned char *pData
*                 int len
* Output        : None
* Return        : 0:success
* Auther        : ygg
* Date          : 2018.01.18
*****************************************************************************/
int mcuUart::escape_mcuUart_data(unsigned char *pData, int len)
{
	int i;
	int totalEscapeDataLen;
	int escapeDataLen;
	int escapeTimes = 0;
	unsigned char *pBuffData = NULL;

	unsigned char *pBuff = (unsigned char *)malloc(BUFF_LEN);
	if(pBuff == NULL)
		return -1;
    memset(pBuff, 0, BUFF_LEN);

	pBuffData = pBuff;
	*pBuffData = 0x7e;
	MCULOG("Escape *pBuffData:%02x, pBuff=%02x addr:%x\n",*pBuffData,*pBuff, pBuffData);
	pBuffData++;

	escapeDataLen = len-2;
	MCULOG("escapeDataLen:%d\n", escapeDataLen);

	for(i = 0; i < escapeDataLen; i++)
	{
		if((pData[i+1] != 0x7e) && (pData[i+1] != 0x7d))
		{
			*pBuffData = pData[i+1];
			pBuffData++;
		}
		else
		{
			if(pData[i+1] == 0x7e)
			{
				*pBuffData++ = 0x7d;
				*pBuffData++ = 0x02;
			}
			else if(pData[i+1] == 0x7d)
			{
				*pBuffData++ = 0x7d;
				*pBuffData++ = 0x01;
			}
			escapeTimes++;
			MCULOG("escapeTimes:%d\n", escapeTimes);
		}		
	}

	*pBuffData++ = 0x7e;

	MCULOG("escapeTimes:%d\n", escapeTimes);
	totalEscapeDataLen = escapeDataLen+escapeTimes+2;
	MCULOG("totalEscapeDataLen:%d\n", totalEscapeDataLen);
	
	write(fd, pBuff, totalEscapeDataLen);
	MCULOG("write data ok!\n");

	/*AES_CBC_encrypt_decrypt();

	printOutMsg(pBuff, totalEscapeDataLen);

	uint8_t outputData[1024];
	uint16_t len1;
	memset(outputData, 0,sizeof(outputData));

	aes_encrypt_cbc(outputData, &len1, pBuff, totalEscapeDataLen);

	printf("\n\nlen1 = %d\n\n", len1);

	uint8_t outputData1[1024];
	uint16_t len2;
	memset(outputData1, 0,sizeof(outputData1));

	aes_decrypt_cbc(outputData1, &len2, outputData, len1);
	
	printf("\n\nlen2 = %d\n\n", len2); */

	if (pBuff != NULL)
	{
		free(pBuff);
		pBuff = NULL;
	}

	return 0;
}


/*****************************************************************************
* Function Name : pack_report4GState
* Description   : 打包4G状态函数
* Input			: unsigned char *pBuff
*                 int length
* Output        : None
* Return        : 数据长度
* Auther        : ygg
* Date          : 2018.01.18
*****************************************************************************/
uint16_t mcuUart::pack_report4GState(unsigned char *pBuff, int length)
{
    uint8_t *pos = pBuff;
    uint8_t u8Csq = 0;
    uint32_t LTEState = 0;

    //主机状态
    if (LTEState & (0x01 << 0))
        *pos++ = 0x01;
    else
        *pos++ = 0x00;

    //主机4G网络天线状态
    *pos++ = 0x00;

    //主机WIFI网络天线状态
    *pos++ = 0x00;

    //主机蓝牙网络天线状态
    *pos++ = 0x00;

    //4G网络工作状态
    *pos++ = 0x05;//((LTEState >> 7) &0xff);
    //AP模式下WIFI网络工作状态
    *pos++ = 0x01;
    //STA模式下WIFI网络工作状态
    *pos++ = 0x01;
    //蓝牙网路工作状态
    *pos++ = 0x00;
    //4G网络信号强度
    u8Csq = ((LTEState &(0xff << 15)) >> 15);
    *pos++ = u8Csq;

    return (uint16_t)(pos - pBuff);
}


/*****************************************************************************
* Function Name : pack_successMark
* Description   : 打包成功标志函数
* Input			: unsigned char *pBuff
*                 uint8_t *pData
*                 uint16_t len
* Output        : None
* Return        : 数据长度
* Auther        : ygg
* Date          : 2018.01.18
*****************************************************************************/
uint16_t mcuUart::pack_successMark(unsigned char *pBuff, uint8_t *pData, uint16_t len)
{
    uint8_t *pos = pBuff;

	*pos++ = (len & 0xff) >> 0;
	*pos++ = *pData;

    return (uint16_t)(pos - pBuff);
}

int mcuUart::unpack_updateTimingInfo(unsigned char *pData, unsigned int len)
{
	uint16_t i;
	
/*	for (i = 0; i < len; i++)
		MCU_NO("%02x ",*(pData+i));
	MCU_NO("\n\n");*/
//	printf("====== Begin Show unpack_updateTimingInfo data ======\n");
	
//	for(int j = 0; j < len; j++)
//	{
//		printf("%02x ", pData[j]);
//		MCULOG("%02x ",pData[j]);
		
//	}
	
//	printf("====== End Show unpack_updateTimingInfo data ======\n");


	unsigned char *pos = pData;
	uint8_t TempData = 0;
	uint16_t WTempData = 0;
	uint32_t DWTempData = 0;
	
	if(NULL != pData && len > 0)
	{
		TempData = *pos++;
		p_GBT32960_handle->dataInfo.vehicleInfo.vehicleState = TempData;
//		printf("p_GBT32960_handle->dataInfo.vehicleInfo.vehicleState : %02x\n", p_GBT32960_handle->dataInfo.vehicleInfo.vehicleState);
		
//		MCULOG("p_GBT32960_handle->dataInfo.vehicleInfo.vehicleState : %02x\n",p_GBT32960_handle->dataInfo.vehicleInfo.vehicleState);
		
		TempData = *pos++;
		p_GBT32960_handle->dataInfo.vehicleInfo.chargeState = TempData;
//		printf("p_GBT32960_handle->dataInfo.vehicleInfo.chargeState : %02x\n", p_GBT32960_handle->dataInfo.vehicleInfo.chargeState);
		
		TempData = *pos++;
		p_GBT32960_handle->dataInfo.vehicleInfo.runMode = TempData;
//		printf("p_GBT32960_handle->dataInfo.vehicleInfo.runMode : %02x\n", p_GBT32960_handle->dataInfo.vehicleInfo.runMode);
		DWTempData = *pos++;
		DWTempData = ((DWTempData&0xff) << 8) | (*pos++);
		DWTempData = ((DWTempData&0xffff) << 8) | (*pos++);
		DWTempData = ((DWTempData&0xffffff) << 8) | (*pos++);		
		p_GBT32960_handle->dataInfo.vehicleInfo.vehicleMileage = DWTempData * 0.1;
//		printf("p_GBT32960_handle->dataInfo.vehicleInfo.vehicleMileage : %02x\n", p_GBT32960_handle->dataInfo.vehicleInfo.vehicleMileage);
		
		WTempData = *pos++;
		WTempData = ((WTempData&0xff) << 8) | (*pos++);
		p_GBT32960_handle->dataInfo.vehicleInfo.vehicleVoltage = WTempData * 0.1;
//		printf("p_GBT32960_handle->dataInfo.vehicleInfo.vehicleVoltage : %02x\n", p_GBT32960_handle->dataInfo.vehicleInfo.vehicleVoltage);
		
		WTempData = *pos++;
		WTempData = ((WTempData&0xff) << 8) | (*pos++);
		p_GBT32960_handle->dataInfo.vehicleInfo.vehicleCurrent = WTempData * 0.1;
//		printf("p_GBT32960_handle->dataInfo.vehicleInfo.vehicleCurrent : %02x\n", p_GBT32960_handle->dataInfo.vehicleInfo.vehicleCurrent);
		TempData = *pos++;
		p_GBT32960_handle->dataInfo.vehicleInfo.SOC = TempData;
//		printf("p_GBT32960_handle->dataInfo.vehicleInfo.SOC : %02x\n", p_GBT32960_handle->dataInfo.vehicleInfo.SOC);
		TempData = *pos++;
		p_GBT32960_handle->dataInfo.vehicleInfo.DcDcState = TempData;
//		printf("p_GBT32960_handle->dataInfo.vehicleInfo.DcDcState : %02x\n", p_GBT32960_handle->dataInfo.vehicleInfo.DcDcState);
		TempData = *pos++;
		p_GBT32960_handle->dataInfo.vehicleInfo.gear = TempData;
//		printf("p_GBT32960_handle->dataInfo.vehicleInfo.gear : %02x\n", p_GBT32960_handle->dataInfo.vehicleInfo.gear);
		WTempData = *pos++;
		WTempData = ((WTempData&0xff) << 8) | (*pos++);
		p_GBT32960_handle->dataInfo.vehicleInfo.insulationResistance = WTempData;
//		printf("p_GBT32960_handle->dataInfo.vehicleInfo.insulationResistance : %02x\n", p_GBT32960_handle->dataInfo.vehicleInfo.insulationResistance);

		TempData = *pos++;
		p_GBT32960_handle->dataInfo.driveMotorInfo.totalDriveMotor = TempData;
//		printf("p_GBT32960_handle->dataInfo.driveMotorInfo.totalDriveMotor : %02x\n", p_GBT32960_handle->dataInfo.driveMotorInfo.totalDriveMotor);
		for(i = 0; i < p_GBT32960_handle->dataInfo.driveMotorInfo.totalDriveMotor; i++)
		{
			TempData = *pos++;
			p_GBT32960_handle->dataInfo.driveMotorInfo.driveMotor[i].driveMotorState = TempData;
			TempData = *pos++;
			p_GBT32960_handle->dataInfo.driveMotorInfo.driveMotor[i].driveMotorControllerTemp = TempData;
			WTempData = *pos++;
			WTempData = ((WTempData&0xff) << 8) | (*pos++);
			p_GBT32960_handle->dataInfo.driveMotorInfo.driveMotor[i].driveMotorRotationalSpeed = WTempData;
			WTempData = *pos++;
			WTempData = ((WTempData&0xff) << 8) | (*pos++);
			p_GBT32960_handle->dataInfo.driveMotorInfo.driveMotor[i].driveMotorTorque = WTempData * 0.1;
			TempData = *pos++;
			p_GBT32960_handle->dataInfo.driveMotorInfo.driveMotor[i].driveMotorTemp = TempData;
			WTempData = *pos++;
			WTempData = ((WTempData&0xff) << 8) | (*pos++);
			p_GBT32960_handle->dataInfo.driveMotorInfo.driveMotor[i].motorControllerVin = WTempData * 0.1;
			WTempData = *pos++;
			WTempData = ((WTempData&0xff) << 8) | (*pos++);
			p_GBT32960_handle->dataInfo.driveMotorInfo.driveMotor[i].motorControllerDC = WTempData * 0.1;
		}

		TempData = *pos++;
		p_GBT32960_handle->dataInfo.extremeInfo.MaxVolBatterySubsysNumber = TempData;
		TempData = *pos++;
		p_GBT32960_handle->dataInfo.extremeInfo.MaxVolBattery = TempData;
		WTempData = *pos++;
		WTempData = ((WTempData&0xff) << 8) | (*pos++);
		p_GBT32960_handle->dataInfo.extremeInfo.MaxBatteryVoltageValue = WTempData * 0.001;
		TempData = *pos++;
		p_GBT32960_handle->dataInfo.extremeInfo.MinVolBatterySubsysNumber = TempData;
		TempData = *pos++;
		p_GBT32960_handle->dataInfo.extremeInfo.MinVolBattery = TempData;
		WTempData = *pos++;
		WTempData = ((WTempData&0xff) << 8) | (*pos++);
		p_GBT32960_handle->dataInfo.extremeInfo.MinBatteryVoltageValue = WTempData * 0.001;
		TempData = *pos++;
		p_GBT32960_handle->dataInfo.extremeInfo.MaxTemperatureSubsysNumber = TempData;
		TempData = *pos++;
		p_GBT32960_handle->dataInfo.extremeInfo.MaxTemperatureProbe = TempData;
		TempData = *pos++;
		p_GBT32960_handle->dataInfo.extremeInfo.MaxTemperatureValue = TempData;
		TempData = *pos++;
		p_GBT32960_handle->dataInfo.extremeInfo.MinTemperatureSubsysNumber = TempData;
		TempData = *pos++;
		p_GBT32960_handle->dataInfo.extremeInfo.MinTemperatureProbe = TempData;
		TempData = *pos++;
		p_GBT32960_handle->dataInfo.extremeInfo.MinTemperatureValue = TempData;

		
		TempData = *pos++;
		p_GBT32960_handle->dataInfo.alarmInfo.maxAlarmLevel = TempData;
		DWTempData = *pos++;
		DWTempData = ((DWTempData & 0xff)<< 8) | *pos++;
		DWTempData = ((DWTempData & 0xffff)<< 8) | *pos++;
		DWTempData = ((DWTempData & 0xffffff)<< 8) | *pos++;
		p_GBT32960_handle->dataInfo.alarmInfo.generalAlarm.AlarmFlag = DWTempData;	
//		printf("====== fault. ======\n");
		pos += 10;  //从电池故障开始
		TempData = *pos++;
		p_GBT32960_handle->dataInfo.alarmInfo.totalFaultNumOfEnergyStorageDev = TempData;
//		printf("alarmInfo.totalFaultNumOfEnergyStorageDev : %02x\n", p_GBT32960_handle->dataInfo.alarmInfo.totalFaultNumOfEnergyStorageDev);
		for(i = 0; i < p_GBT32960_handle->dataInfo.alarmInfo.totalFaultNumOfEnergyStorageDev; i++ )
		{
			DWTempData = *pos++;
			DWTempData = ((DWTempData & 0xff)<< 8) | *pos++;
			DWTempData = ((DWTempData & 0xffff)<< 8) | *pos++;
			DWTempData = ((DWTempData & 0xffffff)<< 8) | *pos++;
			p_GBT32960_handle->dataInfo.alarmInfo.energyStorageDevFault[i].fault = DWTempData;
		}
//		printf("alarmInfo.energyStorageDevFault[i].fault : %08x\n", p_GBT32960_handle->dataInfo.alarmInfo.energyStorageDevFault[i].fault);
		
		TempData = *pos++;
		p_GBT32960_handle->dataInfo.alarmInfo.totalFaultNumOfDriveMotor = TempData;
		for(i = 0; i < p_GBT32960_handle->dataInfo.alarmInfo.totalFaultNumOfDriveMotor; i++ )
		{
			DWTempData = *pos++;
			DWTempData = ((DWTempData & 0xff)<< 8) | *pos++;
			DWTempData = ((DWTempData & 0xffff)<< 8) | *pos++;
			DWTempData = ((DWTempData & 0xffffff)<< 8) | *pos++;
			p_GBT32960_handle->dataInfo.alarmInfo.driveMotorFault[i].fault = DWTempData;
		}
		
		TempData = *pos++;
		p_GBT32960_handle->dataInfo.alarmInfo.totalFaultNumOfOther = TempData;
		for(i = 0; i < p_GBT32960_handle->dataInfo.alarmInfo.totalFaultNumOfOther; i++ )
		{
			DWTempData = *pos++;
			DWTempData = ((DWTempData & 0xff)<< 8) | *pos++;
			DWTempData = ((DWTempData & 0xffff)<< 8) | *pos++;
			DWTempData = ((DWTempData & 0xffffff)<< 8) | *pos++;
			p_GBT32960_handle->dataInfo.alarmInfo.otherFault[i].fault = DWTempData;
		}
		
		pos += 85; //忽略其它故障
		TempData = *pos++;
		if(TempData > 2)
		{
			TempData = 2;
		}
		p_GBT32960_handle->dataInfo.ESDSubsysInfo.totalEnergySubsys = TempData;
		
		for(i =0; i < p_GBT32960_handle->dataInfo.ESDSubsysInfo.totalEnergySubsys; i++)
		{
			TempData = *pos++;
			p_GBT32960_handle->dataInfo.ESDSubsysInfo.ESDevSubsysVol[i].subsysNumber = TempData;
			WTempData = *pos++;
			WTempData = ((WTempData&0xff) << 8) | (*pos++);
			p_GBT32960_handle->dataInfo.ESDSubsysInfo.ESDevSubsysVol[i].subsysVol = WTempData * 0.1;
			WTempData = *pos++;
			WTempData = ((WTempData&0xff) << 8) | (*pos++);
			p_GBT32960_handle->dataInfo.ESDSubsysInfo.ESDevSubsysVol[i].subsysCurrent = WTempData * 0.1;
			WTempData = *pos++;
			WTempData = ((WTempData&0xff) << 8) | (*pos++);
			p_GBT32960_handle->dataInfo.ESDSubsysInfo.ESDevSubsysVol[i].totalNumOfSingleCell = WTempData;
			WTempData = *pos++;
			WTempData = ((WTempData&0xff) << 8) | (*pos++);
			p_GBT32960_handle->dataInfo.ESDSubsysInfo.ESDevSubsysVol[i].frameStartCellNumber = WTempData;
			TempData = *pos++;
			if(TempData > 168)
			{
				TempData = 168;
			}
			
			p_GBT32960_handle->dataInfo.ESDSubsysInfo.ESDevSubsysVol[i].frameStartCellTotal = TempData;
			
			int j = 0; 
			for(j = 0; j < p_GBT32960_handle->dataInfo.ESDSubsysInfo.ESDevSubsysVol[i].frameStartCellTotal ; j++)
			{
				WTempData = *pos++;
				WTempData = ((WTempData&0xff) << 8) | (*pos++);
				p_GBT32960_handle->dataInfo.ESDSubsysInfo.ESDevSubsysVol[i].monomerCellVol[j] = WTempData * 0.001;
			}
			
			WTempData = *pos++;
			WTempData = ((WTempData&0xff) << 8) | (*pos++);
			if(WTempData > 24)
			{
				WTempData = 24;
			}
			p_GBT32960_handle->dataInfo.ESDSubsysInfo.ESDevSubsysVol[i].subsysProbeNumber = WTempData;
			
//			printf("ESDSubsysInfo.ESDevSubsysVol[i].subsysProbeNumber : %04x\n", p_GBT32960_handle->dataInfo.ESDSubsysInfo.ESDevSubsysVol[i].subsysProbeNumber);
			for(j = 0; j < p_GBT32960_handle->dataInfo.ESDSubsysInfo.ESDevSubsysVol[i].subsysProbeNumber; j++)
			{
				TempData = *pos++;
				p_GBT32960_handle->dataInfo.ESDSubsysInfo.ESDevSubsysVol[i].subsysProbeTemper[j] = TempData;
			}	
		}	
		p_GBT32960->updateTBoxParameterInfo();
		
	}
	
//	printf("====== Show Timing data end. ======\n");
//	MCULOG("====== Show Timing data end. ======\n");
	return 0;
}

int mcuUart::unpack_RemoteCtrl(unsigned char *pData, uint16_t datalen)
{
	register uint8_t SubitemCode;
	register uint8_t SubitemCodeParam;
	uint16_t serialNumber;
	static uint8_t ReCtrlTotalType = 0;
	serialNumber = (pData[4] << 8) + pData[5];
	MCULOG("serialNumber:%d\n",serialNumber);
	
	MCULOG("\n\n\nMCU  remote Data:");
	for(int i = 0; i < datalen; i++)
		MCU_NO("%02x ", *(pData+i));
	MCU_NO("\n");

	SubitemCode = pData[8];
	SubitemCodeParam = pData[9];

	MCULOG("MCU SubitemCodeParam == %d\n",SubitemCodeParam);
	switch(SubitemCode)
	{
		case VehicleBody_LockID:
			p_FAWACPInfo_Handle->RemoteControlData.VehicleBody.VehicleBody_Lock = SubitemCodeParam;
			break;
		case VehicleBody_WindowID:
			p_FAWACPInfo_Handle->RemoteControlData.VehicleBody.VehicleBody_Window = SubitemCodeParam;
			break;
		case VehicleBody_SunroofID:
			p_FAWACPInfo_Handle->RemoteControlData.VehicleBody.VehicleBody_Sunroof = SubitemCodeParam;
			break;
		case VehicleBody_TrackingCarID:
			p_FAWACPInfo_Handle->RemoteControlData.VehicleBody.VehicleBody_TrackingCar = SubitemCodeParam;
			break;
		case VehicleBody_LowbeamID:
			p_FAWACPInfo_Handle->RemoteControlData.VehicleBody.VehicleBody_Lowbeam = SubitemCodeParam;
			break;
		case Airconditioner_ControlID:
			p_FAWACPInfo_Handle->RemoteControlData.Airconditioner.Airconditioner_Control.dataBit.dataState = SubitemCodeParam & 0x7F;
			p_FAWACPInfo_Handle->RemoteControlData.Airconditioner.Airconditioner_Control.dataBit.flag = SubitemCodeParam >> 7;
			break;
		case Airconditioner_CompressorSwitchID:
			p_FAWACPInfo_Handle->RemoteControlData.Airconditioner.Airconditioner_CompressorSwitch.dataBit.dataState = SubitemCodeParam & 0x7F;
			p_FAWACPInfo_Handle->RemoteControlData.Airconditioner.Airconditioner_CompressorSwitch.dataBit.flag = SubitemCodeParam >> 7;
			break;
		case Airconditioner_TemperatureID:
			p_FAWACPInfo_Handle->RemoteControlData.Airconditioner.Airconditioner_Temperature.dataBit.dataState = SubitemCodeParam & 0x7F;
			p_FAWACPInfo_Handle->RemoteControlData.Airconditioner.Airconditioner_Temperature.dataBit.flag = SubitemCodeParam >> 7;
			break;
		case Airconditioner_SetAirVolumeID:
			p_FAWACPInfo_Handle->RemoteControlData.Airconditioner.Airconditioner_SetAirVolume.dataBit.dataState = SubitemCodeParam & 0x7F;
			p_FAWACPInfo_Handle->RemoteControlData.Airconditioner.Airconditioner_SetAirVolume.dataBit.flag = SubitemCodeParam >> 7;
			break;
		case Airconditioner_FrontDefrostSwitchID:
			p_FAWACPInfo_Handle->RemoteControlData.Airconditioner.Airconditioner_FrontDefrostSwitch.dataBit.dataState = SubitemCodeParam & 0x7F;
			p_FAWACPInfo_Handle->RemoteControlData.Airconditioner.Airconditioner_FrontDefrostSwitch.dataBit.flag = SubitemCodeParam >> 7;
			break;
		case Airconditioner_HeatedrearID:
			p_FAWACPInfo_Handle->RemoteControlData.Airconditioner.Airconditioner_Heatedrear.dataBit.dataState = SubitemCodeParam & 0x7F;
			p_FAWACPInfo_Handle->RemoteControlData.Airconditioner.Airconditioner_Heatedrear.dataBit.flag = SubitemCodeParam >> 7;
			break;
		case Airconditioner_BlowingModeID:
			p_FAWACPInfo_Handle->RemoteControlData.Airconditioner.Airconditioner_BlowingMode.dataBit.dataState = SubitemCodeParam & 0x7F;
			p_FAWACPInfo_Handle->RemoteControlData.Airconditioner.Airconditioner_BlowingMode.dataBit.flag = SubitemCodeParam >> 7;
			break;
		case Airconditioner_InOutCirculateID:
			p_FAWACPInfo_Handle->RemoteControlData.Airconditioner.Airconditioner_InOutCirculate.dataBit.dataState = SubitemCodeParam & 0x7F;
			p_FAWACPInfo_Handle->RemoteControlData.Airconditioner.Airconditioner_InOutCirculate.dataBit.flag = SubitemCodeParam >> 7;
			break;
		case Airconditioner_AutoSwitchID:
			p_FAWACPInfo_Handle->RemoteControlData.Airconditioner.Airconditioner_AutoSwitch.dataBit.dataState = SubitemCodeParam & 0x7F;
			p_FAWACPInfo_Handle->RemoteControlData.Airconditioner.Airconditioner_AutoSwitch.dataBit.flag = SubitemCodeParam >> 7;
			break;
		case EngineState_SwitchID:
			p_FAWACPInfo_Handle->RemoteControlData.EngineState.EngineState_Switch = SubitemCodeParam;
			break;
		case VehicleSeat_DrivingSeatID:
			p_FAWACPInfo_Handle->RemoteControlData.VehicleSeat.VehicleSeat_DrivingSeat = SubitemCodeParam;
			break;
		case VehicleSeat_CopilotseatID:
			p_FAWACPInfo_Handle->RemoteControlData.VehicleSeat.VehicleSeat_Copilotseat = SubitemCodeParam;
			break;
		default:
			break;
	}
	//执行一次Subitemnumber加一，等于控制总数时，发送应答
	ReCtrlTotalType++;
	MCULOG("MCU ReCtrlTotalType == %d\n",ReCtrlTotalType);
	if(Subitemnumber == ReCtrlTotalType)
	{
		MCULOG("ReCtrlTotalType == %d\n",ReCtrlTotalType);
		ReplayRemoteCtrl(CFAWACP::cfawacp->cb_TspRemoteCtrl);
		ReCtrlTotalType = 0;
	}
	
	return 0;
}

//回调ACP发送远程控制应答指定
void mcuUart::ReplayRemoteCtrl(callback_ReplayRemoteCtrl cb_TspRemoteCtrl)
{
	cb_TspRemoteCtrl();	
}


int mcuUart::cb_RemoteCtrlCmd(uint8_t SubitemCode, uint8_t SubitemCodeParam, uint8_t SubitemTotal)
{
	MCULOG("SubitemTotal == %d\n",SubitemTotal);
	int i,headLen;
	int totalDataLen;
	unsigned char *pos = NULL;
	unsigned short int dataLen = 2;
	unsigned int checkCode;
	static unsigned short int serialNO = 0;
	m_mcuUart->Subitemnumber = SubitemTotal;

	unsigned char *pBuff = (unsigned char *)malloc(BUFF_LEN);
	if(pBuff == NULL)
		return -1;

	memset(pBuff, 0, BUFF_LEN);
	pos = pBuff;

	*pos++ = 0x7e;
	//data length
	*pos++ = 0;
	*pos++ = 0;
	//cmd
	*pos++ = 0x84;
	//serial NO.
	*pos++ = (serialNO & 0xff00) >> 8;
	*pos++ = (serialNO & 0xff) >> 0;

	//data property: the data which be send has been encrypted
	*pos++ = 0;
	*pos++ = 0;

	headLen = pos-pBuff;
	//MCULOG("headLen = %d \t subCmd:%02x\n", headLen, subCmd);

	switch(SubitemCode)
	{
		case VehicleBody_LockID:
			*pos++ = 0x01;
			break;
		case VehicleBody_WindowID:
			*pos++ = 0x02;
			break;
		case VehicleBody_SunroofID:
			*pos++ = 0x03;
			break;
		case VehicleBody_TrackingCarID:
			*pos++ = 0x04;
			break;
		case VehicleBody_LowbeamID:
			*pos++ = 0x05;
			break;
		case Airconditioner_ControlID:
			*pos++ = 0x06;
			break;
		case Airconditioner_CompressorSwitchID:
			*pos++ = 0x07;
			break;
		case Airconditioner_TemperatureID:
			*pos++ = 0x08;
			break;
		case Airconditioner_SetAirVolumeID:
			*pos++ = 0x09;
			break;
		case Airconditioner_FrontDefrostSwitchID:
			*pos++ = 0x0A;
			break;
		case Airconditioner_HeatedrearID:
			*pos++ = 0x0B;
			break;
		case Airconditioner_BlowingModeID:
			*pos++ = 0x0C;
			break;
		case Airconditioner_InOutCirculateID:
			*pos++ = 0x0D;
			break;
		case Airconditioner_AutoSwitchID:
			*pos++ = 0x0E;
			break;
		case EngineState_SwitchID:
			*pos++ = 0x0F;
			break;
		case VehicleSeat_DrivingSeatID:
			*pos++ = 0x10;
			break;
		case VehicleSeat_CopilotseatID:
			*pos++ = 0x11;
			break;
		case VehicleChargeMode_ImmediateID:
			*pos++ = 0x12;
			return 0;
			break;
		case VehicleChargeMode_AppointmentID:
			*pos++ = 0x13;
			return 0;
			break;
		case VehicleWIFIStatusID:
			*pos++ = 0x14;
			break;
		case VehicleAutoOUTID:
			*pos++ = 0x15;
			break;
	}
	*pos++ = SubitemCodeParam;

	MCULOG("dataLen = %d\n",dataLen);

	pBuff[1] = (dataLen & 0xff00)>>8;
	pBuff[2] = (dataLen & 0xff)>>0;

//	pos += dataLen;
	
	//Calculated check code
	checkCode = m_mcuUart->Crc16Check(&pBuff[1], pos-pBuff-1);
	//MCULOG("checkCode = %0x\n", checkCode);
	
	*pos++ = (checkCode & 0xff00)>>8;
	*pos++ = (checkCode & 0xff)>>0;

	*pos++ = 0x7e;

	totalDataLen = pos-pBuff;
	MCULOG("totalDataLen =%d", totalDataLen);

	MCULOG("\n\n\nBefore escape Data:");
	for(i = 0; i < totalDataLen; i++)
		MCU_NO("%02x ", *(pBuff+i));
	MCU_NO("\n");

	if(m_mcuUart->escape_mcuUart_data(pBuff, totalDataLen) == -1)
	{
		MCULOG();
		return -1;
	}

	serialNO++;
	if (serialNO > 65534)
		serialNO = 0;

	if (pBuff != NULL)
	{
		free(pBuff);
		pBuff = NULL;
	}
	return 0;
}


int mcuUart::cb_RemoteConfigCmd(uint8_t SubitemCode, uint16_t SubitemVal)
{
	int i,headLen;
	int totalDataLen;
	unsigned char *pos = NULL;
	unsigned short int dataLen = 2;
	unsigned int checkCode;
	static unsigned short int serialNO = 0;
	unsigned char *pBuff = (unsigned char *)malloc(BUFF_LEN);
	if(pBuff == NULL)
		return ;

	memset(pBuff, 0, BUFF_LEN);
	pos = pBuff;

	*pos++ = 0x7e;
	//data length
	*pos++ = 0;
	*pos++ = 0;
	//cmd
	*pos++ = 0x87;
	//serial NO.
	*pos++ = (serialNO & 0xff00) >> 8;
	*pos++ = (serialNO & 0xff) >> 0;

	//data property: the data which be send has been encrypted
	*pos++ = 0;
	*pos++ = 0;

	headLen = pos-pBuff;
	//MCULOG("headLen = %d \t subCmd:%02x\n", headLen, subCmd);

	*pos++ = SubitemCode;
	*pos++ = (SubitemVal >> 8) & 0xFF;
	*pos++ = SubitemVal & 0xFF;

	MCULOG("dataLen = %d\n",dataLen);

	pBuff[1] = (dataLen & 0xff00)>>8;
	pBuff[2] = (dataLen & 0xff)>>0;

//	pos += dataLen;
	
	//Calculated check code
	checkCode = m_mcuUart->Crc16Check(&pBuff[1], pos-pBuff-1);
	//MCULOG("checkCode = %0x\n", checkCode);
	
	*pos++ = (checkCode & 0xff00)>>8;
	*pos++ = (checkCode & 0xff)>>0;

	*pos++ = 0x7e;

	totalDataLen = pos-pBuff;
	MCULOG("totalDataLen =%d", totalDataLen);

	MCULOG("\n\n\nBefore escape Data:");
	for(i = 0; i < totalDataLen; i++)
		MCU_NO("%02x ", *(pBuff+i));
	MCU_NO("\n");

	if(m_mcuUart->escape_mcuUart_data(pBuff, totalDataLen) == -1)
	{
		MCULOG();
		return -1;
	}

	serialNO++;
	if (serialNO > 65534)
		serialNO = 0;

	if (pBuff != NULL)
	{
		free(pBuff);
		pBuff = NULL;
	}
	return 0;
}


//============================================= mcu upgrade ======================================================
#if 1
int mcuUart::pack_mcuUart_upgrade_data(unsigned char cmd, bool isReceivedDataCorrect, int mcuOrPlcFlag)
{
	int i,headLen;
	int totalDataLen;
	unsigned char *pos = NULL;
	unsigned short int dataLen;
	unsigned int checkCode;
	
	static unsigned short int serialNo = 0;
	unsigned char *pData = (unsigned char *)malloc(BUFF_LEN);
	if(pData == NULL)
		return -1;
    memset(pData, 0, BUFF_LEN);
	
	MCULOG("mcuOrPlcFlag = %d\n",mcuOrPlcFlag);
	pos = pData;
	
	*pos++ = 0x7e;
	//data length
	*pos++ = 0;
	*pos++ = 0;
	//cmd
	*pos++ = cmd;
	//serial NO.
	*pos++ = (serialNo & 0xff00) >> 8;
	*pos++ = (serialNo & 0xff) >> 0;
	//data property: the data which be send has been encrypted
	*pos++ = 0;
	*pos++ = 0;

	headLen = pos-pData;
	MCULOG("headLen = %d\n", headLen);
	
	switch (cmd)
	{
		case TBOX_SEND_UPGRADE_CMD:
			dataLen = pack_upgrade_cmd(pData, headLen, mcuOrPlcFlag);
			break;
		case TBOX_RECV_MCU_APPLY_FOR:
			dataLen = pack_upgrade_data(pData, headLen, isReceivedDataCorrect, mcuOrPlcFlag);
			break;
		case MCU_SEND_COMPLETE:
			system(RM_MCU_FILE);
			break;
		default:
			MCULOG("The cmd error!\n");
			break;
	}
	
	MCULOG("dataLen = %d\n",dataLen);

	pData[1] = (dataLen & 0xff00)>>8;
	pData[2] = (dataLen & 0xff)>>0;

	pos += dataLen;
	
	//Calculated check code
	checkCode = Crc16Check(&pData[1], pos-pData-1);
	MCULOG("checkCode = %0x\n", checkCode);
	
	*pos++ = (checkCode & 0xff00)>>8;
	*pos++ = (checkCode & 0xff)>>0;

	*pos++ = 0x7e;

	totalDataLen = pos-pData;
	MCULOG("totalDataLen =%d", totalDataLen);

	MCULOG("Before escape Data:");
	for(i = 0; i < totalDataLen; i++)
		MCU_NO("%02x ", *(pData+i));
	MCU_NO("\n");
	
	escape_mcuUart_data(pData, totalDataLen);

    serialNo++;
    if (serialNo > 10000)
        serialNo = 0;

	if (pData != NULL)
	{
		free(pData);
		pData = NULL;
	}

    return 0;
}

unsigned short int mcuUart::pack_upgrade_cmd(unsigned char *pData, int len, int mcuOrPlcFlag)
{
	unsigned char fileNameLen;
	unsigned char *pos = NULL;

	if(len != 8)
		return -1;

	pos = pData;
	pos += len;

	if(mcuOrPlcFlag == 0)
	{
		fileNameLen = strlen(MCU_UPGRADE_FILE);
		MCULOG("fileNameLen:%d\n", fileNameLen);
	
		*pos++ = 0x00;
		memcpy(pos, upgradeInfo.mcuVersionSize, 4);
		pos += 4;
		//Check code
		memcpy(pos, upgradeInfo.mcuVersionCrc16, 2);
		pos += 2;
		//MCU upgrade file name length
		*pos++ = fileNameLen;
		memcpy(pos, MCU_UPGRADE_FILE, fileNameLen);
		pos+= fileNameLen;
		//MCU version length
		*pos++ = 0x04;
		//MCU version
		memcpy(pos, upgradeInfo.newMcuVersion, 4);
		pos += 4;
	}
	else if(mcuOrPlcFlag == 1)
	{
		fileNameLen = strlen(PLC_UPGRADE_FILE);
		MCULOG("fileNameLen:%d\n", fileNameLen);
		
		*pos++ = 0x01;
		memcpy(pos, upgradeInfo.verionSize, 4);
		pos += 4;
		//Check code
		memcpy(pos, upgradeInfo.checkCodeCrc16, 2);
		pos += 2;
		//Plc upgrade file name length
		*pos++ = fileNameLen;
		memcpy(pos, PLC_UPGRADE_FILE, fileNameLen);
		pos+= fileNameLen;
		//plc version length
		*pos++ = 0x14;
		//plc version
		memcpy(pos, upgradeInfo.newPlcVersion, 20);
		pos += 20;
	}
	
	//Calculated data length
	MCULOG("Pack data len:%d\n",(int)(pos-pData-len));
	
	return (unsigned short int)(pos-pData-len);
}

unsigned short int mcuUart::pack_upgrade_data(unsigned char *pData, int len, bool isReceivedDataCorrect, int mcuOrPlcFlag)
{
	int nRead;
	unsigned char *pos = NULL;
	unsigned int offsetAddr;
	unsigned short int packetLen;
	unsigned char buff[BUFF_LEN];
	int fileFd = -1;

	if(len != 8)
		return -1;

	pos = pData;
	pos += len;

	if(isReceivedDataCorrect == true)
	{
		*pos++ = 0x00;
		memcpy(pos, dataPacketOffsetAddr, 4);
		pos += 4;
		memcpy(pos, dataPacketLen, 2);
		pos += 2;
		
		//get plc upgrade data's offset addr and len
		offsetAddr	= (dataPacketOffsetAddr[0]) << 24;
		offsetAddr += (dataPacketOffsetAddr[1]) << 16;
		offsetAddr += (dataPacketOffsetAddr[2]) << 8;
		offsetAddr += (dataPacketOffsetAddr[3]) << 0;
		MCULOG("offsetAddr = %d\n", offsetAddr);
		
		packetLen  = (dataPacketLen[0]) << 8;
		packetLen += (dataPacketLen[1]) << 0;
		MCULOG("packetLen = %d\n", packetLen);
		
		if(mcuOrPlcFlag == 0)
		{
			fileFd = open(MCU_UPGRADE_FILE, O_RDONLY);
			if (fileFd < 0)
			{
				MCULOG("Open file:%s error.\n", MCU_UPGRADE_FILE);
				return -1;
			}

			lseek(fileFd, offsetAddr, SEEK_SET);

			memset(buff, 0, BUFF_LEN);
			if((nRead = read(fileFd, buff, BUFF_LEN)) != -1)
			{
				MCULOG("read file %s, nRead = %d\n", MCU_UPGRADE_FILE, nRead);
				memcpy(pos, buff, packetLen);
				pos += packetLen;
			}
			
			close(fileFd);
		}
		else if(mcuOrPlcFlag == 1)
		{
			fileFd = open(PLC_UPGRADE_FILE, O_RDONLY);
			if (fileFd < 0)
			{
				MCULOG("Open file:%s error.\n", PLC_UPGRADE_FILE);
				return -1;
			}

			lseek(fileFd, offsetAddr, SEEK_SET);

			memset(buff, 0, BUFF_LEN);
			if((nRead = read(fileFd, buff, BUFF_LEN)) != -1)
			{
				MCULOG("read file %s, nRead = %d\n", PLC_UPGRADE_FILE, nRead);
				memcpy(pos, buff, packetLen);
				pos += packetLen;
			}
			close(fileFd);
		}
	}
	else
	{
		*pos++ = 0x01;
		MCULOG("Can received mcu respond!\n");
	}
	
	MCULOG("Pack data len:%d\n",(int)(pos-pData-len));
	return (unsigned short int)(pos-pData-len);
}

void mcuUart::mcu_apply_for_data(unsigned char* pData, uint32_t len)
{
	int i;
	unsigned int addr;
	unsigned short int packetLen; 
	unsigned char *pos = pData;
	unsigned int verionLength;
	unsigned int fileDatalen;
	unsigned char fileLen;
	
	fileLen = *(pos+8);
	MCULOG("fileLen:%d\n", fileLen);

	memset(fileName, 0, 64);
	memcpy(fileName, pos+8+1, fileLen);
	MCULOG("fileName:%s\n", fileName);

	//get plc upgrade data's addr and len
	addr  = *(pos+8+1+fileLen) << 24;
	addr += *(pos+8+1+fileLen+1) << 16;
	addr += *(pos+8+1+fileLen+2) << 8;
	addr += *(pos+8+1+fileLen+3) << 0;
	MCULOG("offsetAddr = %d\n", addr);
	
	packetLen  = *(pos+8+1+fileLen+4) << 8;
	packetLen += *(pos+8+1+fileLen+5) << 0;
	MCULOG("packetLen = %d\n", packetLen);

	fileDatalen = addr + (unsigned int)packetLen;
	
	if(strcmp((char*)fileName, MCU_UPGRADE_FILE) == 0)
	{
		upgradeInfo.upgradeFlag = 0;
		verionLength  = (upgradeInfo.mcuVersionSize[0]) << 24;
		verionLength += (upgradeInfo.mcuVersionSize[1]) << 16;
		verionLength += (upgradeInfo.mcuVersionSize[2]) << 8;
		verionLength += (upgradeInfo.mcuVersionSize[3]) << 0;
		
		len = strlen(MCU_UPGRADE_FILE); 
		MCULOG("len:%d\n", len);
		
		if((fileLen == len) && (memcmp(MCU_UPGRADE_FILE, fileName, len) == 0) && \
			(fileDatalen <= verionLength))
		{
			MCULOG("Data offset addr and length:\n");
			memcpy(dataPacketOffsetAddr, pos+8+1+fileLen, 4);
			
			for(i = 0; i<4; i++)
				MCU_NO("%02x ",*(dataPacketOffsetAddr+i));
			MCU_NO("\n");
			
			memcpy(dataPacketLen, pos+8+1+fileLen+4, 2);
			
			for(i = 0; i<2; i++)
				MCU_NO("%02x ",*(dataPacketLen+i));
			MCU_NO("\n");
		
			pack_mcuUart_upgrade_data(TBOX_RECV_MCU_APPLY_FOR, true, 0);	
		}
		else
		{
			MCULOG("Received the file name and len error form mcu!\n");
			pack_mcuUart_upgrade_data(TBOX_RECV_MCU_APPLY_FOR, false, 0);
		}
	}
	else if(strcmp((char*)fileName, PLC_UPGRADE_FILE) == 0)
	{
		upgradeInfo.sendPlcDataToMCUFlag = 0;
		verionLength  = (upgradeInfo.verionSize[0]) << 24;
		verionLength += (upgradeInfo.verionSize[1]) << 16;
		verionLength += (upgradeInfo.verionSize[2]) << 8;
		verionLength += (upgradeInfo.verionSize[3]) << 0;

		len = strlen(PLC_UPGRADE_FILE); 
		MCULOG("len:%d\n", len);

		if((fileLen == len) && (memcmp(PLC_UPGRADE_FILE, fileName, len) == 0) && \
			(fileDatalen <= verionLength))
		{
			MCULOG("Data offset addr and length:\n");
			memcpy(dataPacketOffsetAddr, pos+8+1+fileLen, 4);
			
			for(i = 0; i<4; i++)
				MCU_NO("%02x ",*(dataPacketOffsetAddr+i));
			MCU_NO("\n");
			
			memcpy(dataPacketLen, pos+8+1+fileLen+4, 2);
			
			for(i = 0; i<2; i++)
				MCU_NO("%02x ",*(dataPacketLen+i));
			MCU_NO("\n");
		
			pack_mcuUart_upgrade_data(TBOX_RECV_MCU_APPLY_FOR, true, 1);	
		}
		else
		{
			MCULOG("Received the file name and len error form mcu!\n");
			pack_mcuUart_upgrade_data(TBOX_RECV_MCU_APPLY_FOR, false, 1);
		}
	}
}


#endif


#if 1
/*****************************************************************************
* Function Name : unpack_MCU_SND_Upgrade_Info
* Description   : get upgrade info
* Input			: unsigned char *pData
*                 unsigned int len
* Output        : None
* Return        : 0:success
* Auther        : ygg
* Date          : 2018.05.22
*****************************************************************************/
int mcuUart::unpack_MCU_SND_Upgrade_Info(unsigned char* pData, unsigned int len)
{
	//uint16_t SN;
	uint8_t fileName[50];
	uint8_t length;
	uint8_t *pos = pData;
	
	if(*(pos+8) == 0x00)
	{
		MCULOG("upgrade tbox\n");
	}
	else
		MCULOG("upgrade other\n");

	dataSize = (*(pos+9)<<24)+(*(pos+10)<<16)+(*(pos+11)<<8)+*(pos+12);
	MCULOG("dataSize:%d\n", dataSize);

	crc16 = (*(pos+13)<<8)+*(pos+14);
	MCULOG("crc16:%04x, %d\n", crc16, crc16);

	length = *(pos+15);
	MCULOG("file name len:%d\n", *(pos+15));

	memset(fileName, 0, sizeof(fileName));
	memcpy(fileName, pos+16, length);
	MCULOG("file name:%s\n", fileName);

	pos = pos+16+length;
	
	length = *pos;
	MCULOG("file version len:%d\n", length);

	memset(tboxVersion, 0, sizeof(tboxVersion));
	memcpy(tboxVersion, pos+1, length);
	MCULOG("file version:%s\n", tboxVersion);

	//SN = (pData[4] << 8) + pData[5];
	//MCULOG("serialNumber:%d\n",SN);

	//set the result as successfully
	length = 0x00;
	packProtocolData(TBOX_REPLY_ID, 0, &length, 1, 0); //SN);

	return 0;
}

/*****************************************************************************
* Function Name : unpack_MCU_SND_Upgrade_Data
* Description   : get upgrade data
* Input			: unsigned char *pData
*                 unsigned int len
* Output        : None
* Return        : 0:success
* Auther        : ygg
* Date          : 2018.05.22
*****************************************************************************/
int mcuUart::unpack_MCU_SND_Upgrade_Data(unsigned char* pData, unsigned int len)
{
	int ret;
	uint16_t length;
	uint8_t result;
	
	length = (pData[8]<<16)+pData[9];
	MCULOG("data len:%d\n", length);
	
	ret = storageFile(LTE_FILE_NAME, &pData[10], length);

	if(ret == -1)
		result = 0x01;
	else	
		result = 0x00;

	packProtocolData(TBOX_REPLY_ID, 0, &result, 1, 0); //SN);

	return 0;
}

int mcuUart::storageFile(uint8_t *fileName, uint8_t *pData, uint32_t len)
{
	int fd = open(fileName, O_RDWR | O_CREAT | O_APPEND, 0777);
	if (-1 == fd)
	{
		printf("open file failed\n");
		return -1;
	}
	if (len != write(fd, pData, len))
	{
		printf("write file error!\n");
		return -1;
	}

	close(fd);

	return 0;
}

/*****************************************************************************
* Function Name : unpack_MCU_SND_Upgrade_CMPL
* Description   : send upgrade completely
* Input			: None
* Output        : None
* Return        : 0:success
* Auther        : ygg
* Date          : 2018.05.22
*****************************************************************************/
int mcuUart::unpack_MCU_SND_Upgrade_CMPL()
{
	int ret;
	uint32_t crc;
	uint8_t result = 0;
	
	ret = calculate_files_CRC(LTE_FILE_NAME, &crc);
	if(ret != 0)
	{
		result = 0x01;
	}
	else
	{
		if(crc == crc16)
			result = 0x00;
		else{		
			result = 0x01;
			system(RM_LTE_FILE);
		}
	}

	packProtocolData(TBOX_REPLY_ID, 0, &result, 1, 0);

	return 0;
}

/*****************************************************************************
* Function Name : calculate_files_CRC
* Description   : calculat
* Input			: None
* Output        : None
* Return        : 0:success
* Auther        : ygg
* Date          : 2018.05.22
*****************************************************************************/
int mcuUart::calculate_files_CRC(char *fileName, uint32_t *crc)
{
	int nRead;
	uint8_t buff[1024];
	int fd = 0;

	fd = open(fileName, O_RDONLY);
	if (fd < 0)
	{
		printf("Open file:%s error.\n", fileName);
		return -1;
	}else
		printf("Open file:%s success.\n", fileName);

	memset(buff, 0, sizeof(buff));
	while ((nRead = read(fd, buff, sizeof(buff))) > 0)
	{
		*crc = Crc16Check(buff, nRead);
	}

	close(fd);

	return 0;
}


#endif

