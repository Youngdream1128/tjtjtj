#include "mcuUart.h"
#include "AdcVoltageCheck.h"


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
        memset(&FaultSigan, 0, sizeof(Fault_Sigan_t));
	m_mcuUart = this;
	
	mcuUartInit();
	
	if(pthread_create(&CheckPWMThreadId, NULL, CheckPWMThread, this) != 0)
		MCULOG("Cannot creat CheckPWMThread:%s\n", strerror(errno));

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

void *mcuUart::CheckPWMThread(void *arg)
{
	pthread_detach(pthread_self()); 
	mcuUart *pmcuUart = (mcuUart*)arg;

	int nNumCheckHighVol = 0;
	int nNumCheckLowVol  = 0;

	int nVoltage_SRS 	 = 0;//检测SRS电压值
	int nPeriodCheckPWM  = 0;//检测高电平的周期
	int nCountNullCheckVol = 0; //检测电平次数

	usleep(200000);
	int iLoop = 0;
	int iRet = -1;
	while(1)
	{
		usleep(40000);
		//检测SRS电压
		if(adc_voltage_check(&nVoltage_SRS) == 0)
		{
			if(nVoltage_SRS < 1800 && nVoltage_SRS > 1200)//高电平
			{
				nNumCheckHighVol++;
				nCountNullCheckVol = 0;
				}
			else if(nVoltage_SRS < 400)//低电平
			{
				nNumCheckLowVol++;
				nCountNullCheckVol = 0;
				}
			else
			{
				nCountNullCheckVol++;
				//printf("check adc voltage count = %d\n",nCountNullCheckVol);
				}				
			//printf("Current nVoltage_SRS:%dmv CheckHighVol = %d, CheckLowVol = %d\n", nVoltage_SRS,nNumCheckHighVol,nNumCheckLowVol);
		}
		else
		{
			nCountNullCheckVol++;
			//printf("check adc voltage count = %d\n",nCountNullCheckVol);
		}
		
		//30秒次数未检测到电压值，则认为碰撞硬线已断开
		if(nCountNullCheckVol >= 750) 
		{
			nCountNullCheckVol = 0;
			p_FAWACPInfo_Handle->voltageFaultSRSState = 1;//SRS硬线断开(检测电压值)
		}
		else
		{
			p_FAWACPInfo_Handle->voltageFaultSRSState = 0;//SRS硬线断开(检测电压值)
		}
		
		//正常输出
		if(nNumCheckHighVol == 5 && nNumCheckLowVol == 1)
		{
			nNumCheckHighVol = 0;
			nNumCheckLowVol  = 0;
			nPeriodCheckPWM  = 0;			
			//printf("^^^^^^^Deployment Cammand SRS is normal\n");
		}
		//碰撞周期
		if(nNumCheckHighVol == 1 && nNumCheckLowVol == 5)
		{
			nPeriodCheckPWM++;
			//printf("^^^^^^^Deployment Cammand SRS nPeriodCheckPWM = %d ^^^^^^^\n",nPeriodCheckPWM);
		}
		
		if(nPeriodCheckPWM >= 20)
		{
			nNumCheckHighVol = 0;
			nNumCheckLowVol  = 0;
			nPeriodCheckPWM  = 0;
			{
				if(CFAWACP::cfawacp->m_loginState == 2 && CFAWACP::cfawacp->m_ConnectedState == true)
				{
					pmcuUart->reportEventCmd(CFAWACP::cfawacp->timingReportingData, 1, ACPApp_EmergencyDataID);
				}
				for(iLoop = 0; iLoop < 3; iLoop++)
	 			{
	 				if(0 == voiceCall(p_FAWACPInfo_Handle->RemoteDeviceConfigInfo[0].EmergedCall))
						break;
	 		 	}
			}			
			printf("^^^^^^^Deployment Cammand SRS ECall^^^^^^^\n");
		}
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
	MCULOG("cmdId:%02x \n",cmdId);
	
    switch (cmdId)
    {
		case START_SYNC_CMD:	//0x00
			unpack_syncParameter(pData, datalen);
			break;
		case HEART_BEAT_CMD:	//0x10
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
		case MCU_SND_MESSAGES_ID:	//0x0B
			unpack_text_messages(pData, datalen);
			break;
		case TEXT_TO_SPEECH_ID:		//0x0C
			unpack_tts_voice(pData, datalen);
			break;
		case MCU_SND_UPGRADE_INFO:	//0x0D
			unpack_MCU_SND_Upgrade_Info(pData, datalen);
			break;
		case MCU_SND_UPGRADE_DATA:	//0x0E
			unpack_MCU_SND_Upgrade_Data(pData, datalen);
			break;
		case MCU_SND_UPGRADE_CMPL:	//0x8D
			unpack_MCU_SND_Upgrade_CMPL();
			break;
		case TBOX_REMOTECTRL_CMD:	//0x04
		{
			serialNumber = (pData[4] << 8) + pData[5];
			packProtocolData(TBOX_GENERAL_RESP, 0x04, NULL, 0, serialNumber);
			unpack_RemoteCtrl(pData, datalen);
		}
			break;
		case 0x06:
			mcu_apply_for_data(pData, datalen);
			break;
		case MCU_SND_TBOXINFO:		//0x12
			unpack_TboxConfigInfo(pData, datalen);
			break;
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
#if 1
//解析MCU发送的TBOX信息	VIN和配置码
int mcuUart::unpack_TboxConfigInfo(unsigned char *pData, unsigned int len)
{
	uint8_t *pTemp = pData;
	int fd = 0;
	uint8_t Flag = pTemp[8];
	uint8_t DataLen = pTemp[9];
	printf("DataLen == %d\n",DataLen);
	printf("===============Start Configuer TBox VIN===================\n");
	printf("VIN::");	
	for(int i = 0; i< len; i++)
		printf("%02x ", pData[i]);
	printf("\n\n\n");
	switch(Flag)
	{
		case 0x01:
			dataPool->setTboxConfigInfo(VinID, &pTemp[10], DataLen);
			break;
		case 0x02:
			dataPool->setTboxConfigInfo(ConfigCodeID, &pTemp[10], DataLen);
			break;
		default:
			break;
	}

	if(dataPool->updateTboxConfig() != 0)
	{
		MCULOG("update tboxconfiginfo error\n");
		return -1;
	}
	
	return 0;
}
#endif

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
	p_FAWACPInfo_Handle->VehicleCondData.GPSData.latitude = (uint32_t)pData[12]*1000000+
												(((float)pData[13]+((float)((pData[14]<<8)+pData[15])/100000))/60)*1000000;
	p_FAWACPInfo_Handle->VehicleCondData.GPSData.longitude = (uint32_t)pData[16]*1000000+
												(((float)pData[17]+((float)((pData[18]<<8)+pData[19])/100000))/60)*1000000;
	//MCULOG("Lat:%lu\n",p_FAWACPInfo_Handle->VehicleCondData.GPSData.latitude);
	//MCULOG("Lon:%lu\n",p_FAWACPInfo_Handle->VehicleCondData.GPSData.longitude);
	p_FAWACPInfo_Handle->VehicleCondData.GPSData.altitude = (pData[20] << 8) + pData[21];
	p_FAWACPInfo_Handle->VehicleCondData.GPSData.degree = (pData[22] << 8) + pData[23];
	//MCULOG("altitude:%lu\n",p_FAWACPInfo_Handle->VehicleCondData.GPSData.altitude);
	//MCULOG("Degree:%lu\n",p_FAWACPInfo_Handle->VehicleCondData.GPSData.degree);
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
	static uint8_t keepMileageType = 0;
	static uint8_t DoorIntrusType = 0;
	static uint8_t OutFileTyoe = 0;
	static uint8_t VehCollideType = 0;
	//************************车况数据***************************//
	p_FAWACPInfo_Handle->VehicleCondData.RemainedOil.RemainedOilGrade = pData[30] & 0x0F;//剩余油量等级
	p_FAWACPInfo_Handle->VehicleCondData.RemainedOil.RemainedOilValue = pData[31];	//剩余油量数值
	p_FAWACPInfo_Handle->VehicleCondData.Odometer = (pData[32] << 24) + (pData[33] << 16) + (pData[34] << 8) + pData[35];//总里程Last Value
	p_FAWACPInfo_Handle->VehicleCondData.Battery = ((pData[36] << 8) & 0xFF00) | (pData[37] & 0xFF);	//蓄电池电量
	p_FAWACPInfo_Handle->VehicleCondData.CurrentSpeed = ((pData[38] << 8) & 0xFF00) | (pData[39] & 0xFF);//实时车速
	p_FAWACPInfo_Handle->VehicleCondData.LTAverageSpeed = ((pData[40] << 8) & 0xFF00) | (pData[41] & 0xFF);	//长时平均速度
	p_FAWACPInfo_Handle->VehicleCondData.STAverageSpeed = ((pData[42] << 8) & 0xFF00) | (pData[43] & 0xFF);	//短时平均速度
	p_FAWACPInfo_Handle->VehicleCondData.LTAverageOil = ((pData[44] << 8) & 0xFF00) | (pData[45] & 0xFF);//长时平均油耗
	p_FAWACPInfo_Handle->VehicleCondData.STAverageOil = ((pData[46] << 8) & 0xFF00) | (pData[47] & 0xFF);//短时平均油耗
//	MCULOG("RemainedOil == %d\n",p_FAWACPInfo_Handle->VehicleCondData.RemainedOil);
//	MCULOG("Odometer == %d\n",p_FAWACPInfo_Handle->VehicleCondData.Odometer);
//	MCULOG("Battery == %d\n",p_FAWACPInfo_Handle->VehicleCondData.Battery);
//	MCULOG("LTAverageOil == %d\n",p_FAWACPInfo_Handle->VehicleCondData.LTAverageOil);
//	MCULOG("STAverageOil == %d\n",p_FAWACPInfo_Handle->VehicleCondData.STAverageOil);
	//车门状态
	p_FAWACPInfo_Handle->VehicleCondData.CarDoorState.bitState.drivingDoor = pData[48] & 0x01;
	p_FAWACPInfo_Handle->VehicleCondData.CarDoorState.bitState.copilotDoor = (pData[48] >> 1) & 0x01;
	p_FAWACPInfo_Handle->VehicleCondData.CarDoorState.bitState.leftRearDoor = (pData[48] >> 2) & 0x01;
	p_FAWACPInfo_Handle->VehicleCondData.CarDoorState.bitState.rightRearDoor = (pData[48] >> 3) & 0x01;
	p_FAWACPInfo_Handle->VehicleCondData.CarDoorState.bitState.rearCanopy = (pData[48] >> 4) & 0x01;
	p_FAWACPInfo_Handle->VehicleCondData.CarDoorState.bitState.engineCover = (pData[48] >> 5) & 0x01;
//	MCULOG("drivingDoor == %d\n",p_FAWACPInfo_Handle->VehicleCondData.CarDoorState.bitState.drivingDoor);
//	MCULOG("copilotDoor == %d\n",p_FAWACPInfo_Handle->VehicleCondData.CarDoorState.bitState.copilotDoor);
//	MCULOG("leftRearDoor == %d\n",p_FAWACPInfo_Handle->VehicleCondData.CarDoorState.bitState.leftRearDoor);
//	MCULOG("rightRearDoor == %d\n",p_FAWACPInfo_Handle->VehicleCondData.CarDoorState.bitState.rightRearDoor);
//	MCULOG("rearCanopy == %d\n",p_FAWACPInfo_Handle->VehicleCondData.CarDoorState.bitState.rearCanopy);
//	MCULOG("engineCover == %d\n",p_FAWACPInfo_Handle->VehicleCondData.CarDoorState.bitState.engineCover);
	//车锁状态
	p_FAWACPInfo_Handle->VehicleCondData.CarLockState.bitState.rightRearLock = pData[49] & 0x03;
	p_FAWACPInfo_Handle->VehicleCondData.CarLockState.bitState.leftRearLock = (pData[49] >> 2) & 0x03;
	p_FAWACPInfo_Handle->VehicleCondData.CarLockState.bitState.copilotLock = (pData[49] >> 4) & 0x03;
	p_FAWACPInfo_Handle->VehicleCondData.CarLockState.bitState.drivingLock = (pData[49] >> 6) & 0x03;
//	MCULOG("rightRearLock == %d\n",p_FAWACPInfo_Handle->VehicleCondData.CarLockState.bitState.rightRearLock);
//	MCULOG("leftRearLock == %d\n",p_FAWACPInfo_Handle->VehicleCondData.CarLockState.bitState.leftRearLock);
//	MCULOG("copilotLock == %d\n",p_FAWACPInfo_Handle->VehicleCondData.CarLockState.bitState.copilotLock);
//	MCULOG("drivingLock == %d\n",p_FAWACPInfo_Handle->VehicleCondData.CarLockState.bitState.drivingLock);
	//天窗状态
	p_FAWACPInfo_Handle->VehicleCondData.sunroofState = pData[50];
//	MCULOG("sunroofState == %d\n",p_FAWACPInfo_Handle->VehicleCondData.sunroofState);
	//车窗状态数据
	p_FAWACPInfo_Handle->VehicleCondData.WindowState.bitState.leftFrontWindow = pData[52] & 0x07;
	p_FAWACPInfo_Handle->VehicleCondData.WindowState.bitState.rightFrontWindow = (pData[52] >> 3) & 0x07;
	p_FAWACPInfo_Handle->VehicleCondData.WindowState.bitState.leftRearWindow = ((pData[52] >> 6) & 0x03) + ((pData[51] << 2) & 0x04);
	p_FAWACPInfo_Handle->VehicleCondData.WindowState.bitState.rightRearWindow = ((pData[51] >> 1) & 0x07);
//	MCULOG("leftFrontWindow == %d\n",p_FAWACPInfo_Handle->VehicleCondData.WindowState.bitState.leftFrontWindow );
//	MCULOG("rightFrontWindow == %d\n",p_FAWACPInfo_Handle->VehicleCondData.WindowState.bitState.rightFrontWindow );
//	MCULOG("leftRearWindow == %d\n",p_FAWACPInfo_Handle->VehicleCondData.WindowState.bitState.leftRearWindow );
//	MCULOG("rightRearWindow == %d\n",p_FAWACPInfo_Handle->VehicleCondData.WindowState.bitState.rightRearWindow );
	//车灯状态数据
	p_FAWACPInfo_Handle->VehicleCondData.CarlampState.bitState.headlights = pData[53] & 0x01;
	p_FAWACPInfo_Handle->VehicleCondData.CarlampState.bitState.positionlights = (pData[53] >> 1) & 0x01;
	p_FAWACPInfo_Handle->VehicleCondData.CarlampState.bitState.nearlights = (pData[53] >> 2) & 0x01;
	p_FAWACPInfo_Handle->VehicleCondData.CarlampState.bitState.rearfoglights = (pData[53] >> 3) & 0x01;
	p_FAWACPInfo_Handle->VehicleCondData.CarlampState.bitState.frontfoglights = (pData[53] >> 4) & 0x01;
//	MCULOG("headlights == %d\n",p_FAWACPInfo_Handle->VehicleCondData.CarlampState.bitState.headlights );
//	MCULOG("positionlights == %d\n",p_FAWACPInfo_Handle->VehicleCondData.CarlampState.bitState.positionlights );
//	MCULOG("nearlights == %d\n",p_FAWACPInfo_Handle->VehicleCondData.CarlampState.bitState.nearlights );
//	MCULOG("rearfoglights == %d\n",p_FAWACPInfo_Handle->VehicleCondData.CarlampState.bitState.rearfoglights );
//	MCULOG("frontfoglights == %d\n",p_FAWACPInfo_Handle->VehicleCondData.CarlampState.bitState.frontfoglights );
	//轮胎信息数据
	p_FAWACPInfo_Handle->VehicleCondData.TyreState.rightrearTyrePress = pData[61];
	p_FAWACPInfo_Handle->VehicleCondData.TyreState.leftrearTyrePress = pData[60];
	p_FAWACPInfo_Handle->VehicleCondData.TyreState.rightfrontTyrePress = pData[59];
	p_FAWACPInfo_Handle->VehicleCondData.TyreState.leftfrontTyrePress = pData[58];
	p_FAWACPInfo_Handle->VehicleCondData.TyreState.rightrearTemperature = pData[57];
	p_FAWACPInfo_Handle->VehicleCondData.TyreState.leftrearTemperature = pData[56];
	p_FAWACPInfo_Handle->VehicleCondData.TyreState.rightfrontTemperature = pData[55];
	p_FAWACPInfo_Handle->VehicleCondData.TyreState.leftfrontTemperature = pData[54];
	//TBox_MCU版本
	memcpy(p_FAWACPInfo_Handle->VehicleCondData.VerTboxMCU, &pData[62], 12);
	//发动机状态
	p_FAWACPInfo_Handle->VehicleCondData.EngineState = pData[74] & 0x07;
	//实时方向盘转角数据
	p_FAWACPInfo_Handle->VehicleCondData.WheelState.bitState.wheeldegree = ((pData[75] << 7) & 0x7F00) | (pData[76] & 0xFF);
	p_FAWACPInfo_Handle->VehicleCondData.WheelState.bitState.wheeldirection = (pData[75] >> 7) & 0x01;
	//发动机转速
	p_FAWACPInfo_Handle->VehicleCondData.EngineSpeed = (pData[77] << 8) + pData[78];
	//档位信息
	p_FAWACPInfo_Handle->VehicleCondData.Gearstate = pData[79] & 0x0F;
	//手刹状态
	p_FAWACPInfo_Handle->VehicleCondData.HandbrakeState = pData[80] & 0x03;
	//电子驻车状态
	p_FAWACPInfo_Handle->VehicleCondData.ParkingState = pData[81] & 0x03;
	//供电模式
	p_FAWACPInfo_Handle->VehicleCondData.Powersupplymode = pData[82] & 0x0F;
	//安全带状态
	p_FAWACPInfo_Handle->VehicleCondData.Safetybeltstate = pData[83] & 0x03;
	//剩余保养里程
	p_FAWACPInfo_Handle->VehicleCondData.RemainUpkeepMileage = (pData[84] << 8) + pData[85];
	if(keepMileageType == 0 && CFAWACP::cfawacp->m_loginState == 2){
		if(p_FAWACPInfo_Handle->VehicleCondData.RemainUpkeepMileage <= 500){
			reportEventCmd(CFAWACP::cfawacp->timingReportingData, 3, ACPApp_VehicleAlarmID);
			keepMileageType = 1;
		}
	}
	//空调相关信息
	p_FAWACPInfo_Handle->VehicleCondData.AirconditionerInfo.airconditionerState = pData[88] & 0x01;
	p_FAWACPInfo_Handle->VehicleCondData.AirconditionerInfo.compressorState = (pData[88] >> 1) & 0x01;
	p_FAWACPInfo_Handle->VehicleCondData.AirconditionerInfo.autoState = (pData[88] >> 2) & 0x01;
	p_FAWACPInfo_Handle->VehicleCondData.AirconditionerInfo.defrostState = (pData[88] >> 3) & 0x01;
	p_FAWACPInfo_Handle->VehicleCondData.AirconditionerInfo.inOutCirculateState = (pData[88] >> 4) & 0x01;
	p_FAWACPInfo_Handle->VehicleCondData.AirconditionerInfo.blowingLevel = (pData[88] >> 5) & 0x07;
	p_FAWACPInfo_Handle->VehicleCondData.AirconditionerInfo.blowingMode = pData[87] & 0x07;
	p_FAWACPInfo_Handle->VehicleCondData.AirconditionerInfo.Temperature = ((pData[87] >> 3) & 0x1F) + ((pData[86] & 0x03) << 5);
	//持续时间信息
	p_FAWACPInfo_Handle->VehicleCondData.KeepingstateTime.accelerateTime = pData[93];
	p_FAWACPInfo_Handle->VehicleCondData.KeepingstateTime.decelerateTime = pData[92];
	p_FAWACPInfo_Handle->VehicleCondData.KeepingstateTime.wheelTime = pData[91];
	p_FAWACPInfo_Handle->VehicleCondData.KeepingstateTime.overspeedTime = ((pData[89] << 8) & 0xFF00) | pData[90];
	//动力电池
	p_FAWACPInfo_Handle->VehicleCondData.PowerCellsState.BatAveraTempera = pData[94];
	p_FAWACPInfo_Handle->VehicleCondData.PowerCellsState.elecTempera = pData[95];
	p_FAWACPInfo_Handle->VehicleCondData.PowerCellsState.elecSOH = pData[96];
	p_FAWACPInfo_Handle->VehicleCondData.PowerCellsState.quantity = pData[97];
	p_FAWACPInfo_Handle->VehicleCondData.PowerCellsState.electricity = ((pData[98] << 8) & 0xFF00) | (pData[99] & 0xFF);
	p_FAWACPInfo_Handle->VehicleCondData.PowerCellsState.voltage = ((pData[100] << 8) & 0xFF00) | (pData[101] & 0xFF);
	//充电状态数据
	p_FAWACPInfo_Handle->VehicleCondData.ChargeState.chargeState = pData[104];
	p_FAWACPInfo_Handle->VehicleCondData.ChargeState.remainChargeTime = pData[103];
	p_FAWACPInfo_Handle->VehicleCondData.ChargeState.chargeMode = pData[102] & 0x03;
	//TBOX-OS版本
	memcpy(p_FAWACPInfo_Handle->VehicleCondData.VerTboxOS, &pData[105], sizeof(p_FAWACPInfo_Handle->VehicleCondData.VerTboxOS));
	//IVI版本
	memcpy(p_FAWACPInfo_Handle->VehicleCondData.VerIVI, &pData[117], sizeof(p_FAWACPInfo_Handle->VehicleCondData.VerIVI));
	//充电枪连接状态
	p_FAWACPInfo_Handle->VehicleCondData.ChargeConnectState = pData[133] & 0x0F;
	//制动踏板开关
	p_FAWACPInfo_Handle->VehicleCondData.BrakePedalSwitch = pData[134] & 0x03;
	//加速踏板开关
	p_FAWACPInfo_Handle->VehicleCondData.AcceleraPedalSwitch = pData[135];
	//YAW传感器信号
	p_FAWACPInfo_Handle->VehicleCondData.YaWSensorInfoSwitch.TransverseAccele = ((pData[136] << 8) & 0x0F00) | pData[137];
	p_FAWACPInfo_Handle->VehicleCondData.YaWSensorInfoSwitch.LongituAccele = ((pData[138] << 8) & 0x0F00) | pData[139];
	p_FAWACPInfo_Handle->VehicleCondData.YaWSensorInfoSwitch.YawVelocity = ((pData[140] << 8) & 0x0F00) | pData[141];
	//环境温度
	p_FAWACPInfo_Handle->VehicleCondData.AmbientTemperat.AmbientTemperat = ((pData[142] << 8) & 0x0700) | pData[143];
	//纯电动继电器及线圈状态
	p_FAWACPInfo_Handle->VehicleCondData.PureElecRelayState.MainPositRelayCoilState = pData[144] & 0x01;
	p_FAWACPInfo_Handle->VehicleCondData.PureElecRelayState.MainNegaRelayCoilState = (pData[144] >> 1) & 0x01;
	p_FAWACPInfo_Handle->VehicleCondData.PureElecRelayState.PrefillRelayCoilState = (pData[144] >> 2) & 0x01;
	p_FAWACPInfo_Handle->VehicleCondData.PureElecRelayState.RechargePositRelayCoilState = (pData[144] >> 3) & 0x01;
	p_FAWACPInfo_Handle->VehicleCondData.PureElecRelayState.RechargeNegaRelayCoilState = (pData[144] >> 4) & 0x01;
	//剩余续航里程
	p_FAWACPInfo_Handle->VehicleCondData.ResidualRange = ((pData[145] << 8) & 0xFF00) | pData[146];
	//新能源热管理请求
	p_FAWACPInfo_Handle->VehicleCondData.NewEnergyHeatManage.BatteryHeatRequest = pData[147] & 0x01;
	p_FAWACPInfo_Handle->VehicleCondData.NewEnergyHeatManage.Motor1CoolRequest = (pData[147] >> 1) & 0x07;
	p_FAWACPInfo_Handle->VehicleCondData.NewEnergyHeatManage.Motor2CoolRequest = (pData[147] >> 4) & 0x07;
	//车辆工作模式
	p_FAWACPInfo_Handle->VehicleCondData.VehWorkMode.VehWorkMode = pData[148] & 0x0F;
	//电机工作状态
	p_FAWACPInfo_Handle->VehicleCondData.MotorWorkState.Motor1Workstate = pData[149] & 0x07;
	p_FAWACPInfo_Handle->VehicleCondData.MotorWorkState.Motor2Workstate = (pData[149] >> 3) & 0x07;
	//高压系统准备状态
	p_FAWACPInfo_Handle->VehicleCondData.HighVoltageState = pData[150] & 0x03;

	//************************故障信息***************************//
	p_FAWACPInfo_Handle->AcpCodeFault.ACPCODEFAULTEMSState = pData[151];		//发动机管理系统故障
	if(p_FAWACPInfo_Handle->AcpCodeFault.ACPCODEFAULTEMSState != FaultSigan.ACPCODEFAULTEMSState){
		if(FaultSigan.ACPCODEFAULTEMSState != 0)
			reportEventCmd(CFAWACP::cfawacp->timingReportingData, 18, ACPApp_VehicleCondUploadID);
		FaultSigan.ACPCODEFAULTEMSState = p_FAWACPInfo_Handle->AcpCodeFault.ACPCODEFAULTEMSState;
	}
	p_FAWACPInfo_Handle->AcpCodeFault.ACPCODEFAULTTCUState = pData[152];		//变速箱控制单元故障
	if(p_FAWACPInfo_Handle->AcpCodeFault.ACPCODEFAULTTCUState != FaultSigan.ACPCODEFAULTTCUState){
		if(FaultSigan.ACPCODEFAULTTCUState != 0)
			reportEventCmd(CFAWACP::cfawacp->timingReportingData, 18, ACPApp_VehicleCondUploadID);
		FaultSigan.ACPCODEFAULTTCUState = p_FAWACPInfo_Handle->AcpCodeFault.ACPCODEFAULTTCUState;
	}
	p_FAWACPInfo_Handle->AcpCodeFault.ACPCODEFAULTEmissionState = pData[153];//排放系统故障
	if(p_FAWACPInfo_Handle->AcpCodeFault.ACPCODEFAULTEmissionState != FaultSigan.ACPCODEFAULTEmissionState){
		if(FaultSigan.ACPCODEFAULTEmissionState != 0)
			reportEventCmd(CFAWACP::cfawacp->timingReportingData, 18, ACPApp_VehicleCondUploadID);
		FaultSigan.ACPCODEFAULTEmissionState = p_FAWACPInfo_Handle->AcpCodeFault.ACPCODEFAULTEmissionState;
	}
	p_FAWACPInfo_Handle->AcpCodeFault.ACPCODEFAULTSRSState = pData[154];		//安全气囊系统故障
	if(p_FAWACPInfo_Handle->AcpCodeFault.ACPCODEFAULTSRSState != FaultSigan.ACPCODEFAULTSRSState){
		if(FaultSigan.ACPCODEFAULTSRSState != 0)
			reportEventCmd(CFAWACP::cfawacp->timingReportingData, 18, ACPApp_VehicleCondUploadID);
		FaultSigan.ACPCODEFAULTSRSState = p_FAWACPInfo_Handle->AcpCodeFault.ACPCODEFAULTSRSState;
	}
	p_FAWACPInfo_Handle->AcpCodeFault.ACPCODEFAULTESPState = pData[155];		//电子稳定系统故障
	if(p_FAWACPInfo_Handle->AcpCodeFault.ACPCODEFAULTESPState != FaultSigan.ACPCODEFAULTESPState){
		if(FaultSigan.ACPCODEFAULTESPState != 0)
			reportEventCmd(CFAWACP::cfawacp->timingReportingData, 18, ACPApp_VehicleCondUploadID);
		FaultSigan.ACPCODEFAULTESPState = p_FAWACPInfo_Handle->AcpCodeFault.ACPCODEFAULTESPState;
	}
	p_FAWACPInfo_Handle->AcpCodeFault.ACPCODEFAULTABSState = pData[156];		//防抱死刹车系统
	if(p_FAWACPInfo_Handle->AcpCodeFault.ACPCODEFAULTABSState != FaultSigan.ACPCODEFAULTABSState){
		if(FaultSigan.ACPCODEFAULTABSState != 0)
			reportEventCmd(CFAWACP::cfawacp->timingReportingData, 18, ACPApp_VehicleCondUploadID);
		FaultSigan.ACPCODEFAULTABSState = p_FAWACPInfo_Handle->AcpCodeFault.ACPCODEFAULTABSState;
	}
	p_FAWACPInfo_Handle->AcpCodeFault.ACPCODEFAULTEPASState = pData[157];		//电子助力转向系统
	if(p_FAWACPInfo_Handle->AcpCodeFault.ACPCODEFAULTEPASState != FaultSigan.ACPCODEFAULTEPASState){
		if(FaultSigan.ACPCODEFAULTEPASState != 0)
			reportEventCmd(CFAWACP::cfawacp->timingReportingData, 18, ACPApp_VehicleCondUploadID);
		FaultSigan.ACPCODEFAULTEPASState = p_FAWACPInfo_Handle->AcpCodeFault.ACPCODEFAULTEPASState;
	}
	p_FAWACPInfo_Handle->AcpCodeFault.ACPCODEFAULTOilPressureState = pData[158];	//机油压力报警
	if(p_FAWACPInfo_Handle->AcpCodeFault.ACPCODEFAULTOilPressureState != FaultSigan.ACPCODEFAULTOilPressureState){
		if(FaultSigan.ACPCODEFAULTOilPressureState != 0)
			reportEventCmd(CFAWACP::cfawacp->timingReportingData, 18, ACPApp_VehicleCondUploadID);
		FaultSigan.ACPCODEFAULTOilPressureState = p_FAWACPInfo_Handle->AcpCodeFault.ACPCODEFAULTOilPressureState;
	}
	p_FAWACPInfo_Handle->AcpCodeFault.ACPCODEFAULTLowOilIDState = pData[159];	//油量低报警
	if(p_FAWACPInfo_Handle->AcpCodeFault.ACPCODEFAULTLowOilIDState != FaultSigan.ACPCODEFAULTLowOilIDState){
		if(FaultSigan.ACPCODEFAULTLowOilIDState != 0)
			reportEventCmd(CFAWACP::cfawacp->timingReportingData, 4, ACPApp_VehicleAlarmID);
		FaultSigan.ACPCODEFAULTLowOilIDState = p_FAWACPInfo_Handle->AcpCodeFault.ACPCODEFAULTLowOilIDState;
	}
	
	p_FAWACPInfo_Handle->AcpCodeFault.ACPCODEFAULTBrakeFluidLevelState = pData[160];	//制动液位报警
	if(p_FAWACPInfo_Handle->AcpCodeFault.ACPCODEFAULTBrakeFluidLevelState != FaultSigan.ACPCODEFAULTBrakeFluidLevelState){
		if(FaultSigan.ACPCODEFAULTBrakeFluidLevelState != 0)
			reportEventCmd(CFAWACP::cfawacp->timingReportingData, 18, ACPApp_VehicleCondUploadID);
		FaultSigan.ACPCODEFAULTBrakeFluidLevelState = p_FAWACPInfo_Handle->AcpCodeFault.ACPCODEFAULTBrakeFluidLevelState;
	}
	p_FAWACPInfo_Handle->AcpCodeFault.ACPCODEFAULTBBWState = pData[161];		//制动系统故障
	if(p_FAWACPInfo_Handle->AcpCodeFault.ACPCODEFAULTBBWState != FaultSigan.ACPCODEFAULTBBWState){
		if(FaultSigan.ACPCODEFAULTBBWState != 0)
			reportEventCmd(CFAWACP::cfawacp->timingReportingData, 18, ACPApp_VehicleCondUploadID);
		FaultSigan.ACPCODEFAULTBBWState = p_FAWACPInfo_Handle->AcpCodeFault.ACPCODEFAULTBBWState;
	}
	p_FAWACPInfo_Handle->AcpCodeFault.ACPCODEFAULTTPMSState = pData[162];		//胎压系统故障
	if(p_FAWACPInfo_Handle->AcpCodeFault.ACPCODEFAULTTPMSState != FaultSigan.ACPCODEFAULTTPMSState){
		if(FaultSigan.ACPCODEFAULTTPMSState != 0)
			reportEventCmd(CFAWACP::cfawacp->timingReportingData, 18, ACPApp_VehicleCondUploadID);
		FaultSigan.ACPCODEFAULTTPMSState = p_FAWACPInfo_Handle->AcpCodeFault.ACPCODEFAULTTPMSState;
	}
	p_FAWACPInfo_Handle->AcpCodeFault.ACPCODEFAULTSTTState = pData[163];		//启停系统故障
	if(p_FAWACPInfo_Handle->AcpCodeFault.ACPCODEFAULTSTTState != FaultSigan.ACPCODEFAULTSTTState){
		if(FaultSigan.ACPCODEFAULTSTTState != 0)
			reportEventCmd(CFAWACP::cfawacp->timingReportingData, 18, ACPApp_VehicleCondUploadID);
		FaultSigan.ACPCODEFAULTSTTState = p_FAWACPInfo_Handle->AcpCodeFault.ACPCODEFAULTSTTState;
	}
	p_FAWACPInfo_Handle->AcpCodeFault.ACPCODEFAULTExtLightState = pData[164];//外部灯光故障
	if(p_FAWACPInfo_Handle->AcpCodeFault.ACPCODEFAULTExtLightState != FaultSigan.ACPCODEFAULTExtLightState){
		if(FaultSigan.ACPCODEFAULTExtLightState != 0)
			reportEventCmd(CFAWACP::cfawacp->timingReportingData, 18, ACPApp_VehicleCondUploadID);
		FaultSigan.ACPCODEFAULTExtLightState = p_FAWACPInfo_Handle->AcpCodeFault.ACPCODEFAULTExtLightState;
	}
	p_FAWACPInfo_Handle->AcpCodeFault.ACPCODEFAULTESCLState = pData[165];		//电子转向柱锁故障
	if(p_FAWACPInfo_Handle->AcpCodeFault.ACPCODEFAULTESCLState != FaultSigan.ACPCODEFAULTESCLState){
		if(FaultSigan.ACPCODEFAULTESCLState != 0)
			reportEventCmd(CFAWACP::cfawacp->timingReportingData, 18, ACPApp_VehicleCondUploadID);
		FaultSigan.ACPCODEFAULTESCLState = p_FAWACPInfo_Handle->AcpCodeFault.ACPCODEFAULTESCLState;
	}
	p_FAWACPInfo_Handle->AcpCodeFault.ACPCODEFAULTEngineOverwaterState = pData[166];//发动机水温过高报警
	if(p_FAWACPInfo_Handle->AcpCodeFault.ACPCODEFAULTEngineOverwaterState != FaultSigan.ACPCODEFAULTEngineOverwaterState){
		if(FaultSigan.ACPCODEFAULTEngineOverwaterState != 0)
			reportEventCmd(CFAWACP::cfawacp->timingReportingData, 18, ACPApp_VehicleCondUploadID);
		FaultSigan.ACPCODEFAULTEngineOverwaterState = p_FAWACPInfo_Handle->AcpCodeFault.ACPCODEFAULTEngineOverwaterState;
	}
	p_FAWACPInfo_Handle->AcpCodeFault.ACPCODEFAULTElecParkUnitState = pData[167];	//电子驻车单元系统故障
	if(p_FAWACPInfo_Handle->AcpCodeFault.ACPCODEFAULTElecParkUnitState != FaultSigan.ACPCODEFAULTElecParkUnitState){
		if(FaultSigan.ACPCODEFAULTElecParkUnitState != 0)
			reportEventCmd(CFAWACP::cfawacp->timingReportingData, 18, ACPApp_VehicleCondUploadID);
		FaultSigan.ACPCODEFAULTElecParkUnitState = p_FAWACPInfo_Handle->AcpCodeFault.ACPCODEFAULTElecParkUnitState;
	}
	p_FAWACPInfo_Handle->AcpCodeFault.ACPCODEFAULTAHBState = pData[168];	//智能远光系统故障
	if(p_FAWACPInfo_Handle->AcpCodeFault.ACPCODEFAULTAHBState != FaultSigan.ACPCODEFAULTAHBState){
		if(FaultSigan.ACPCODEFAULTAHBState != 0)
			reportEventCmd(CFAWACP::cfawacp->timingReportingData, 18, ACPApp_VehicleCondUploadID);
		FaultSigan.ACPCODEFAULTAHBState = p_FAWACPInfo_Handle->AcpCodeFault.ACPCODEFAULTAHBState;
	}
	p_FAWACPInfo_Handle->AcpCodeFault.ACPCODEFAULTACSState = pData[169];	//自适应巡航系统故障
	if(p_FAWACPInfo_Handle->AcpCodeFault.ACPCODEFAULTACSState != FaultSigan.ACPCODEFAULTACSState){
		if(FaultSigan.ACPCODEFAULTACSState != 0)
			reportEventCmd(CFAWACP::cfawacp->timingReportingData, 18, ACPApp_VehicleCondUploadID);
		FaultSigan.ACPCODEFAULTACSState = p_FAWACPInfo_Handle->AcpCodeFault.ACPCODEFAULTACSState;
	}
	p_FAWACPInfo_Handle->AcpCodeFault.ACPCODEFAULTFCWSState = pData[170];	//前碰撞预警系统故障
	if(p_FAWACPInfo_Handle->AcpCodeFault.ACPCODEFAULTFCWSState != FaultSigan.ACPCODEFAULTFCWSState){
		if(FaultSigan.ACPCODEFAULTFCWSState != 0)
			reportEventCmd(CFAWACP::cfawacp->timingReportingData, 18, ACPApp_VehicleCondUploadID);
		FaultSigan.ACPCODEFAULTFCWSState = p_FAWACPInfo_Handle->AcpCodeFault.ACPCODEFAULTFCWSState;
	}
	p_FAWACPInfo_Handle->AcpCodeFault.ACPCODEFAULTLDWState = pData[171];	//道路偏离预警系统故障
	if(p_FAWACPInfo_Handle->AcpCodeFault.ACPCODEFAULTLDWState != FaultSigan.ACPCODEFAULTLDWState){
		if(FaultSigan.ACPCODEFAULTLDWState != 0)
			reportEventCmd(CFAWACP::cfawacp->timingReportingData, 18, ACPApp_VehicleCondUploadID);
		FaultSigan.ACPCODEFAULTLDWState = p_FAWACPInfo_Handle->AcpCodeFault.ACPCODEFAULTLDWState;
	}
	p_FAWACPInfo_Handle->AcpCodeFault.ACPCODEFAULTBlindSpotDetectState = pData[172];//盲区检测系统故障
	if(p_FAWACPInfo_Handle->AcpCodeFault.ACPCODEFAULTBlindSpotDetectState != FaultSigan.ACPCODEFAULTBlindSpotDetectState){
		if(FaultSigan.ACPCODEFAULTBlindSpotDetectState != 0)
			reportEventCmd(CFAWACP::cfawacp->timingReportingData, 18, ACPApp_VehicleCondUploadID);
		FaultSigan.ACPCODEFAULTBlindSpotDetectState = p_FAWACPInfo_Handle->AcpCodeFault.ACPCODEFAULTBlindSpotDetectState;
	}
	p_FAWACPInfo_Handle->AcpCodeFault.ACPCODEFAULTAirconManualState = pData[173];	//空调人为操作
	if(p_FAWACPInfo_Handle->AcpCodeFault.ACPCODEFAULTAirconManualState != FaultSigan.ACPCODEFAULTAirconManualState){
		if(FaultSigan.ACPCODEFAULTAirconManualState != 0)
			reportEventCmd(CFAWACP::cfawacp->timingReportingData, 18, ACPApp_VehicleCondUploadID);
		FaultSigan.ACPCODEFAULTAirconManualState = p_FAWACPInfo_Handle->AcpCodeFault.ACPCODEFAULTAirconManualState;
	}
	p_FAWACPInfo_Handle->AcpCodeFault.ACPCODEFAULTHVSystemState = pData[174];	//高压系统故障
	if(p_FAWACPInfo_Handle->AcpCodeFault.ACPCODEFAULTHVSystemState != FaultSigan.ACPCODEFAULTHVSystemState){
		if(FaultSigan.ACPCODEFAULTHVSystemState != 0)
			reportEventCmd(CFAWACP::cfawacp->timingReportingData, 18, ACPApp_VehicleCondUploadID);
		FaultSigan.ACPCODEFAULTHVSystemState = p_FAWACPInfo_Handle->AcpCodeFault.ACPCODEFAULTHVSystemState;
	}
	p_FAWACPInfo_Handle->AcpCodeFault.ACPCODEFAULTHVInsulateState = pData[175];	//高压绝缘故障
	if(p_FAWACPInfo_Handle->AcpCodeFault.ACPCODEFAULTHVInsulateState != FaultSigan.ACPCODEFAULTHVInsulateState){
		if(FaultSigan.ACPCODEFAULTHVInsulateState != 0)
			reportEventCmd(CFAWACP::cfawacp->timingReportingData, 18, ACPApp_VehicleCondUploadID);
		FaultSigan.ACPCODEFAULTHVInsulateState = p_FAWACPInfo_Handle->AcpCodeFault.ACPCODEFAULTHVInsulateState;
	}
	p_FAWACPInfo_Handle->AcpCodeFault.ACPCODEFAULTHVILState = pData[176];	//高压互锁故障
	if(p_FAWACPInfo_Handle->AcpCodeFault.ACPCODEFAULTHVILState != FaultSigan.ACPCODEFAULTHVILState){
		if(FaultSigan.ACPCODEFAULTHVILState != 0)
			reportEventCmd(CFAWACP::cfawacp->timingReportingData, 18, ACPApp_VehicleCondUploadID);
		FaultSigan.ACPCODEFAULTHVILState = p_FAWACPInfo_Handle->AcpCodeFault.ACPCODEFAULTHVILState;
	}
	p_FAWACPInfo_Handle->AcpCodeFault.ACPCODEFAULTEVCellState = pData[177];	//动力电池故障
	if(p_FAWACPInfo_Handle->AcpCodeFault.ACPCODEFAULTEVCellState != FaultSigan.ACPCODEFAULTEVCellState){
		if(FaultSigan.ACPCODEFAULTEVCellState != 0)
			reportEventCmd(CFAWACP::cfawacp->timingReportingData, 18, ACPApp_VehicleCondUploadID);
		FaultSigan.ACPCODEFAULTEVCellState = p_FAWACPInfo_Handle->AcpCodeFault.ACPCODEFAULTEVCellState;
	}
	p_FAWACPInfo_Handle->AcpCodeFault.ACPCODEFAULTPowerMotorState = pData[178];	//动力电机故障
	if(p_FAWACPInfo_Handle->AcpCodeFault.ACPCODEFAULTPowerMotorState != FaultSigan.ACPCODEFAULTPowerMotorState){
		if(FaultSigan.ACPCODEFAULTPowerMotorState != 0)
			reportEventCmd(CFAWACP::cfawacp->timingReportingData, 18, ACPApp_VehicleCondUploadID);
		FaultSigan.ACPCODEFAULTPowerMotorState = p_FAWACPInfo_Handle->AcpCodeFault.ACPCODEFAULTPowerMotorState;
	}
	p_FAWACPInfo_Handle->AcpCodeFault.ACPCODEFAULTEParkState = pData[179];	//E-Park故障
	if(p_FAWACPInfo_Handle->AcpCodeFault.ACPCODEFAULTEParkState != FaultSigan.ACPCODEFAULTEParkState){
		if(FaultSigan.ACPCODEFAULTEParkState != 0)
			reportEventCmd(CFAWACP::cfawacp->timingReportingData, 18, ACPApp_VehicleCondUploadID);
		FaultSigan.ACPCODEFAULTEParkState = p_FAWACPInfo_Handle->AcpCodeFault.ACPCODEFAULTEParkState;
	}
	p_FAWACPInfo_Handle->AcpCodeFault.ACPCODEFAULTEVCellLowBatteryState = pData[180];	//动力电池电量过低报警
	if(p_FAWACPInfo_Handle->AcpCodeFault.ACPCODEFAULTEVCellLowBatteryState != FaultSigan.ACPCODEFAULTEVCellLowBatteryState){
		if(FaultSigan.ACPCODEFAULTEVCellLowBatteryState != 0)
			reportEventCmd(CFAWACP::cfawacp->timingReportingData, 18, ACPApp_VehicleCondUploadID);
		FaultSigan.ACPCODEFAULTEVCellLowBatteryState = p_FAWACPInfo_Handle->AcpCodeFault.ACPCODEFAULTEVCellLowBatteryState;
	}
	p_FAWACPInfo_Handle->AcpCodeFault.ACPCODEFAULTEVCellOverTemperateState = pData[181];	//动力电池温度过高报警
	if(p_FAWACPInfo_Handle->AcpCodeFault.ACPCODEFAULTEVCellOverTemperateState != FaultSigan.ACPCODEFAULTEVCellOverTemperateState){
		if(FaultSigan.ACPCODEFAULTEVCellOverTemperateState != 0)
			reportEventCmd(CFAWACP::cfawacp->timingReportingData, 18, ACPApp_VehicleCondUploadID);
		FaultSigan.ACPCODEFAULTEVCellOverTemperateState = p_FAWACPInfo_Handle->AcpCodeFault.ACPCODEFAULTEVCellOverTemperateState;
	}
	p_FAWACPInfo_Handle->AcpCodeFault.ACPCODEFAULTPowerMotorOverTemperateState = pData[182];//动力电机温度过高报警
	if(p_FAWACPInfo_Handle->AcpCodeFault.ACPCODEFAULTPowerMotorOverTemperateState != FaultSigan.ACPCODEFAULTPowerMotorOverTemperateState){
		if(FaultSigan.ACPCODEFAULTPowerMotorOverTemperateState != 0)
			reportEventCmd(CFAWACP::cfawacp->timingReportingData, 18, ACPApp_VehicleCondUploadID);
		FaultSigan.ACPCODEFAULTPowerMotorOverTemperateState = p_FAWACPInfo_Handle->AcpCodeFault.ACPCODEFAULTPowerMotorOverTemperateState;
	}
	p_FAWACPInfo_Handle->AcpCodeFault.ACPCODEFAULTConstantSpeedSystemFailState = pData[183];//定速巡航系统故障
	if(p_FAWACPInfo_Handle->AcpCodeFault.ACPCODEFAULTConstantSpeedSystemFailState != FaultSigan.ACPCODEFAULTConstantSpeedSystemFailState){
		if(FaultSigan.ACPCODEFAULTConstantSpeedSystemFailState != 0)
			reportEventCmd(CFAWACP::cfawacp->timingReportingData, 18, ACPApp_VehicleCondUploadID);
		FaultSigan.ACPCODEFAULTConstantSpeedSystemFailState = p_FAWACPInfo_Handle->AcpCodeFault.ACPCODEFAULTConstantSpeedSystemFailState;
	}
	p_FAWACPInfo_Handle->AcpCodeFault.ACPCODEFAULTChargerFaultState = pData[184];//充电机故障
	if(p_FAWACPInfo_Handle->AcpCodeFault.ACPCODEFAULTChargerFaultState != FaultSigan.ACPCODEFAULTChargerFaultState){
		if(FaultSigan.ACPCODEFAULTChargerFaultState != 0)
			reportEventCmd(CFAWACP::cfawacp->timingReportingData, 18, ACPApp_VehicleCondUploadID);
		FaultSigan.ACPCODEFAULTChargerFaultState = p_FAWACPInfo_Handle->AcpCodeFault.ACPCODEFAULTChargerFaultState;
	}
	p_FAWACPInfo_Handle->AcpCodeFault.ACPCODEFAULTAirFailureState = pData[185];	 //空调故障
	if(p_FAWACPInfo_Handle->AcpCodeFault.ACPCODEFAULTAirFailureState != FaultSigan.ACPCODEFAULTAirFailureState){
		if(FaultSigan.ACPCODEFAULTAirFailureState != 0)
			reportEventCmd(CFAWACP::cfawacp->timingReportingData, 18, ACPApp_VehicleCondUploadID);
		FaultSigan.ACPCODEFAULTAirFailureState = p_FAWACPInfo_Handle->AcpCodeFault.ACPCODEFAULTAirFailureState;
	}
	p_FAWACPInfo_Handle->AcpCodeFault.ACPCODEFAULTAlternateAuxSystemFailState = pData[186];	 //换道辅助系统故障
	if(p_FAWACPInfo_Handle->AcpCodeFault.ACPCODEFAULTAlternateAuxSystemFailState != FaultSigan.ACPCODEFAULTAlternateAuxSystemFailState){
		if(FaultSigan.ACPCODEFAULTAlternateAuxSystemFailState != 0)
			reportEventCmd(CFAWACP::cfawacp->timingReportingData, 18, ACPApp_VehicleCondUploadID);
		FaultSigan.ACPCODEFAULTAlternateAuxSystemFailState = p_FAWACPInfo_Handle->AcpCodeFault.ACPCODEFAULTAlternateAuxSystemFailState;
	}
	p_FAWACPInfo_Handle->AcpCodeFault.ACPCODEFAULTAutoEmergeSystemFailState = pData[187];	 //自动紧急制动系统故障
	if(p_FAWACPInfo_Handle->AcpCodeFault.ACPCODEFAULTAutoEmergeSystemFailState != FaultSigan.ACPCODEFAULTAutoEmergeSystemFailState){
		if(FaultSigan.ACPCODEFAULTAutoEmergeSystemFailState != 0)
			reportEventCmd(CFAWACP::cfawacp->timingReportingData, 18, ACPApp_VehicleCondUploadID);
		FaultSigan.ACPCODEFAULTAutoEmergeSystemFailState = p_FAWACPInfo_Handle->AcpCodeFault.ACPCODEFAULTAutoEmergeSystemFailState;
	}
	p_FAWACPInfo_Handle->AcpCodeFault.ACPCODEFAULTReverRadarSystemFailState = pData[188];	 //倒车雷达系统故障
	if(p_FAWACPInfo_Handle->AcpCodeFault.ACPCODEFAULTReverRadarSystemFailState != FaultSigan.ACPCODEFAULTReverRadarSystemFailState){
		if(FaultSigan.ACPCODEFAULTReverRadarSystemFailState != 0)
			reportEventCmd(CFAWACP::cfawacp->timingReportingData, 18, ACPApp_VehicleCondUploadID);
		FaultSigan.ACPCODEFAULTReverRadarSystemFailState = p_FAWACPInfo_Handle->AcpCodeFault.ACPCODEFAULTReverRadarSystemFailState;
	}
	p_FAWACPInfo_Handle->AcpCodeFault.ACPCODEFAULTElecGearSystemFailState = pData[189];	 //电子换挡器系统故障
	if(p_FAWACPInfo_Handle->AcpCodeFault.ACPCODEFAULTElecGearSystemFailState != FaultSigan.ACPCODEFAULTElecGearSystemFailState){
		if(FaultSigan.ACPCODEFAULTElecGearSystemFailState != 0)
			reportEventCmd(CFAWACP::cfawacp->timingReportingData, 18, ACPApp_VehicleCondUploadID);
		FaultSigan.ACPCODEFAULTElecGearSystemFailState = p_FAWACPInfo_Handle->AcpCodeFault.ACPCODEFAULTElecGearSystemFailState;
	}
	p_FAWACPInfo_Handle->AcpCodeFault.ACPCODEFAULTTyreAlarmState.LeftFrontTirePressAlarm = pData[193] & 0x07;	 //左前胎压报警
	if(p_FAWACPInfo_Handle->AcpCodeFault.ACPCODEFAULTTyreAlarmState.LeftFrontTirePressAlarm != FaultSigan.LeftFrontTirePressAlarm){
		if(FaultSigan.LeftFrontTirePressAlarm != 0)
			reportEventCmd(CFAWACP::cfawacp->timingReportingData, 18, ACPApp_VehicleCondUploadID);
		FaultSigan.LeftFrontTirePressAlarm = p_FAWACPInfo_Handle->AcpCodeFault.ACPCODEFAULTTyreAlarmState.LeftFrontTirePressAlarm;
	}
	p_FAWACPInfo_Handle->AcpCodeFault.ACPCODEFAULTTyreAlarmState.LeftFrontTireTempAlarm = (pData[193] >> 3) & 0x07;	 //左前胎温报警
	if(p_FAWACPInfo_Handle->AcpCodeFault.ACPCODEFAULTTyreAlarmState.LeftFrontTireTempAlarm != FaultSigan.LeftFrontTireTempAlarm){
		if(FaultSigan.LeftFrontTireTempAlarm != 0)
			reportEventCmd(CFAWACP::cfawacp->timingReportingData, 18, ACPApp_VehicleCondUploadID);
		FaultSigan.LeftFrontTireTempAlarm = p_FAWACPInfo_Handle->AcpCodeFault.ACPCODEFAULTTyreAlarmState.LeftFrontTireTempAlarm;
	}
	p_FAWACPInfo_Handle->AcpCodeFault.ACPCODEFAULTTyreAlarmState.RightFrontTirePressAlarm = pData[192] & 0x07;	 //右前胎压报警
	if(p_FAWACPInfo_Handle->AcpCodeFault.ACPCODEFAULTTyreAlarmState.RightFrontTirePressAlarm != FaultSigan.RightFrontTirePressAlarm){
		if(FaultSigan.RightFrontTirePressAlarm != 0)
			reportEventCmd(CFAWACP::cfawacp->timingReportingData, 18, ACPApp_VehicleCondUploadID);
		FaultSigan.RightFrontTirePressAlarm = p_FAWACPInfo_Handle->AcpCodeFault.ACPCODEFAULTTyreAlarmState.RightFrontTirePressAlarm;
	}
	p_FAWACPInfo_Handle->AcpCodeFault.ACPCODEFAULTTyreAlarmState.RightrontTireTempAlarm = (pData[192] >> 3) & 0x07;	 //右前胎温报警
	if(p_FAWACPInfo_Handle->AcpCodeFault.ACPCODEFAULTTyreAlarmState.RightrontTireTempAlarm != FaultSigan.RightrontTireTempAlarm){
		if(FaultSigan.RightrontTireTempAlarm != 0)
			reportEventCmd(CFAWACP::cfawacp->timingReportingData, 18, ACPApp_VehicleCondUploadID);
		FaultSigan.RightrontTireTempAlarm = p_FAWACPInfo_Handle->AcpCodeFault.ACPCODEFAULTTyreAlarmState.RightrontTireTempAlarm;
	}
	p_FAWACPInfo_Handle->AcpCodeFault.ACPCODEFAULTTyreAlarmState.LeftRearTirePressAlarm = pData[191] & 0x07;	 //左后胎压报警
	if(p_FAWACPInfo_Handle->AcpCodeFault.ACPCODEFAULTTyreAlarmState.LeftRearTirePressAlarm != FaultSigan.LeftRearTirePressAlarm){
		if(FaultSigan.LeftRearTirePressAlarm != 0)
			reportEventCmd(CFAWACP::cfawacp->timingReportingData, 18, ACPApp_VehicleCondUploadID);
		FaultSigan.LeftRearTirePressAlarm = p_FAWACPInfo_Handle->AcpCodeFault.ACPCODEFAULTTyreAlarmState.LeftRearTirePressAlarm;
	}
	p_FAWACPInfo_Handle->AcpCodeFault.ACPCODEFAULTTyreAlarmState.LeftRearTireTempAlarm = (pData[191] >> 3) & 0x07;	 //左后胎温报警
	if(p_FAWACPInfo_Handle->AcpCodeFault.ACPCODEFAULTTyreAlarmState.LeftRearTireTempAlarm != FaultSigan.LeftRearTireTempAlarm){
		if(FaultSigan.LeftRearTireTempAlarm != 0)
			reportEventCmd(CFAWACP::cfawacp->timingReportingData, 18, ACPApp_VehicleCondUploadID);
		FaultSigan.LeftRearTireTempAlarm = p_FAWACPInfo_Handle->AcpCodeFault.ACPCODEFAULTTyreAlarmState.LeftRearTireTempAlarm;
	}
	p_FAWACPInfo_Handle->AcpCodeFault.ACPCODEFAULTTyreAlarmState.RightRearTirePressAlarm = pData[190] & 0x07;	 //右后胎压报警
	if(p_FAWACPInfo_Handle->AcpCodeFault.ACPCODEFAULTTyreAlarmState.RightRearTirePressAlarm != FaultSigan.RightRearTirePressAlarm){
		if(FaultSigan.RightRearTirePressAlarm != 0)
			reportEventCmd(CFAWACP::cfawacp->timingReportingData, 18, ACPApp_VehicleCondUploadID);
		FaultSigan.RightRearTirePressAlarm = p_FAWACPInfo_Handle->AcpCodeFault.ACPCODEFAULTTyreAlarmState.RightRearTirePressAlarm;
	}
	p_FAWACPInfo_Handle->AcpCodeFault.ACPCODEFAULTTyreAlarmState.RightRearTireTempAlarm = (pData[190] >> 3) & 0x07;	 //右后胎温报警
	if(p_FAWACPInfo_Handle->AcpCodeFault.ACPCODEFAULTTyreAlarmState.RightRearTireTempAlarm != FaultSigan.RightRearTireTempAlarm){
		if(FaultSigan.RightRearTireTempAlarm != 0)
			reportEventCmd(CFAWACP::cfawacp->timingReportingData, 18, ACPApp_VehicleCondUploadID);
		FaultSigan.RightRearTireTempAlarm = p_FAWACPInfo_Handle->AcpCodeFault.ACPCODEFAULTTyreAlarmState.RightRearTireTempAlarm;
	}
	p_FAWACPInfo_Handle->AcpCodeFault.ACPCODEFAULTDCDCConverterFaultState = pData[194];	 //直流直流转换器故障
	if(p_FAWACPInfo_Handle->AcpCodeFault.ACPCODEFAULTDCDCConverterFaultState != FaultSigan.ACPCODEFAULTDCDCConverterFaultState){
		if(FaultSigan.ACPCODEFAULTDCDCConverterFaultState != 0)
			reportEventCmd(CFAWACP::cfawacp->timingReportingData, 18, ACPApp_VehicleCondUploadID);
		FaultSigan.ACPCODEFAULTDCDCConverterFaultState = p_FAWACPInfo_Handle->AcpCodeFault.ACPCODEFAULTDCDCConverterFaultState;
	}
	p_FAWACPInfo_Handle->AcpCodeFault.ACPCODEFAULTVehControllerFailState = pData[195];	 //整车控制器故障
	if(p_FAWACPInfo_Handle->AcpCodeFault.ACPCODEFAULTVehControllerFailState != FaultSigan.ACPCODEFAULTVehControllerFailState){
		if(FaultSigan.ACPCODEFAULTVehControllerFailState != 0)
			reportEventCmd(CFAWACP::cfawacp->timingReportingData, 18, ACPApp_VehicleCondUploadID);
		FaultSigan.ACPCODEFAULTVehControllerFailState = p_FAWACPInfo_Handle->AcpCodeFault.ACPCODEFAULTVehControllerFailState;
	}
	p_FAWACPInfo_Handle->AcpCodeFault.ACPCODEFAULTPureElecRelayCoilState.MainPositRelayCoilFault = pData[196] & 0x01;	 //主正继电器线圈状态
	if(p_FAWACPInfo_Handle->AcpCodeFault.ACPCODEFAULTPureElecRelayCoilState.MainPositRelayCoilFault != FaultSigan.MainPositRelayCoilFault){
		if(FaultSigan.MainPositRelayCoilFault != 0)
			reportEventCmd(CFAWACP::cfawacp->timingReportingData, 18, ACPApp_VehicleCondUploadID);
		FaultSigan.MainPositRelayCoilFault = p_FAWACPInfo_Handle->AcpCodeFault.ACPCODEFAULTPureElecRelayCoilState.MainPositRelayCoilFault;
	}
	p_FAWACPInfo_Handle->AcpCodeFault.ACPCODEFAULTPureElecRelayCoilState.MainNegaRelayCoilFault = (pData[196] >> 1) & 0x01;	 //主负继电器线圈状态
	if(p_FAWACPInfo_Handle->AcpCodeFault.ACPCODEFAULTPureElecRelayCoilState.MainNegaRelayCoilFault != FaultSigan.MainNegaRelayCoilFault){
		if(FaultSigan.MainNegaRelayCoilFault != 0)
			reportEventCmd(CFAWACP::cfawacp->timingReportingData, 18, ACPApp_VehicleCondUploadID);
		FaultSigan.MainNegaRelayCoilFault = p_FAWACPInfo_Handle->AcpCodeFault.ACPCODEFAULTPureElecRelayCoilState.MainNegaRelayCoilFault;
	}
	p_FAWACPInfo_Handle->AcpCodeFault.ACPCODEFAULTPureElecRelayCoilState.PrefillRelayCoilFault = (pData[196] >> 2) & 0x01;	 //预充继电器线圈状态
	if(p_FAWACPInfo_Handle->AcpCodeFault.ACPCODEFAULTPureElecRelayCoilState.PrefillRelayCoilFault != FaultSigan.PrefillRelayCoilFault){
		if(FaultSigan.PrefillRelayCoilFault != 0)
			reportEventCmd(CFAWACP::cfawacp->timingReportingData, 18, ACPApp_VehicleCondUploadID);
		FaultSigan.PrefillRelayCoilFault = p_FAWACPInfo_Handle->AcpCodeFault.ACPCODEFAULTPureElecRelayCoilState.PrefillRelayCoilFault;
	}
	p_FAWACPInfo_Handle->AcpCodeFault.ACPCODEFAULTPureElecRelayCoilState.RechargePositRelayCoilFault = (pData[196] >> 3) & 0x01;	 //充电正继电器线圈状态
	if(p_FAWACPInfo_Handle->AcpCodeFault.ACPCODEFAULTPureElecRelayCoilState.RechargePositRelayCoilFault != FaultSigan.RechargePositRelayCoilFault){
		if(FaultSigan.RechargePositRelayCoilFault != 0)
			reportEventCmd(CFAWACP::cfawacp->timingReportingData, 18, ACPApp_VehicleCondUploadID);
		FaultSigan.RechargePositRelayCoilFault = p_FAWACPInfo_Handle->AcpCodeFault.ACPCODEFAULTPureElecRelayCoilState.RechargePositRelayCoilFault;
	}
	p_FAWACPInfo_Handle->AcpCodeFault.ACPCODEFAULTPureElecRelayCoilState.RechargeNegaRelayCoilFault = (pData[196] >> 4) & 0x01;	 //充电负继电器线圈状态
	if(p_FAWACPInfo_Handle->AcpCodeFault.ACPCODEFAULTPureElecRelayCoilState.RechargeNegaRelayCoilFault != FaultSigan.RechargeNegaRelayCoilFault){
		if(FaultSigan.RechargeNegaRelayCoilFault != 0)
			reportEventCmd(CFAWACP::cfawacp->timingReportingData, 18, ACPApp_VehicleCondUploadID);
		FaultSigan.RechargeNegaRelayCoilFault = p_FAWACPInfo_Handle->AcpCodeFault.ACPCODEFAULTPureElecRelayCoilState.RechargeNegaRelayCoilFault;
	}
	p_FAWACPInfo_Handle->AcpCodeFault.ACPCODEFAULTPureElecRelayCoilState.MainPositiveRelayFault = (pData[196] >> 5) & 0x01;	 //主正继电器故障
	if(p_FAWACPInfo_Handle->AcpCodeFault.ACPCODEFAULTPureElecRelayCoilState.MainPositiveRelayFault != FaultSigan.MainPositiveRelayFault){
		if(FaultSigan.MainPositiveRelayFault != 0)
			reportEventCmd(CFAWACP::cfawacp->timingReportingData, 18, ACPApp_VehicleCondUploadID);
		FaultSigan.MainPositiveRelayFault = p_FAWACPInfo_Handle->AcpCodeFault.ACPCODEFAULTPureElecRelayCoilState.MainPositiveRelayFault;
	}
	p_FAWACPInfo_Handle->AcpCodeFault.ACPCODEFAULTPureElecRelayCoilState.MainNagetiveRelayFault = (pData[196] >> 6) & 0x01;	 //主负继电器故障
	if(p_FAWACPInfo_Handle->AcpCodeFault.ACPCODEFAULTPureElecRelayCoilState.MainNagetiveRelayFault != FaultSigan.MainNagetiveRelayFault){
		if(FaultSigan.MainNagetiveRelayFault != 0)
			reportEventCmd(CFAWACP::cfawacp->timingReportingData, 18, ACPApp_VehicleCondUploadID);
		FaultSigan.MainNagetiveRelayFault = p_FAWACPInfo_Handle->AcpCodeFault.ACPCODEFAULTPureElecRelayCoilState.MainNagetiveRelayFault;
	}

	//************************驾驶行为特殊事件分析***************************//
	p_FAWACPInfo_Handle->AcpCodeFault.ACPCODEFAULTTSROverSpeedAlarmState = pData[197];	  //TSR超速报警
	p_FAWACPInfo_Handle->AcpCodeFault.ACPCODEFAULTTSRLimitSpeedState = pData[198];		  //TSR限速
	p_FAWACPInfo_Handle->AcpCodeFault.ACPCODEFAULTAEBInterventionState = pData[199];	  //AEB介入
	p_FAWACPInfo_Handle->AcpCodeFault.ACPCODEFAULTABSInterventionState = pData[200];	  //ABS介入
	p_FAWACPInfo_Handle->AcpCodeFault.ACPCODEFAULTASRInterventionState = pData[201];	  //ASR介入
	p_FAWACPInfo_Handle->AcpCodeFault.ACPCODEFAULTESPInterventionState = pData[202];	  //ESP介入
	p_FAWACPInfo_Handle->AcpCodeFault.ACPCODEFAULTDSMAlarmState = pData[203];			  //DSM报警
	p_FAWACPInfo_Handle->AcpCodeFault.ACPCODEFAULTTowHandOffDiskState = pData[204];		  //双手离开方向盘提示
	p_FAWACPInfo_Handle->AcpCodeFault.ACPCODEFAULTACCState = pData[205];				  //ACC状态
	p_FAWACPInfo_Handle->AcpCodeFault.ACPCODEFAULTACCSetSpeedState = pData[206];		  //ACC速度设定
	p_FAWACPInfo_Handle->AcpCodeFault.ACPCODEFAULTFCWAlarmState = pData[207];		      //FCW报警
	p_FAWACPInfo_Handle->AcpCodeFault.ACPCODEFAULTFCWState = pData[208];				  //FCW状态
	p_FAWACPInfo_Handle->AcpCodeFault.ACPCODEFAULTFCWAlarmAccePedalFallState = pData[209];//FCW报警后加速踏板陡降时长
	p_FAWACPInfo_Handle->AcpCodeFault.ACPCODEFAULTFCWAlarmFirstBrakeState = pData[210];   //FCW报警后首次刹车时长
	p_FAWACPInfo_Handle->AcpCodeFault.ACPCODEFAULTSLDWState = pData[211];				  //LDW状态
	p_FAWACPInfo_Handle->AcpCodeFault.ACPCODEFAULTLDWAlarmState = pData[212];			  //LDW报警
	p_FAWACPInfo_Handle->AcpCodeFault.ACPCODEFAULTLDWAlarmDireDiskResponseState = pData[213];//LDW报警后方向盘响应
	p_FAWACPInfo_Handle->AcpCodeFault.ACPCODEFAULTLKAState = pData[214];				//LKA状态
	p_FAWACPInfo_Handle->AcpCodeFault.ACPCODEFAULTLKAInterventionState = pData[215];	//LKA介入
	p_FAWACPInfo_Handle->AcpCodeFault.ACPCODEFAULTLKADriverTakeOverPromptState = pData[216];//LKA驾驶员接管提示
	p_FAWACPInfo_Handle->AcpCodeFault.ACPCODEFAULTLKADriverResponsState = pData[217];	//LKA驾驶员接管提示后方向盘响应时
	p_FAWACPInfo_Handle->AcpCodeFault.ACPCODEFAULTLBSDState = pData[218];				//BSD状态
	p_FAWACPInfo_Handle->AcpCodeFault.ACPCODEFAULTBSDLeftSideAlarmState = pData[219];	//BSD左侧报警
	p_FAWACPInfo_Handle->AcpCodeFault.ACPCODEFAULTBSDRightSideAlarmState = pData[220];	//BSD右侧报警
	p_FAWACPInfo_Handle->AcpCodeFault.ACPCODEFAULTBSDAlarmReftWheelRespState = pData[221];//BSD报警后方向盘响应时长
	p_FAWACPInfo_Handle->AcpCodeFault.ACPCODEFAULTBSDAlarmFirstBrakeState = pData[222];	//BSD报警后首次刹车时长
	p_FAWACPInfo_Handle->AcpCodeFault.ACPCODEFAULTBSDAlarmPedalAcceState = pData[223];	//BSD报警后加速踏板开始陡降时长
	p_FAWACPInfo_Handle->AcpCodeFault.ACPCODEFAULTCrossLeftReportState = pData[224];	//交叉车流预警左侧报警
	p_FAWACPInfo_Handle->AcpCodeFault.ACPCODEFAULTCrossRightReportState = pData[225];	//交叉车流预警右侧报警
	p_FAWACPInfo_Handle->AcpCodeFault.ACPCODEFAULTCrossAlarmWhellState = pData[226];	//交叉车流预警后方向盘报警时长
	p_FAWACPInfo_Handle->AcpCodeFault.ACPCODEFAULTCrossAlarmStopState = pData[227];		//交叉车流预警报警后首次刹车时长
	p_FAWACPInfo_Handle->AcpCodeFault.ACPCODEFAULTCrossAlarmAcceTreadleState = pData[228];//交叉车流预警后加速踏板开始陡降时长
	p_FAWACPInfo_Handle->AcpCodeFault.ACPCODEFAULTAlterTrackAssistLeftAlarmState = pData[229];	//变道辅助左侧报警
	p_FAWACPInfo_Handle->AcpCodeFault.ACPCODEFAULTAlterTrackAssistRightAlarmState = pData[230];//变道辅助右侧报警
	p_FAWACPInfo_Handle->AcpCodeFault.ACPCODEFAULTAlterTrackAssistDireRepsonState = pData[231];//变道辅助报警后方向盘响应
	p_FAWACPInfo_Handle->AcpCodeFault.ACPCODEFAULTAlterTrackAssistFirstStopState = pData[232];//变道辅助报警后首次刹车时长
	p_FAWACPInfo_Handle->AcpCodeFault.ACPCODEFAULTAlterTrackAssistAcceDropState = pData[233];//变道辅助报警后加速踏板开始陡降时长

	SigEvent.s_DoorIntrusAlarm = pData[234];	//车门入侵报警
	if(DoorIntrusType == 0 && CFAWACP::cfawacp->m_loginState == 2){
		if(SigEvent.s_DoorIntrusAlarm != DoorIntrusType){
			reportEventCmd(CFAWACP::cfawacp->timingReportingData, 1, ACPApp_VehicleAlarmID);
			DoorIntrusType = 1;
		}
	}
	SigEvent.s_VehOutFileAlarm = pData[235];	//紧急熄火报警
	if(OutFileTyoe == 0 && CFAWACP::cfawacp->m_loginState == 2){
		if(SigEvent.s_VehOutFileAlarm != OutFileTyoe){
			reportEventCmd(CFAWACP::cfawacp->timingReportingData, 5, ACPApp_VehicleAlarmID);
			OutFileTyoe = 1;
		}
	}

	//碰到信号
	SigEvent.s_VehCollideAlarm = pData[236];
	if(VehCollideType == 0 && CFAWACP::cfawacp->m_loginState == 2)
	{
		if(SigEvent.s_VehCollideAlarm != VehCollideType){
			reportEventCmd(CFAWACP::cfawacp->timingReportingData, 1, ACPApp_EmergencyDataID);
			VehCollideType = 1;			
		}
	}
	if(SigEvent.s_VehCollideAlarm == 1)
	{	for(uint8_t iLoop = 0; iLoop < 3; iLoop++)
		{
			if(voiceCall(p_FAWACPInfo_Handle->RemoteDeviceConfigInfo[0].EmergedCall) == 0)
			break;
		}
	}
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
	//MCULOG("Escape *pBuffData:%02x, pBuff=%02x addr:%x\n",*pBuffData,*pBuff, pBuffData);
	pBuffData++;

	escapeDataLen = len-2;
	//MCULOG("escapeDataLen:%d\n", escapeDataLen);

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

	//MCULOG("escapeTimes:%d\n", escapeTimes);
	totalEscapeDataLen = escapeDataLen+escapeTimes+2;
	//MCULOG("totalEscapeDataLen:%d\n", totalEscapeDataLen);
	
	write(fd, pBuff, totalEscapeDataLen);
	//MCULOG("write data ok!\n");

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
  /*  uint8_t *pos = pBuff;
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
    *pos++ = u8Csq;*/

uint8_t *pos = pBuff;

	//4G network register status
	*pos++ = tboxInfo.networkStatus.networkRegSts;

	//4G call status
    *pos++ = tboxInfo.operateionStatus.phoneType;

	//4G signal strength
	*pos++ = tboxInfo.networkStatus.signalStrength;

	//wifi status
	if(tboxInfo.operateionStatus.wifiStartStatus == -1)
		*pos++ = 1;
	else
		*pos++ = 0;

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
	uint8_t SubitemCode;
	uint8_t SubitemCodeParam;
	int isLoop = 0;
	static uint8_t iFlag = 0;
	static time_t NowTime = 0;
	time_t LastTime = 0;

	SubitemCode = pData[8];		//信号编码
	SubitemCodeParam = pData[9];//信号值
	printf("===============8090 Remote Ctrl Msg===============\n");
	printf("MCU SubitemCode == %d, SubitemCodeParam == %d\n",SubitemCode, SubitemCodeParam);
	for(int8_t i = 0; i < datalen; i++)
		printf("%02x ", pData[i]);
	printf("\n\n\n");

	MCULOG("MCU SubitemCode == %d\n",SubitemCode);
	switch(SubitemCode)
	{
		case MCUVehBody_Lock:
			p_FAWACPInfo_Handle->RemoteControlData.VehicleBody.VehicleBody_Lock = SubitemCodeParam;
			isLoop = 1;
			break;
		case MCUVehBody_Window:
			p_FAWACPInfo_Handle->RemoteControlData.VehicleBody.VehicleBody_Window = SubitemCodeParam;
			isLoop = 1;
			break;
		case MCUVehBody_Sunroof:
			p_FAWACPInfo_Handle->RemoteControlData.VehicleBody.VehicleBody_Sunroof = SubitemCodeParam;
			isLoop = 1;
			break;
		case MCUVehBody_TrackingCar:
			p_FAWACPInfo_Handle->RemoteControlData.VehicleBody.VehicleBody_TrackingCar = SubitemCodeParam;
			isLoop = 1;
			break;
		case MCUVehBody_Lowbeam:
			p_FAWACPInfo_Handle->RemoteControlData.VehicleBody.VehicleBody_Lowbeam = SubitemCodeParam;
			isLoop = 1;
			break;
		case MCUAir_Control:
			p_FAWACPInfo_Handle->RemoteControlData.Airconditioner.Airconditioner_Control.dataBit.dataState = SubitemCodeParam & 0x7F;
			p_FAWACPInfo_Handle->RemoteControlData.Airconditioner.Airconditioner_Control.dataBit.flag = SubitemCodeParam >> 7;
			iFlag++;
			break;
		case MCUAir_CompressorSwitch:
			p_FAWACPInfo_Handle->RemoteControlData.Airconditioner.Airconditioner_CompressorSwitch.dataBit.dataState = SubitemCodeParam & 0x7F;
			p_FAWACPInfo_Handle->RemoteControlData.Airconditioner.Airconditioner_CompressorSwitch.dataBit.flag = SubitemCodeParam >> 7;
			iFlag++;
			break;
		case MCUAir_Temperature:
			p_FAWACPInfo_Handle->RemoteControlData.Airconditioner.Airconditioner_Temperature.dataBit.dataState = SubitemCodeParam & 0x7F;
			p_FAWACPInfo_Handle->RemoteControlData.Airconditioner.Airconditioner_Temperature.dataBit.flag = SubitemCodeParam >> 7;
			iFlag++;
			break;
		case MCUAir_SetAirVolume:
			p_FAWACPInfo_Handle->RemoteControlData.Airconditioner.Airconditioner_SetAirVolume.dataBit.dataState = SubitemCodeParam & 0x7F;
			p_FAWACPInfo_Handle->RemoteControlData.Airconditioner.Airconditioner_SetAirVolume.dataBit.flag = SubitemCodeParam >> 7;
			iFlag++;
			break;
		case MCUAir_FrontDefrostSwitch:
			p_FAWACPInfo_Handle->RemoteControlData.Airconditioner.Airconditioner_FrontDefrostSwitch.dataBit.dataState = SubitemCodeParam & 0x7F;
			p_FAWACPInfo_Handle->RemoteControlData.Airconditioner.Airconditioner_FrontDefrostSwitch.dataBit.flag = SubitemCodeParam >> 7;
			iFlag++;
			break;
		case MCUAir_Heatedrear:
			p_FAWACPInfo_Handle->RemoteControlData.Airconditioner.Airconditioner_Heatedrear.dataBit.dataState = SubitemCodeParam & 0x7F;
			p_FAWACPInfo_Handle->RemoteControlData.Airconditioner.Airconditioner_Heatedrear.dataBit.flag = SubitemCodeParam >> 7;
			iFlag++;
			break;
		case MCUAir_BlowingMode:
			p_FAWACPInfo_Handle->RemoteControlData.Airconditioner.Airconditioner_BlowingMode.dataBit.dataState = SubitemCodeParam & 0x7F;
			p_FAWACPInfo_Handle->RemoteControlData.Airconditioner.Airconditioner_BlowingMode.dataBit.flag = SubitemCodeParam >> 7;
			iFlag++;
			break;
		case MCUAir_InOutCirculate:
			p_FAWACPInfo_Handle->RemoteControlData.Airconditioner.Airconditioner_InOutCirculate.dataBit.dataState = SubitemCodeParam & 0x7F;
			p_FAWACPInfo_Handle->RemoteControlData.Airconditioner.Airconditioner_InOutCirculate.dataBit.flag = SubitemCodeParam >> 7;
			iFlag++;
			break;
		case MCUAir_AutoSwitch:
			p_FAWACPInfo_Handle->RemoteControlData.Airconditioner.Airconditioner_AutoSwitch.dataBit.dataState = SubitemCodeParam & 0x7F;
			p_FAWACPInfo_Handle->RemoteControlData.Airconditioner.Airconditioner_AutoSwitch.dataBit.flag = SubitemCodeParam >> 7;
			iFlag++;
			break;
		case MCUEngineState_Switch:
			p_FAWACPInfo_Handle->RemoteControlData.EngineState.EngineState_Switch = SubitemCodeParam;
			isLoop = 1;
			break;
		case MCUVehSeat_DrivingSeat:
			p_FAWACPInfo_Handle->RemoteControlData.VehicleSeat.VehicleSeat_DrivingSeat = SubitemCodeParam;
			isLoop = 1;
			break;
		case MCUVehSeat_Copilotseat:
			p_FAWACPInfo_Handle->RemoteControlData.VehicleSeat.VehicleSeat_Copilotseat = SubitemCodeParam;
			isLoop = 1;
			break;
		case MCUVehChargeMode_Immediate:
			break;
		case MCUVehChargeMode_Appointment:
			break;
		case MCUVehChargeMode_EmergeCharg:
			break;
		case MCUVeh_WIFIStatus:
			break;
		case MCUVeh_AutoOUT:
			break;
		case MCUVehBody_LuggageCar:
			break;
		case MCUVehSeat_DrivingSeatMomery:
			break;
		case MCUVeh_EnterRemoteModeFail:
			isLoop = 1;
			break;
		default:
			break;
	}

	if(isLoop == 0)	//多条控制消息
	{
		if(iFlag == ctrlTotal)	//空调多条反馈全部收到
		{
			ReplayRemoteCtrl(CFAWACP::cfawacp->cb_TspRemoteCtrl);
			iFlag = 0;
		}
		else if(iFlag == 1)	//获取空调多条控制消息第一个反馈时的时间
		{
			time(&NowTime);
		}
		else	//超出等待时长
		{
			time(&LastTime);
			if(LastTime - NowTime >= 10)
			{
				ReplayRemoteCtrl(CFAWACP::cfawacp->cb_TspRemoteCtrl);
				iFlag = 0;
			}
		}
	}
	else	//单条消息
	{
		ReplayRemoteCtrl(CFAWACP::cfawacp->cb_TspRemoteCtrl);
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
	int i,headLen;
	int totalDataLen;
	unsigned char *pos = NULL;
	unsigned short int dataLen = 2;
	unsigned int checkCode;
	static unsigned short int serialNO = 0;
	printf("===========Begin Remote Ctrl seccess ===========!!\n");
	m_mcuUart->ctrlTotal = SubitemTotal;
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
	printf("headLen = %d \t subCmd:%02x\n", headLen, SubitemCode);

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
			return 0;
			break;
		case VehicleAutoOUTID:
			*pos++ = 0x15;
			return 0;			
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
	printf("===========Send Remote Ctrl seccess ===========!!\n");

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
	
	if (!access(LTE_FILE_NAME, F_OK))
	{
		MCULOG("Exisist tbox upgrade file, remove it!\n");
		system(RM_LTE_FILE);
	}
	
	if(*(pos+8) == 0x00)
	{
		MCULOG("upgrade tbox\n");
	}
	else
		MCULOG("upgrade other\n");

	MCULOG("%02x, %02x, %02x, %02x,\n", *(pos+9),*(pos+10),*(pos+11), *(pos+12));

	dataSize = (*(pos+9)<<24)+(*(pos+10)<<16)+(*(pos+11)<<8)+*(pos+12);
	MCULOG("dataSize:%d\n", dataSize);

	crc32 = (*(pos+13)<<24)+(*(pos+14)<<16)+(*(pos+15)<<8)+*(pos+16);
	MCULOG("crc16:%04x, %d\n", crc32, crc32);

	length = *(pos+17);
	MCULOG("file name len:%d\n", *(pos+17));

	memset(fileName, 0, sizeof(fileName));
	memcpy(fileName, pos+18, length);
	MCULOG("file name:%s\n", fileName);

	pos = pos+18+length;
	
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
		MCULOG("open file failed\n");
		return -1;
	}
	if (len != write(fd, pData, len))
	{
		MCULOG("write file error!\n");
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
		MCULOG("result: %d\n", result);
	}
	else
	{
		if(crc == crc32)
		{
			result = 0x00;
			MCULOG("result: %d\n", result);
		}
		else{		
			result = 0x01;
			MCULOG("result: %d\n", result);
			system(RM_LTE_FILE);
		}
	}

	MCULOG("Upgrade result: %d\n", result);

	packProtocolData(TBOX_REPLY_ID, 0, &result, 1, 0);

	if(result == 0)
	{
		wds_qmi_release();
		nas_qmi_release();
		voice_qmi_release();
		mcm_sms_release();
		mcuUart::m_mcuUart->close_uart();
		LteAtCtrl->~LTEModuleAtCtrl();
		printf("################### exit 2 ###########################\n");
		
		exit(0);
	}

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
	int isFirst = 0;

	fd = open(fileName, O_RDONLY);
	if (fd < 0)
	{
		printf("Open file:%s error.\n", fileName);
		return -1;
	}else
		printf("Open file:%s success.\n", fileName);

	memset(buff, 0, sizeof(buff));

	while(1)
	{
		if((nRead = read(fd, buff, sizeof(buff))) > 0)
		{
			if(isFirst == 0)
			{
				*crc = crc32Check(0xFFFFFFFF, buff, nRead);
				isFirst = 1;
			}
			else
			{
				*crc = crc32Check(*crc, buff, nRead);
			}
		}
		else
		{
			isFirst = 0;
			break;
		}
	}

	/*while ((nRead = read(fd, buff, sizeof(buff))) > 0)
	{
		*crc = crc32Check(buff, nRead);
	}*/

	close(fd);

	return 0;
}

static const unsigned int crc32tab[] = {
	0x00000000L, 0x77073096L, 0xee0e612cL, 0x990951baL,
	0x076dc419L, 0x706af48fL, 0xe963a535L, 0x9e6495a3L,
	0x0edb8832L, 0x79dcb8a4L, 0xe0d5e91eL, 0x97d2d988L,
	0x09b64c2bL, 0x7eb17cbdL, 0xe7b82d07L, 0x90bf1d91L,
	0x1db71064L, 0x6ab020f2L, 0xf3b97148L, 0x84be41deL,
	0x1adad47dL, 0x6ddde4ebL, 0xf4d4b551L, 0x83d385c7L,
	0x136c9856L, 0x646ba8c0L, 0xfd62f97aL, 0x8a65c9ecL,
	0x14015c4fL, 0x63066cd9L, 0xfa0f3d63L, 0x8d080df5L,
	0x3b6e20c8L, 0x4c69105eL, 0xd56041e4L, 0xa2677172L,
	0x3c03e4d1L, 0x4b04d447L, 0xd20d85fdL, 0xa50ab56bL,
	0x35b5a8faL, 0x42b2986cL, 0xdbbbc9d6L, 0xacbcf940L,
	0x32d86ce3L, 0x45df5c75L, 0xdcd60dcfL, 0xabd13d59L,
	0x26d930acL, 0x51de003aL, 0xc8d75180L, 0xbfd06116L,
	0x21b4f4b5L, 0x56b3c423L, 0xcfba9599L, 0xb8bda50fL,
	0x2802b89eL, 0x5f058808L, 0xc60cd9b2L, 0xb10be924L,
	0x2f6f7c87L, 0x58684c11L, 0xc1611dabL, 0xb6662d3dL,
	0x76dc4190L, 0x01db7106L, 0x98d220bcL, 0xefd5102aL,
	0x71b18589L, 0x06b6b51fL, 0x9fbfe4a5L, 0xe8b8d433L,
	0x7807c9a2L, 0x0f00f934L, 0x9609a88eL, 0xe10e9818L,
	0x7f6a0dbbL, 0x086d3d2dL, 0x91646c97L, 0xe6635c01L,
	0x6b6b51f4L, 0x1c6c6162L, 0x856530d8L, 0xf262004eL,
	0x6c0695edL, 0x1b01a57bL, 0x8208f4c1L, 0xf50fc457L,
	0x65b0d9c6L, 0x12b7e950L, 0x8bbeb8eaL, 0xfcb9887cL,
	0x62dd1ddfL, 0x15da2d49L, 0x8cd37cf3L, 0xfbd44c65L,
	0x4db26158L, 0x3ab551ceL, 0xa3bc0074L, 0xd4bb30e2L,
	0x4adfa541L, 0x3dd895d7L, 0xa4d1c46dL, 0xd3d6f4fbL,
	0x4369e96aL, 0x346ed9fcL, 0xad678846L, 0xda60b8d0L,
	0x44042d73L, 0x33031de5L, 0xaa0a4c5fL, 0xdd0d7cc9L,
	0x5005713cL, 0x270241aaL, 0xbe0b1010L, 0xc90c2086L,
	0x5768b525L, 0x206f85b3L, 0xb966d409L, 0xce61e49fL,
	0x5edef90eL, 0x29d9c998L, 0xb0d09822L, 0xc7d7a8b4L,
	0x59b33d17L, 0x2eb40d81L, 0xb7bd5c3bL, 0xc0ba6cadL,
	0xedb88320L, 0x9abfb3b6L, 0x03b6e20cL, 0x74b1d29aL,
	0xead54739L, 0x9dd277afL, 0x04db2615L, 0x73dc1683L,
	0xe3630b12L, 0x94643b84L, 0x0d6d6a3eL, 0x7a6a5aa8L,
	0xe40ecf0bL, 0x9309ff9dL, 0x0a00ae27L, 0x7d079eb1L,
	0xf00f9344L, 0x8708a3d2L, 0x1e01f268L, 0x6906c2feL,
	0xf762575dL, 0x806567cbL, 0x196c3671L, 0x6e6b06e7L,
	0xfed41b76L, 0x89d32be0L, 0x10da7a5aL, 0x67dd4accL,
	0xf9b9df6fL, 0x8ebeeff9L, 0x17b7be43L, 0x60b08ed5L,
	0xd6d6a3e8L, 0xa1d1937eL, 0x38d8c2c4L, 0x4fdff252L,
	0xd1bb67f1L, 0xa6bc5767L, 0x3fb506ddL, 0x48b2364bL,
	0xd80d2bdaL, 0xaf0a1b4cL, 0x36034af6L, 0x41047a60L,
	0xdf60efc3L, 0xa867df55L, 0x316e8eefL, 0x4669be79L,
	0xcb61b38cL, 0xbc66831aL, 0x256fd2a0L, 0x5268e236L,
	0xcc0c7795L, 0xbb0b4703L, 0x220216b9L, 0x5505262fL,
	0xc5ba3bbeL, 0xb2bd0b28L, 0x2bb45a92L, 0x5cb36a04L,
	0xc2d7ffa7L, 0xb5d0cf31L, 0x2cd99e8bL, 0x5bdeae1dL,
	0x9b64c2b0L, 0xec63f226L, 0x756aa39cL, 0x026d930aL,
	0x9c0906a9L, 0xeb0e363fL, 0x72076785L, 0x05005713L,
	0x95bf4a82L, 0xe2b87a14L, 0x7bb12baeL, 0x0cb61b38L,
	0x92d28e9bL, 0xe5d5be0dL, 0x7cdcefb7L, 0x0bdbdf21L,
	0x86d3d2d4L, 0xf1d4e242L, 0x68ddb3f8L, 0x1fda836eL,
	0x81be16cdL, 0xf6b9265bL, 0x6fb077e1L, 0x18b74777L,
	0x88085ae6L, 0xff0f6a70L, 0x66063bcaL, 0x11010b5cL,
	0x8f659effL, 0xf862ae69L, 0x616bffd3L, 0x166ccf45L,
	0xa00ae278L, 0xd70dd2eeL, 0x4e048354L, 0x3903b3c2L,
	0xa7672661L, 0xd06016f7L, 0x4969474dL, 0x3e6e77dbL,
	0xaed16a4aL, 0xd9d65adcL, 0x40df0b66L, 0x37d83bf0L,
	0xa9bcae53L, 0xdebb9ec5L, 0x47b2cf7fL, 0x30b5ffe9L,
	0xbdbdf21cL, 0xcabac28aL, 0x53b39330L, 0x24b4a3a6L,
	0xbad03605L, 0xcdd70693L, 0x54de5729L, 0x23d967bfL,
	0xb3667a2eL, 0xc4614ab8L, 0x5d681b02L, 0x2a6f2b94L,
	0xb40bbe37L, 0xc30c8ea1L, 0x5a05df1bL, 0x2d02ef8dL
};

#if 0
unsigned int mcuUart::crc32Check(unsigned char *buff, unsigned int size)
{
    unsigned int i, crc;
    crc = 0xFFFFFFFF;

    for (i = 0; i < size; i++)
        crc = crc32tab[(crc ^ buff[i]) &0xff] ^ (crc >> 8);

    return crc ^ 0xFFFFFFFF;
}
#endif

unsigned int mcuUart::crc32Check(unsigned int crc, unsigned char *buff, unsigned int size)
{
    unsigned int i;

    for (i = 0; i < size; i++)
        crc = crc32tab[(crc ^ buff[i]) &0xff] ^ (crc >> 8);

    return crc;
}

#endif

