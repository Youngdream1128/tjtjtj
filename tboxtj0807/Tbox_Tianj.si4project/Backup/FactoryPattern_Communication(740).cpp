#include "FactoryPattern_Communication.h"


MSG_HEADER_ST msgHeader;
DISPATCHER_MSG_ST dispatcherMsg;



FactoryPattern_Communication::~FactoryPattern_Communication()
{
	close(sockfd);
}
FactoryPattern_Communication::FactoryPattern_Communication()
{
    sockfd = -1;
    accept_fd = -1;
}
int FactoryPattern_Communication::FactoryPattern_Communication_Init()
{
	int epoll_fd;
	struct epoll_event events[MAX_EVENT_NUMBER];
	
    while(1){
	    if(socketConnect() == 0)
	    {
	    	FACTORYPATTERN_LOG("create socket ok!\n");
		
		    epoll_fd = epoll_create(5);
		    if(epoll_fd == -1)
		    {
		        printf("FactoryPattern: fail to create epoll!\n");
		        close(sockfd);
		        sleep(1);
		    }else{
		    	break;
		    }
	    }
	    sleep(1);
    }

    add_socketFd(epoll_fd, sockfd);

	while(1)
	{
		int num = epoll_wait(epoll_fd, events, MAX_EVENT_NUMBER, -1);
		if(num < 0)
		{
			printf("FactoryPattern: epoll failure!\n");
			break;
		}
		et_process(events, num, epoll_fd, sockfd);
	}
	
    close(sockfd);

	return 0;
}

int FactoryPattern_Communication::set_non_block(int fd)
{
    int old_flag = fcntl(fd, F_GETFL);
    if(old_flag < 0)
    {
        perror("fcntl");
        return -1;
    }
    if(fcntl(fd, F_SETFL, old_flag | O_NONBLOCK) < 0)
    {
        perror("fcntl");
        return -1;
    }

    return 0;
}

void FactoryPattern_Communication::add_socketFd(int epoll_fd, int fd)
{
    struct epoll_event event;
    event.data.fd = fd;
	event.events = EPOLLIN | EPOLLET;
    epoll_ctl(epoll_fd, EPOLL_CTL_ADD, fd, &event);
    if(set_non_block(fd) == 0)
    	FACTORYPATTERN_LOG("set_non_block ok!\n");
}

int FactoryPattern_Communication::socketConnect()
{
    int opt = 1;
    sockfd = socket(AF_INET, SOCK_STREAM, 0);
    if(sockfd == -1)
    {
        FACTORYPATTERN_ERROR("socket error!");
        return -1;
    }
    setsockopt(sockfd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

    memset(&serverAddr, 0, sizeof(serverAddr));
    serverAddr.sin_family = AF_INET;
    serverAddr.sin_addr.s_addr = inet_addr(FACTORYPATTERN_SERVER);
    serverAddr.sin_port = htons(FACTORYPATTERN_PORT);

    if(bind(sockfd, (struct sockaddr *)&serverAddr, sizeof(serverAddr)) == -1)
    {
        FactoryPattern_ERROR("bind failed.\n");
        close(sockfd);
        return -1;
    }

    if(listen(sockfd, LISTEN_BACKLOG) == -1)
    {
        FACTORYPATTERN_ERROR("listen failed.\n");
        close(sockfd);
        return -1;
    }

    return 0;
}

void FactoryPattern_Communication::et_process(struct epoll_event *events, int number, int epoll_fd, int socketfd)
{
    int i;
    int dataLen;
    uint8_t buff[BUFFER_SIZE] = {0};

    for(i = 0; i < number; i++)
    {
        if(events[i].data.fd == socketfd)
        {
            struct sockaddr_in clientAddr;
            socklen_t clientAddrLen = sizeof(clientAddr);
            int connfd = accept(socketfd, (struct sockaddr *)&clientAddr, &clientAddrLen);
            add_socketFd(epoll_fd, connfd);
        }
        else if(events[i].events & EPOLLIN)
        {
            FACTORYPATTERN_LOG("et mode: event trigger once!\n");
            memset(buff, 0, BUFFER_SIZE);

            #if 0
            dataLen = received_and_check_data(events[i].data.fd, buff, BUFFER_SIZE);
            accept_fd = events[i].data.fd;
        	if(checkSum(buff, dataLen))
            {
                unpack_Protocol_Analysis(buff, dataLen);
            }
            #endif

            #if 1
            dataLen = recv(events[i].data.fd, buff, BUFFER_SIZE, 0);
            accept_fd = events[i].data.fd;
			FACTORYPATTERN_LOG("FACTORYPATTERN_LOG recv : dataLen = %d\n", dataLen);
            if(dataLen <= 0)
            {
				epoll_ctl(epoll_fd, EPOLL_CTL_DEL, events[i].data.fd, NULL); 
                close(events[i].data.fd);
            }
            else
            {
                FACTORYPATTERN_LOG("FactoryPattern Recevied data len %d:\n", dataLen);
				for(i = 0; i < dataLen; ++i)
					FACTORYPATTERN("%02x ", *(buff + i));
				FACTORYPATTERN("\n\n");
				
	            if(checkSum(buff, dataLen))
	            {
	                unpack_Protocol_Analysis(buff, dataLen);
	            }
            }
            #endif
        }
        else
        {
            printf("something unexpected happened!\n");
        }
    }
}

uint16_t FactoryPattern_Communication::received_and_check_data(int fd, uint8_t *buff, int size)
{
#if 0

	uint16_t len;
	uint8_t data;
	uint8_t state = 0;
	uint8_t *pos = buff;

	bool start = true;
	static int isfirstFrame = 0;

	while(start)
	{
		if(recv(fd, &data, 1, 0) > 0)
		{
			//IVI("buff:%p, pos:%p, len:%d, isfirstFrame: %d, data:%02x \n",buff, pos, pos-buff, isfirstFrame, data);
			//IVI("%02x ", data);
			if(isfirstFrame == 0)
			{
				switch (state)
				{
					case 0:
						if(data == 0x46)
						{
							pos = buff;
							*pos = data;
							state = 1;
						}
						break;
					case 1:
						if(data == 0x4c)
						{
							*++pos = data;
							state = 2;
						}
						break;
					case 2:
						if(data == 0x43)
						{
							*++pos = data;
							state = 3;
						}
						break;
					case 3:
						if(data == 0x46)
						{
							isfirstFrame = 1;
						}
						*++pos = data;
						
						break;
					default:
						state = 0;
						break;
						
				}
			}
			else if(isfirstFrame == 1) //check the data is the second frame
			{
				if(data == 0x4c)
				{
					start = false;
					isfirstFrame = 2;
					break;
				}
				else
				{
					isfirstFrame = 0;
					state = 3;
					*++pos = data;
				}
			}
			else if(isfirstFrame == 2)
			{
				pos = buff;
				*pos++ = 0x46;
				*pos = 0x4c;
				state = 2;
				isfirstFrame = 0;
			}
		}
	}

	if(start == false)
		len = pos-buff;
	
	IVI("\n@@@@@@@@@@@@@@@@@@@@@@@@@ received frame data, data len:%d\n", len);
	for(int i=0; i<len; i++)
		IVI("%02x ", *(buff+i));
	IVI("\n\n");
#endif	
	return len;
}

uint8_t FactoryPattern_Communication::checkSum(uint8_t *pData, int len)
{
	unsigned int u16InitCrc = 0xffff;
	unsigned int i;
	unsigned char j;
	unsigned char u8ShitBit;
	uint8_t *pos = pData+2;
	uint16_t crcDatalen = ((pos[0] << 8) & 0xFF00) | (pos[1] & 0x00FF);
	uint16_t crc16 = ((pos[len-3] << 8) & 0xFF00) | (pos[len-2] & 0x00FF);

	FACTORYPATTERN("FACTORYPATTERN: crcDatalen = %d, crc16 = %d,", crcDatalen, crc16);
		
	for(i = 0; i < crcDatalen; i++)
	{		
		u16InitCrc ^= pos[i];
		for(j=0; j<8; j++)
		{
			u8ShitBit = u16InitCrc&0x01;
			u16InitCrc >>= 1;
			if(u8ShitBit != 0)
			{
				u16InitCrc ^= 0xa001;
			}		
		}
	}
	if(crc16 == u16InitCrc)
		return 0;
	else
		return 1;
}


uint8_t FactoryPattern_Communication::checkSum_BCC(uint8_t *pData, uint16_t len)
{
	uint8_t *pos = pData;
	uint8_t checkSum = *pos++;
	
	for(uint16_t i = 0; i<(len-1); i++)
	{
		checkSum ^= *pos++;
	}

	return checkSum;
}
//切换生产模式
uint8_t FactoryPattern_Communication::unpack_ChangeTboxMode(uint8_t *pData, int len)
{
	uint8_t *pos = pData;
	uint8_t nCurMode = pos[0];

	mCurrentTboxPattern = nCurMode;
	//kill other thread
	pack_hande_data_and_send( MSG_COMMAND_ModeID, 0)；
/*
	uint8_t *dataBuff = (uint8_t *)malloc(BUFFER_SIZE);
	if(dataBuff == NULL)
	{
		FACTORYPATTERN_LOG("malloc dataBuff error!");
		return 1;
	}
	memset(dataBuff, 0, BUFFER_SIZE);

	uint8_t *pos = dataBuff;

	*pos++ = (MSG_HEADER_T_ID>>8)  & 0xFF;
	*pos++ = (MSG_HEADER_T_ID>>0)  & 0xFF;
	uint16_t len = 1;
	*pos++ = (len>>8)  & 0xFF;
	*pos++ = (len>>0)  & 0xFF;
	*pos++ = MSG_COMMAND_ModeID;
	*pos++ = 1;
	
	uint16_t crc16 = checkSum_BCC(dataBuff+2, 4);
	*pos++ = (crc16>>8) & 0xFF;
	*pos++ = (crc16>>0) & 0xFF;
	
	*pos++ = 0x0A;
	pack_hande_data_and_send(MSG_COMMAND_ModeID);
*/
	return 0;
}


//生产配置
uint8_t FactoryPattern_Communication::unpack_Analysis_A5ToConfig(uint8_t *pData, int len)
{
	uint8_t *pos = pData;
	
	TBox_Config_ST tboxConfigST;
	tboxConfigST.PowerDomain_Len = *pos++;
	tboxConfigST.PowerDomain = *pos++;
	tboxConfigST.TBoxVIN_Len = *pos++;
	memcpy(tboxConfigST.Tbox_VIN, pos, 17);	
	pos += 17;
	tboxConfigST.SK_Len = *pos++;
	memcpy(tboxConfigST.SK, pos, 6);	
	pos += 6;
	tboxConfigST.SupplierSN_Len= *pos++;	//SN长度
	memcpy(tboxConfigST.SupplierSN, pos, 11);	//供应商序列号SN
	
	//写入文件

	int fd = open(Pattern_FILE, O_RDWR);
    if ( -1 == fd)
    {
		printf("updatePara open file failed\n");
        return -1;
    }
    
    if ( -1 == write(fd, &tboxConfigST, sizeof(TBox_Config_ST)))
    {
		printf("updatePara write file error!\n");
		close(fd);
        return -1;
    }
    close(fd);
	system("sync");


	//返回数据
	//pack_hande_data_and_send(nCurMode, 0);


	pack_hande_data_and_send( MSG_COMMAND_Config, 0)；
	return 0;
}
//生产指标项测试
uint8_t FactoryPattern_Communication::unpack_Analysis_A5ToDetecting(uint8_t *pData, int len)
{
	uint8_t *pos = pData;
	uint8_t TAG_ID = pos[0];
	//pack_hande_data_and_send(nCurMode, 0);

	pack_hande_data_and_send( MSG_COMMAND_Detecting, TAG_ID)；

	return 0;
}

uint8_t FactoryPattern_Communication::pack_hande_data_and_send(uint16_t MsgID, uint8_t state)
{
	uint16_t dataLen;
	int length;

	uint8_t *dataBuff = (uint8_t *)malloc(BUFFER_SIZE);
	if(dataBuff == NULL)
	{
		FACTORYPATTERN_LOG("malloc dataBuff error!");
		return 1;
	}
	memset(dataBuff, 0, BUFFER_SIZE);
	uint8_t *pos = dataBuff;

	//protocol header
	*pos++ = (MSG_HEADER_T_ID >>8) & 0xFF;
	*pos++ = (MSG_HEADER_T_ID >>0) & 0xFF;
	
	//datalen
	*pos++=0;
	*pos++=0;

	dataLen = pack_Protocol_Data(pos, BUFFER_SIZE, MsgID, state);
	
	//Fill in total data length
	dataBuff[3] = (dataLen>>8) & 0xFF;
	dataBuff[4] = (dataLen>>0) & 0xFF;

	pos+=dataLen;

	//crc
	*pos++=0;
	*pos++=0;

	//end
	*pos++=0x0A;
	dataLen+=7;
	
	if((length = send(accept_fd, dataBuff, dataLen, 0)) < 0)
	{
		close(accept_fd);
	}
	else
	{
		FACTORYPATTERN_LOG("Send data ok,length:%d\n", length);
	}

	if(dataBuff != NULL)
	{
		free(dataBuff);
		dataBuff = NULL;
	}
	
	return 0;
}

uint16_t FactoryPattern_Communication::pack_Protocol_Data(uint8_t *pData, int len, uint16_t MsgID, uint8_t state)
{
	uint16_t dataLen = 0;
	switch (MsgID)
	{
		case MSG_COMMAND_ModeID:
			dataLen=pack_mode_data(pData,len);
			break;
		case MSG_COMMAND_Config:
			dataLen = pack_Config_data(pData,len);
			break;
		case MSG_COMMAND_Detecting:
			dataLen = pack_Detect_data(pData,len,state);
			break;
		default:
			FACTORYPATTERN_LOG("cmd error");
			break;
	}
	return dataLen;
}

uint16_t FactoryPattern_Communication::pack_mode_data(uint8_t *pData, int len)
{
	uint8_t *pos = pData;
	*pos++=MSG_COMMAND_ModeID;
	
	//判断权限
	*pos++=0；
	return (uint16_t)(pos-pData); 
}
uint16_t FactoryPattern_Communication::pack_Config_data(uint8_t *pData, int len)
{
	uint8_t *pos = pData;
	*pos++=MSG_COMMAND_Config;
	//判断是否成功
	*pos++=0；
	return (uint16_t)(pos-pData); 
}
uint16_t FactoryPattern_Communication::pack_Detect_data(uint8_t *pData, int len, uint8_t state)
{
	uint16_t dataLen = 0;
	switch (state)
	{
		case TAGID_CANSTATUS:
			dataLen=pack_CANSTATUS_data(pData,len);
			break;
		case TAGID_CANDATA:
			dataLen = pack_CANDATA_data(pData,len);
			break;
		case TAGID_SIXSENSOR:
			dataLen = pack_SIXSENSOR_data(pData,len);
			break;
		case TAGID_Emmc:
			dataLen = pack_Emmc_data(pData,len);
			break;
		case TAGID_WIFI:
			dataLen = pack_WIFI_data(pData,len);
			break;
		case TAGID_BT:
			dataLen = pack_BT_data(pData,len);
			break;
		case TAGID_IVISTATUS:
			dataLen = pack_IVISTATUS_data(pData,len);
			break;
		case TAGID_APN2:
			dataLen = pack_APN2_data(pData,len);
			break;
		case TAGID_ECall:
			dataLen = pack_ECall_data(pData,len);
			break;
		case TAGID_GPSOpen:
			dataLen = pack_GPSOpen_data(pData,len);
			break;
		case TAGID_GPSSHORT:
			dataLen = pack_GPSSHORT_data(pData,len);
			break;
		case TAGID_GPSSIGN:
			dataLen = pack_GPSSIGN_data(pData,len);
			break;
		default:
			FACTORYPATTERN_LOG("tag error");
			break;
	}
	return dataLen;
}

uint16_t FactoryPattern_Communication::pack_CANSTATUS_data(uint8_t *pData, int len)
{
	uint8_t *pos = pData;

	//判断是否成功
	*pos++=0；
	return (uint16_t)(pos-pData); 
}
uint16_t FactoryPattern_Communication::pack_CANDATA_data(uint8_t *pData, int len)
{
	uint8_t *pos = pData;

	//判断是否成功
	*pos++=0；
	return (uint16_t)(pos-pData); 
}
uint16_t FactoryPattern_Communication::pack_SIXSENSOR_data(uint8_t *pData, int len)
{
	uint8_t *pos = pData;

	//判断是否成功
	*pos++=0；
	return (uint16_t)(pos-pData); 
}
uint16_t FactoryPattern_Communication::pack_Emmc_data(uint8_t *pData, int len)
{	
	uint8_t *pos = pData;

	//判断是否成功
	*pos++=0；
	return (uint16_t)(pos-pData); 
}
uint16_t FactoryPattern_Communication::pack_WIFI_data(uint8_t *pData, int len)
{
	uint8_t *pos = pData;

	//判断是否成功
	*pos++=0；
	return (uint16_t)(pos-pData);
}
uint16_t FactoryPattern_Communication::pack_BT_data(uint8_t *pData, int len)
{
	uint8_t *pos = pData;

	//判断是否成功
	*pos++=0；
	return (uint16_t)(pos-pData);
}
uint16_t FactoryPattern_Communication::pack_IVISTATUS_data(uint8_t *pData, int len)
{
	uint8_t *pos = pData;

	//判断是否成功
	*pos++=0；
	return (uint16_t)(pos-pData);
}
uint16_t FactoryPattern_Communication::pack_APN2_data(uint8_t *pData, int len)
{
	uint8_t *pos = pData;

	//判断是否成功
	*pos++=0；
	return (uint16_t)(pos-pData);
}
uint16_t FactoryPattern_Communication::pack_ECall_data(uint8_t *pData, int len)
{
	uint8_t *pos = pData;

	//判断是否成功
	*pos++=0；
	return (uint16_t)(pos-pData);
}
uint16_t FactoryPattern_Communication::pack_GPSOpen_data(uint8_t *pData, int len)
{
	uint8_t *pos = pData;

	//判断是否成功
	*pos++=0；
	return (uint16_t)(pos-pData);
}
uint16_t FactoryPattern_Communication::pack_GPSSHORT_data(uint8_t *pData, int len)
{
	uint8_t *pos = pData;

	//判断是否成功
	*pos++=0；
	return (uint16_t)(pos-pData);
}
uint16_t FactoryPattern_Communication::pack_GPSSIGN_data(uint8_t *pData, int len)
{
	uint8_t *pos = pData;

	//判断是否成功
	*pos++=0；
	return (uint16_t)(pos-pData);
}



uint8_t FactoryPattern_Communication::unpack_Protocol_Analysis(uint8_t *pData, int len)
{
	int dataLen;
	uint16_t DispatcherMsgLen;
	uint8_t *pos = pData;

	if((pData[0] == 0x55) && (pData[1] == 0xAA) && (pData[len-1] == 0x0A))
	{
		FACTORYPATTERN_LOG("FACTORYPATTERN  Analysis header&nail check ok!\n");
	}else{
		FACTORYPATTERN_LOG("FACTORYPATTERN  Analysis header&nail check fail !\n");
		return 1;
	}
	//消息体长度
	uint16_t MsgBodyLen = (*(pos+2)<<8) + *(pos+3);//((pos[2] << 8) & 0xFF00) | (pos[3] & 0x00FF);	
	//消息命令字
	uint8_t MsgCommandID = *(pos+4);
	FACTORYPATTERN_LOG("FACTORYPATTERN_LOG: msg body len = %d commandID = %d\n", MsgBodyLen, MsgCommandID);
	
	switch (MsgCommandID)
	{
		case MSG_COMMAND_ModeID://模式选择
			unpack_ChangeTboxMode(pos+5, MsgBodyLen);
			break;			
		case MSG_COMMAND_Config://生产配置模式
			unpack_Analysis_A5ToConfig(pos+5, MsgBodyLen);
			break;
		case MSG_COMMAND_Detecting:	//生产测试模式
			unpack_Analysis_A5ToDetecting(pos+5, MsgBodyLen);
			break;
		default:
			FACTORYPATTERN_LOG("FACTORYPATTERN： ApplicationID error!\n");
			break;		
	}

	return 0;
}

