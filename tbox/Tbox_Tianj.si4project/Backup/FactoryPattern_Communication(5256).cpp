#include "FactoryPattern_Communication.h"


MSG_HEADER_ST msgHeader;
DISPATCHER_MSG_ST dispatcherMsg;



FactoryPattern_Communication::FactoryPattern_Communication()
{
	close(sockfd);
}
FactoryPattern_Communication::~FactoryPattern_Communication()
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

//解析配置&测试
uint8_t FactoryPattern_Communication::unpack_Analysis_CfgDetecting(uint8_t *pData, int len)
{
	uint8_t *pos = pData;
	uint8_t utagID;
	uint8_t uTLVLen;	
	while(len > 0)
	{
		utagID  = pos[0];
		uTLVLen = pos[1];
		
		unpack_CfgDetecting(pos, uTLVLen, utagID);
		len -= (uTLVLen+2);
		pos += (uTLVLen+2);
	}

	//pack_hande_data_and_send(nCurMode, 0);

	return 0;
}
//解析配置&测试
uint8_t FactoryPattern_Communication::unpack_CfgDetecting(uint8_t *pData, int len, uint8_t tagID)
{
	uint8_t *pos = pData;
	uint8_t TAG_ID = pos[0];
	switch(TAG_ID)
	{
		case TAGID_CANSTATUS: 
	}

	//pack_hande_data_and_send(nCurMode, 0);

	return 0;
}


//生产配置
uint8_t FactoryPattern_Communication::unpack_A5ToConfig(uint8_t *pData, int len)
{
	uint8_t *pos = pData;
	uint8_t TAG_ID = pos[0];
	switch(TAG_ID)
	{
		case TAGID_CANSTATUS: 
	}

	//pack_hande_data_and_send(nCurMode, 0);

	return 0;
}
//生产指标项测试
uint8_t FactoryPattern_Communication::unpack_A5ToDetecting(uint8_t *pData, int len)
{
	uint8_t *pos = pData;
	
	//pack_hande_data_and_send(nCurMode, 0);

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
	

	dispatcherMsg.MessageID = MsgID;
	
	dataLen = pack_Protocol_Data(dataBuff, BUFFER_SIZE, &dispatcherMsg, state);
	
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
		case MSG_COMMAND_CfgDetecting://配置&测试模式
			unpack_Analysis_CfgDetecting(pos+5, MsgBodyLen);
			break;		
		case MSG_COMMAND_Config://生产配置模式
			unpack_A5ToConfig(pos+5, MsgBodyLen);
			break;
		case MSG_COMMAND_Detecting:	//生产测试模式
			unpack_A5ToDetecting(pos+5, MsgBodyLen);
			break;
		default:
			FACTORYPATTERN_LOG("FACTORYPATTERN： ApplicationID error!\n");
			break;		
	}

	return 0;
}

uint8_t FactoryPattern_Communication::TBOX_Call_State_Report(uint8_t *pData, int len, uint16_t msgType)
{
	int i;
	int phoneNumLen;
	int state;
	uint8_t *pos = NULL;
	char phoneNum[20];

	pos = pData;
	memset(phoneNum, 0, sizeof(phoneNum));
	
	if(msgType == TBOX_CallCommandReq)
	{
		IVI_LOG("  222222222222222222222222222222222222222222\n");

		if(pData[1] > 0)
		{
			memcpy(phoneNum, pos+1, pData[1]);
			IVI_LOG("phoneNum: %s\n", phoneNum);
		}
		
		switch (pData[0])
		{
			case 0:
				IVI_LOG("phone len: %d\n", pData[1]);
				
				/**
				 * Modify by the customer request
				 * when call send the state as 3,
				 * show the calling connecting.
				 */
                pack_TBOX_Data_Report(TEST_FLAG, 3, 3);
				
				if(pData[1] > 0)
				{
					state = voiceCall(phoneNum);
				}
				else if(pData[1] == 0x00)  //B-Call
				{
					dataPool->getPara(B_CALL_ID, (void *)phoneNum, sizeof(phoneNum));
					IVI_LOG("B_CALL telephone number: %s\n", phoneNum);
					
					state = voiceCall(phoneNum);
				}
				
				break;
			case 1:
				IVI_LOG("answer phone callid:%d\n", callID);
				state = hangUp_answer(VOICE_CALL_ANSWER, callID);

				break;
			case 2:
				IVI_LOG("hangup phone callid:%d\n", callID);
				state = hangUp_answer(VOICE_CALL_HANDUP, callID);
				
				break;				
			default:
				IVI_LOG("Call state error!\n");
				break;		
		}

        /**
         * Modify by the customer request
         * when call state is -1, show the calling error, then
         * send hang up telephone.
         */
        
		if(state == -1)
		{
			IVI_LOG("Call state error!\n");
            syslog(LOG_DEBUG, " TBOX_Call_State_Report  voiceCallState = 2  ###############################\n");
            pack_TBOX_Data_Report(TEST_FLAG, 3, 2);
		}
		else
		{
            pack_hande_data_and_send(2, state);
		}
	}
	else if(msgType == TBOX_CallStateReport)
	{
		pack_TBOX_Data_Report(TEST_FLAG, 3, 0);
	}
	
	return 0;
}

uint8_t FactoryPattern_Communication::TBOX_Network_State_Query()
{
	pack_hande_data_and_send(2, 0);

	return 0;
}

uint8_t FactoryPattern_Communication::TBOX_Vehicle_VINCode_Query()
{
	pack_hande_data_and_send(2, 0);

	return 0;
}

uint8_t FactoryPattern_Communication::TBOX_Telephone_Num_Query()
{
	pack_hande_data_and_send(2, 0);

	return 0;
}

uint8_t FactoryPattern_Communication::TBOX_Ecall_State_Report()
{
	pack_hande_data_and_send(2, 0);

	return 0;
}

uint8_t FactoryPattern_Communication::TBOX_General_Info_Query()
{
	pack_hande_data_and_send(2, 0);

	return 0;
}

uint8_t FactoryPattern_Communication::TBOX_WIFI_Info_Query(uint8_t *pData, int len, uint16_t msgType)
{
	int i;
	int ret;
	int authType = 5;
	int encryptMode = 4;
	char buff[20];
	uint8_t *pos = pData;
	char ssid[20] = {0};
	char password[20] = {0};
	
	if(msgType == GET_TBOX_WIFI)
	{
	    if(setWifiState == false)
		pack_hande_data_and_send(msgType, 0);
	}
	else if(msgType == SET_TBOX_WIFI)
	{
		setWifiState = true;

		for(i = 0; i<len; i++)
		{
			syslog(LOG_DEBUG, "%02x ", *(pData + i));
		}
		syslog(LOG_DEBUG, "\n");
		
		syslog(LOG_DEBUG, "TBOX_WIFI_Info_Query     set wifi  11111111111\n");
		
		pos = pos+1;
		
		//set wifi ssid
		for(i = 0; i<15; i++)
		{
			if(*pos == 0x00)
				break;
			ssid[i] = *pos++;
		}
		IVI_LOG("Set wifi ssid:%s\n", ssid);
		syslog(LOG_DEBUG, "TBOX_WIFI_Info_Query  Set wifi ssid:%s\n", ssid);

		ret = strlen(ssid);

        memset(buff, 0, sizeof(buff));
        wifi_get_ap_ssid(buff, AP_INDEX_STA);

        if(strcmp(buff, ssid) == 0)
        {
		    syslog(LOG_DEBUG, "111111111  the same ssid   buff:%s, ssid:%s\n", buff, ssid);
        }else{
		    syslog(LOG_DEBUG, "wifi_set_ap_ssid ssid:%s\n", ssid);
		    wifi_set_ap_ssid(ssid, AP_INDEX_STA);
        }

		pos += (15-ret);
		//set wifi password
		for(i = 0; i<16; i++)
		{
			if(*pos == 0x00)
				break;
			password[i] = *pos++;
		}
		IVI_LOG("Set wifi password:%s\n", password);
		syslog(LOG_DEBUG, "TBOX_WIFI_Info_Query  Set wifi password:%s\n", password);
		ret = strlen(password);

        memset(buff, 0, sizeof(buff));
        wifi_get_ap_auth(&authType, &encryptMode, buff, AP_INDEX_STA);

        if(strcmp(buff, password) == 0)
        {
		    syslog(LOG_DEBUG, "111111111  the same password   buff:%s, password:%s\n", buff, password);
        }else{
		    syslog(LOG_DEBUG, "wifi_set_ap_auth password:%s\n", password);
            wifi_set_ap_auth(authType, encryptMode, password, AP_INDEX_STA);
        }

        //get wifi open or close state.
        if(!access("/sys/class/net/wlan0", F_OK))
        {
            syslog(LOG_DEBUG, "open wifi \n");
            //wifi is open
		if(pData[0] == 0)
		{
    			//IVI_LOG("close wifi  11111111111\n");
			syslog(LOG_DEBUG, "TBOX_WIFI_Info_Query close wifi  11111111111\n");
			
			if(tboxInfo.operateionStatus.wifiStartStatus == 0)
			{
    				//IVI_LOG("close wifi 2222222222\n");
				syslog(LOG_DEBUG, "TBOX_WIFI_Info_Query close wifi	2222222222\n");
				ret = wifi_OpenOrClose(0);
				if(ret == 0)
				{
					wifi_led_off(1);
					tboxInfo.operateionStatus.wifiStartStatus = -1;
				}
			}
		}
        }
		else
		{
            syslog(LOG_DEBUG, "close wifi \n");
            //wifi is close
    		if(pData[0] == 1)
            {
			IVI_LOG("open wifi  11111111111\n");
			syslog(LOG_DEBUG, "TBOX_WIFI_Info_Query open wifi  11111111111\n");
			
			if(tboxInfo.operateionStatus.wifiStartStatus != 0)
			{
				IVI_LOG("open wifi	22222222222\n");
				syslog(LOG_DEBUG, "TBOX_WIFI_Info_Query open wifi  22222222222\n");
				
				ret = wifi_OpenOrClose(1);
				if(ret == 0)
				{
					wifi_led_on();
					tboxInfo.operateionStatus.wifiStartStatus = 0;
				}
			}
		}
		}
		syslog(LOG_DEBUG, "TBOX_WIFI_Info_Query     set wifi  over  11111111111\n");

		setWifiState = false;
	}

	return 0;
}

uint8_t IVI_Communication::pack_TBOX_Data_Report(uint8_t TestFlag, uint16_t MsgID, uint8_t callState)
{
	int i;
	uint32_t seconds;
	uint16_t dataLen;
	int length;
	static uint8_t MsgCount = 0;
	static uint8_t EventID = 0;
	
	DISPATCHER_MSG_ST dispatcher_msg;
	memset(&dispatcher_msg, 0, sizeof(dispatcher_msg));

	uint8_t *dataBuff = (uint8_t *)malloc(BUFFER_SIZE);
	if(dataBuff == NULL)
	{
		IVI_LOG("malloc dataBuff error!");
		return 1;
	}
	memset(dataBuff, 0, BUFFER_SIZE);

	seconds = Get_Seconds_from_1970();
	dispatcher_msg.EventCreationTime = seconds;

	if(TestFlag == 1){
		dispatcher_msg.EventID = 0x00;
	}else{
		dispatcher_msg.EventID = EventID;
		EventID++;
		if(EventID == 64)
			EventID = 0;
	}
		
	dispatcher_msg.ApplicationID = CallStateReport;
	dispatcher_msg.MessageID = MsgID;
	dispatcher_msg.MsgCounter.uplinkCounter = 0x00;
	dispatcher_msg.MsgCounter.downlinkCounter = MsgCount;
	
	MsgCount++;
	if(MsgCount == 255)
		MsgCount = 0;
		
	dispatcher_msg.ApplicationDataLength; // = 0x00;
	dispatcher_msg.Result = 0x00;
	
	//IVI_LOG("111 dispatcher_msg.ApplicationDataLength:%d\n", dispatcher_msg.ApplicationDataLength);
	
	dataLen = pack_Protocol_Data(dataBuff, BUFFER_SIZE, &dispatcher_msg, callState);
	//IVI_LOG("After dispatcher_msg.ApplicationDataLength:%d\n", dispatcher_msg.ApplicationDataLength);


	syslog(LOG_DEBUG, "pack_TBOX_Data_Report  @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@\n");
	for(i = 0; i<dataLen; i++)
	{
		syslog(LOG_DEBUG, "%02x ", *(dataBuff + i));
	}
	syslog(LOG_DEBUG, "\n");
	
	syslog(LOG_DEBUG, "pack_TBOX_Data_Report  @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@\n");
		
	if((length = send(accept_fd, dataBuff, dataLen, 0)) < 0)
	{
		close(accept_fd);
	}
	else
	{
		IVI_LOG("Send data ok,length:%d\n", length);
	}

	if(dataBuff != NULL)
	{
		free(dataBuff);
		dataBuff = NULL;
	}
	
	return 0;
}

uint8_t IVI_Communication::pack_hande_data_and_send(uint16_t MsgID, uint8_t state)
{
	uint16_t dataLen;
	int length;

	uint8_t *dataBuff = (uint8_t *)malloc(BUFFER_SIZE);
	if(dataBuff == NULL)
	{
		IVI_LOG("malloc dataBuff error!");
		return 1;
	}
	memset(dataBuff, 0, BUFFER_SIZE);

	dispatcherMsg.MessageID = MsgID;
	
	dataLen = pack_Protocol_Data(dataBuff, BUFFER_SIZE, &dispatcherMsg, state);
	
	if((length = send(accept_fd, dataBuff, dataLen, 0)) < 0)
	{
		close(accept_fd);
	}
	else
	{
		IVI_LOG("Send data ok,length:%d\n", length);
	}

	if(dataBuff != NULL)
	{
		free(dataBuff);
		dataBuff = NULL;
	}
	
	return 0;
}

uint16_t IVI_Communication::pack_Protocol_Data(uint8_t *pData, int len, DISPATCHER_MSG_ST *dispatchMsg, uint8_t state)
{
	int i, ret;
	uint16_t dataLen;
	uint16_t totalLen;
	char buff[50];
	int get_authType, get_encryptMode;
    int BCallLen;
	char BCall[20] = {0};

	//For machine telephone
	char phoneNumber[] = "13800000000";

	uint8_t *pos = pData;
	uint8_t *ptr = NULL;

	*pos++ = (MESSAGE_HEADER_ID>>24) & 0xFF;
	*pos++ = (MESSAGE_HEADER_ID>>16) & 0xFF;
	*pos++ = (MESSAGE_HEADER_ID>>8)  & 0xFF;
	*pos++ = (MESSAGE_HEADER_ID>>0)  & 0xFF;

	if((dispatchMsg->ApplicationID == CallStateReport) && (dispatchMsg->MessageID == 0x03)){
		*pos++ = TEST_FLAG;
	}else{
		*pos++ = msgHeader.TestFlag;
	}

	//Msg size;
	*pos++ = 0x00;
	*pos++ = 0x00;

	//dispather length
	*pos++ = 0x00;
	*pos++ = 0x00;

	//security version
	if((dispatchMsg->ApplicationID == CallStateReport) &&(dispatchMsg->MessageID == 0x03)){
		*pos++ = MsgSecurityVerion;
	}else{
		*pos++ = msgHeader.MsgSecurityVer;
	}
	
	/* dispather Msg */
	//Event time
	*pos++ = (dispatchMsg->EventCreationTime>>24) & 0xFF;
	*pos++ = (dispatchMsg->EventCreationTime>>16) & 0xFF;
	*pos++ = (dispatchMsg->EventCreationTime>>8)  & 0xFF;
	*pos++ = (dispatchMsg->EventCreationTime>>0)  & 0xFF;

	//event id
	*pos++ = dispatchMsg->EventID;
	
	//application id
	*pos++ = (dispatchMsg->ApplicationID>>8) & 0xFF;
	*pos++ = (dispatchMsg->ApplicationID>>0) & 0xFF;
	
	//message id
	*pos++ = (dispatchMsg->MessageID>>8) & 0xFF;
	*pos++ = (dispatchMsg->MessageID>>0) & 0xFF;

	//message counter
	*pos++ = dispatchMsg->MsgCounter.uplinkCounter;
	*pos++ = dispatchMsg->MsgCounter.downlinkCounter;

	//application data len
	*pos++ = 0x00; //(dispatchMsg.ApplicationDataLength>>8) & 0xFF;
	*pos++ = 0x00; //(dispatchMsg.ApplicationDataLength>>0) & 0xFF;

	//Result
	*pos++ = (dispatchMsg->Result>>8) & 0xFF;
	*pos++ = (dispatchMsg->Result>>0) & 0xFF;

	//calculation dispather Msg length and fill the length
	dataLen = pos-pData-10;
	IVI_LOG("dispather Msg length: %d\n", dataLen);
	
	pData[7] = (dataLen>>8) & 0xFF;
	pData[8] = (dataLen>>0) & 0xFF;
	/* dispather Msg end*/

	//For calculation MSG size
	ptr = pos;

	//application data
	switch (dispatchMsg->ApplicationID)
	{
		case CallStateReport:
			IVI_LOG("  11111111111111111111111111111111111111111\n");

			if(dispatchMsg->MessageID == 0x02)
			{
				*pos++ = state;
			}
			else if(dispatchMsg->MessageID == 0x03)
			{
                if(state == 0)
                {
				//voicall state
				*pos++ = voiceCallState;
				    //printf("\n voiceCallState:%d\n",voiceCallState);
				ret = strlen(incomingPhoneNum);
				IVI_LOG("incomingPhoneNum length: %d\n", ret);
				
				//phone length
				*pos++ = ret;
				
				if(ret > 0)
				{
					memcpy(pos, incomingPhoneNum, ret);
					pos += ret;

					//the remaining byte fill in 0x00
					ret = 18 - ret;
					IVI_LOG("remaining length: %d\n", ret);

					for(i=0; i<ret; i++)
						*pos++ = 0x00;
				}
				else
				{
					for(i=0; i<18; i++)
					{
						*pos++ = 0x00;
					}
				}
			}
    			else if(state == 3)
    			{
                    *pos++ = state;
                    dataPool->getPara(B_CALL_ID, (void *)BCall, sizeof(BCall));
                    BCallLen = strlen(BCall);
                    *pos++ = BCallLen;

                    memcpy(pos, BCall, BCallLen);
                    pos += BCallLen;

                    //the remaining byte fill in 0x00
                    BCallLen = 18 - BCallLen;
                    IVI_LOG("remaining length: %d\n", BCallLen);

                    for(i=0; i<BCallLen; i++)
                        *pos++ = 0x00;
    			}
                else if(state == 2)
                {
                    *pos++ = state;
                    for(i=0; i<19; i++)
                        *pos++ = 0x00;
                }
			}

			ret = pos - ptr;
			dispatchMsg->ApplicationDataLength = ret;
			IVI_LOG("dispatchMsg->ApplicationDataLength:%d, ret:%d\n",dispatchMsg->ApplicationDataLength, ret);
			
			break;
		case NetworkStateQuery:
			IVI_LOG("  22222222222222222222222222222222222222222\n");
			*pos++ = tboxInfo.networkStatus.networkRegSts;
			*pos++ = tboxInfo.networkStatus.signalStrength;

			//IVI_LOG("  tboxInfo.networkStatus.signalStrength  :%d\n", tboxInfo.networkStatus.signalStrength);
		    syslog(LOG_DEBUG, "111111111  tboxInfo.networkStatus.signalStrength  :%d\n", tboxInfo.networkStatus.signalStrength);

			ret = pos - ptr;
			dispatchMsg->ApplicationDataLength = ret;
			IVI_LOG("dispatchMsg->ApplicationDataLength:%d, ret:%d\n",dispatchMsg->ApplicationDataLength, ret);
			
			break;
		case VehicleVINCodeQuery:
			IVI_LOG("  333333333333333333333333333333333333333333333\n");
			memset(buff, 0, sizeof(buff));
			ret = dataPool->getTboxConfigInfo(VinID, buff, sizeof(buff));
			IVI_LOG("vin:%s, vin len:%d, ret:%d\n", buff, strlen(buff), ret);

			ret = strlen(buff);
			if(ret > 0)
			{
			memcpy(pos, buff, ret);
			pos += ret;
			}
			else
			{
				for(i=0; i<17; i++)
				{
					*pos++ = 0xFF;
				}
			}

			ret = pos - ptr;
			dispatchMsg->ApplicationDataLength = ret;
			IVI_LOG("dispatchMsg->ApplicationDataLength:%d, ret:%d\n",dispatchMsg->ApplicationDataLength, ret);
			
			break;
		case TelephoneNumQuery:
			IVI_LOG("  44444444444444444444444444444444444444444444\n");
			dataLen = strlen(phoneNumber);
			IVI_LOG("phoneNumber length:%d\n", dataLen);
			
			memcpy(pos, phoneNumber, dataLen);
			pos += dataLen;

			ret = pos - ptr;

			dispatchMsg->ApplicationDataLength = dataLen;
			IVI_LOG("dispatchMsg->ApplicationDataLength:%d\n", dispatchMsg->ApplicationDataLength);

			dispatchMsg->ApplicationDataLength = ret;
			IVI_LOG("dispatchMsg->ApplicationDataLength:%d, ret:%d\n",dispatchMsg->ApplicationDataLength, ret);
			break;
		case EcallStateReport:
			IVI_LOG("  5555555555555555555555555555555555555555555555\n");
			if(tboxInfo.operateionStatus.phoneType == 1)
			{
				*pos++ = 0x01;
			}
			else
			{
				*pos++ = 0x00;
			}

			memset(buff, 0, sizeof(buff));
			dataPool->getPara(E_CALL_ID, (void *)buff, sizeof(buff));
			IVI_LOG("E_CALL telephone number: %s\n", buff);
			
			ret = strlen(buff);
			memcpy(pos, buff, ret);
			pos += ret;
			
			//the remaining byte fill in 0x00
			ret = 18 - ret;
			IVI_LOG("length: %d\n", ret);
			
			for(i=0; i<ret; i++)
				*pos++ = 0x00;
			
			ret = pos - ptr;
			dispatchMsg->ApplicationDataLength = ret;
			IVI_LOG("dispatchMsg->ApplicationDataLength:%d, ret:%d\n",dispatchMsg->ApplicationDataLength, ret);
			
			break;
		case TBOXInfoQuery:
			IVI_LOG("  6666666666666666666666666666666666666666666666\n");
			memset(buff, 0, sizeof(buff));
			if(!dataPool->getPara(SIM_ICCID_INFO, (void *)buff, sizeof(buff)))
			{
				IVI_LOG("ICCID:%s\n", buff);
			}

			ret = strlen(buff);
			memcpy(pos, buff, ret);
			pos += ret;
			IVI_LOG("ICCID len:%d\n", ret);

			dispatchMsg->ApplicationDataLength = ret;
			
			memset(buff, 0, sizeof(buff));
			if(!dataPool->getPara(CIMI_INFO, (void *)buff, sizeof(buff)))
			{
				IVI_LOG("IMSI:%s\n", buff);
			}

			ret = strlen(buff);
			memcpy(pos, buff, ret);
			pos += ret;
			IVI_LOG("IMSI len:%d\n", ret);
			
			dispatchMsg->ApplicationDataLength += ret;
			IVI_LOG("111111  dispatchMsg->ApplicationDataLength:%d\n", dispatchMsg->ApplicationDataLength);

			ret = pos - ptr;
			dispatchMsg->ApplicationDataLength = ret;
			IVI_LOG("dispatchMsg->ApplicationDataLength:%d, ret:%d\n",dispatchMsg->ApplicationDataLength, ret);
			
			break;
		case WIFIInfoQuery:
			IVI_LOG("  77777777777777777777777777777777777 \n");
			if(dispatchMsg->MessageID == GET_TBOX_WIFI)
			{
				IVI_LOG("  88888888888888888888  %d \n", tboxInfo.operateionStatus.wifiStartStatus);

				if(tboxInfo.operateionStatus.wifiStartStatus == -1)
					*pos++ = 0x00;
				else if(tboxInfo.operateionStatus.wifiStartStatus == 0)
					*pos++ = 0x01;
					
				memset(buff, 0, sizeof(buff));
				ret = wifi_get_ap_ssid(buff, AP_INDEX_STA);
				IVI_LOG("Get wifi ssid:%s\n", buff);

				ret = strlen(buff);
				IVI_LOG("ssid len:%d\n", ret);
				memcpy(pos, buff, ret);
				pos += ret;

				if(ret < 15)
				{
					for(i=0; i<(15-ret); i++)
						*pos++ = 0x00; 
				}
				
				memset(buff, 0, sizeof(buff));
				ret = wifi_get_ap_auth(&get_authType, &get_encryptMode, buff, AP_INDEX_STA);
				IVI_LOG("Get wifi password:%s\n", buff);

				ret = strlen(buff);
				IVI_LOG("password len:%d\n", ret);
				memcpy(pos, buff, ret);
				pos += ret;

				if(ret < 16)
				{
					for(i=0; i<(16-ret); i++)
						*pos++ = 0x00; 
				}

				ret = pos - ptr;
				dispatchMsg->ApplicationDataLength = ret;
				IVI_LOG("ApplicationDataLength :%d\n", ret);
			}
		
			break;	
	}

	pData[21] = (dispatchMsg->ApplicationDataLength>>8) & 0xFF;
	pData[22] = (dispatchMsg->ApplicationDataLength>>0) & 0xFF;
	IVI_LOG("pData[21]: %02x, pData[22]: %02x\n", pData[21], pData[22]);

	//fill in Msg size
	dataLen = pos-pData-10;
	IVI_LOG("Msg size: %d\n", dataLen);

	pData[5] = (dataLen>>8) & 0xFF;
	pData[6] = (dataLen>>0) & 0xFF;

	dataLen = pos-pData;
	IVI_LOG("data Len: %d\n", dataLen);

	//calculation checkSum
	ret = checkSum_BCC(pData, dataLen);
	*pos++ = checkSum_BCC(pData, dataLen);
	IVI_LOG("checkSum_BCC: %02x\n", ret);

	totalLen = pos-pData;
	IVI_LOG("total data Len: %d\n", totalLen);
	
	IVI("\n+++++++++++++++++++++++++++++++++++++++++\n");
	for(i = 0; i < totalLen; ++i)
		IVI("%02x ", *(pData + i));
	IVI("\n\n");
	IVI("+++++++++++++++++++++++++++++++++++++++++\n");

	return totalLen;
}

uint8_t IVI_Communication::TBOX_Voicall_State(uint8_t *pData)
{
	int i;
	call_info_type call_list[QMI_VOICE_CALL_INFO_MAX_V02];
	
	memcpy((void *)call_list, pData, sizeof(call_list));

	for(i = 0; i < QMI_VOICE_CALL_INFO_MAX_V02; i++)
	{
		if(call_list[i].call_id != 0)
		{
			IVI_LOG(">>>>>>voice callback:\n");
			IVI_LOG("index[%d], call_id=%d, state=%d, direction = %d, number=%s\n",
				  i,
				  call_list[i].call_id,
				  call_list[i].call_state,
				  call_list[i].direction,
				  strlen(call_list[i].phone_number) >= 1?call_list[i].phone_number:"unkonw");
			IVI_LOG("<<<<<<<<<<< \n");
			
			callID = call_list[i].call_id;
			memset(incomingPhoneNum, 0, sizeof(incomingPhoneNum));
			if(strlen(call_list[i].phone_number) >= 1)
			{
				memset(incomingPhoneNum, 0, sizeof(incomingPhoneNum));
				memcpy(incomingPhoneNum, call_list[i].phone_number, strlen(call_list[i].phone_number));
			}

			switch (call_list[i].call_state)
			{
				case CALL_STATE_INCOMING_V02:  //来电
					voiceCallState = 0;
                    TBOX_Call_State_Report(NULL, 0, 3);
					break;
				case CALL_STATE_CONVERSATION_V02:	//通话中
					voiceCallState = 1;
					tboxInfo.operateionStatus.phoneType = 3;
                    TBOX_Call_State_Report(NULL, 0, 3);
					break;
				case CALL_STATE_DISCONNECTING_V02:	//挂断
				case CALL_STATE_END_V02:	        //对方挂断
					voiceCallState = 2;
					tboxInfo.operateionStatus.phoneType = 0;
                    TBOX_Call_State_Report(NULL, 0, 3);
					break;
				case CALL_STATE_ALERTING_V02://alerting
					voiceCallState = 3;
                    TBOX_Call_State_Report(NULL, 0, 3);
					break;
				default:
					break;
			}
			
		}
	} 
	
	return 0;
}

uint32_t IVI_Communication::Get_Seconds_from_1970()
{
	uint32_t seconds;
    time_t tv;
   //struct tm *p;

	time(&tv); /*当前time_t类型UTC时间*/  
	//IVI_LOG("time():%d\n",tv);

	seconds = (uint32_t)tv;
	IVI_LOG("seconds:%d\n", seconds);

	//p = localtime(&tv); /*转换为本地的tm结构的时间按*/  
	//tv = mktime(p); /*重新转换为time_t类型的UTC时间，这里有一个时区的转换,将struct tm 结构的时间转换为从1970年至p的秒数 */ 
	//printf("time()->localtime()->mktime(): %d\n", tv);

    return seconds;
}


