#include "FactoryPattern_Communication.h"

#include <fstream>
#include <iostream>
using namespace std;


uint8_t	mCurrentTboxPattern;	//TBox当前运行模式


FactoryPattern_Communication::FactoryPattern_Communication()
{
    sockfd = -1;
    accept_fd = -1;
}
FactoryPattern_Communication::~FactoryPattern_Communication()
{
	close(sockfd);
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
        FACTORYPATTERN_ERROR("bind failed.\n");
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
			
            dataLen = recv(events[i].data.fd, buff, BUFFER_SIZE, 0);
            accept_fd = events[i].data.fd;
			FACTORYPATTERN_LOG("FACTORYPATTERN_LOG recv : dataLen = %d\n", dataLen);
            if(dataLen <= 0)
            {	

				#if 1
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
				*pos++=0x01;
				*pos++=0x01;

				//
				*pos++=0x01;

				//crc
				*pos++=0x01;
				*pos++=0x01;

				//end
				*pos++=0x0A;
				
	
				int length;
				if((length = send(accept_fd, dataBuff, 8, 0)) < 0)
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
				#endif



				epoll_ctl(epoll_fd, EPOLL_CTL_DEL, events[i].data.fd, NULL); 
                close(events[i].data.fd);
            }
            else
            {
                FACTORYPATTERN_LOG("FactoryPattern Recevied data len %d:\n", dataLen);
				//for(i = 0; i < dataLen; ++i)
				//	FACTORYPATTERN_LOG("%02x ", *(buff + i));
				//FACTORYPATTERN_LOG("\n\n");	

				#if 1

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
				*pos++=0x00;
				*pos++=0x00;

				//
				*pos++=0x01;

				//crc
				*pos++=0x00;
				*pos++=0x00;

				//end
				*pos++=0x0A;
				
	
				int length;
				if((length = send(accept_fd, dataBuff, 8, 0)) < 0)
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
				#endif


	            //if(checkSum(buff, dataLen))
	            {
	                unpack_Protocol_Analysis(buff, dataLen);
	            }
            }
           
        }
        else
        {
            printf("something unexpected happened!\n");
        }
    }
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
//切换生产配置测试模式
uint8_t FactoryPattern_Communication::unpack_ChangeTboxMode(uint8_t *pData, int len)
{
	uint8_t *pos = pData;
	uint8_t nCurMode = pos[0];


	mCurrentTboxPattern = nCurMode;
	//kill other thread
	pack_hande_data_and_send( MSG_COMMAND_ModeID, 0);

	return 0;
}


//生产配置
uint8_t FactoryPattern_Communication::unpack_Analysis_Config(uint8_t *pData, int len)
{
	uint8_t *pos = pData;
	uint8_t TAG_ID = pos[0];	
	int Datalen = (*(pos+1)<<8) + *(pos+2);
	pos += 3;
	TBox_Config_ST tboxConfigST;
	tboxConfigST.PowerDomain_Len = *pos++;
	tboxConfigST.PowerDomain 	 = *pos++;
	tboxConfigST.TBoxVIN_Len 	 = *pos++;
	memcpy(tboxConfigST.Tbox_VIN, pos, 17);	
	pos += 17;
	tboxConfigST.SK_Len = *pos++;
	memcpy(tboxConfigST.SK, pos, 6);	
	pos += 6;
	tboxConfigST.SupplierSN_Len= *pos++;	//SN长度
	memcpy(tboxConfigST.SupplierSN, pos, 11);//供应商序列号SN
	
	//写入文件
	int fd = open(Pattern_FILE, O_RDWR|O_CREAT);
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
	return 0;
}
//生产指标项测试
uint8_t FactoryPattern_Communication::unpack_Analysis_Detecting(uint8_t *pData, int len)
{
	uint8_t *pos = pData;
	uint8_t TAG_ID = pos[0];
	switch(TAG_ID)
	{
	case TAGID_CONFIG:		//生产配置
		unpack_Analysis_Config(pData, len);
		break;
	case TAGID_CANSTATUS:	//Can连通状态
		//状态直接读取
		//需要测试逻辑
		break;
	case TAGID_CANDATA:		//Can数据
		break;
	case TAGID_SIXSENSOR:	//六轴传感器
		break;
	case TAGID_Emmc:		//EMMC
		break;
	case TAGID_WIFI:		//WIFI
		break;
	case TAGID_BT:			//BT
		break;
	case TAGID_IVISTATUS:	//IVI
		break;
	case TAGID_APN2:		//APN2
		break;
	case TAGID_ECall:		//Ecall
		break;
	case TAGID_GPSOpen:		//GPS开路
		break;
	case TAGID_GPSInterrupt://GPS短路
		break;
	case TAGID_GPSSIGN:		//GPS信号
		break;
	case TAGID_MAINPOWER:   //主电电源
		break;
	case TAGID_RESERVEPOWER://备用电源
		break;
	case TAGID_ACCOFF:		//Acc off
		break;
	case TAGID_TBOXINFO:	//Tbox信息
		break;
	default:
		break;	
		
	}
	pack_hande_data_and_send( MSG_COMMAND_Detecting, TAG_ID);

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
		return -1;
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
		case MSG_COMMAND_Detecting:	//生产配置测试模式
			unpack_Analysis_Detecting(pos+5, MsgBodyLen);
			break;
		default:
			FACTORYPATTERN_LOG("FACTORYPATTERN： ApplicationID error!\n");
			break;		
	}

	return 0;
}


//打包
uint8_t FactoryPattern_Communication::pack_hande_data_and_send(uint16_t MsgID, uint8_t TestTag)
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
	*pos++ = MsgID;
	
	dataLen =  pack_Protocol_Data(pos, BUFFER_SIZE, MsgID, TestTag);	
	//Fill in total data length
	dataBuff[2] = (dataLen>>8) & 0xFF;
	dataBuff[3] = (dataLen>>0) & 0xFF;	

	pos+=dataLen;

	//crc
	//uint16_t crc16 = checkSum_BCC(dataBuff+2, 4);
	//*pos++ = (crc16>>8) & 0xFF;
	//*pos++ = (crc16>>0) & 0xFF;
	*pos++=0;
	*pos++=0;

	//end
	*pos++=0x0A;
	
	dataLen	= pos-dataBuff;
	
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

uint16_t FactoryPattern_Communication::pack_Protocol_Data(uint8_t *pData, int len, uint16_t MsgID, uint8_t TestTag)
{
	uint16_t dataLen = 0;
	switch (MsgID)
	{
		case MSG_COMMAND_ModeID:
			dataLen=pack_mode_data(pData,len);
			break;
		case MSG_COMMAND_Detecting:
			dataLen = pack_Detect_data(pData,len,TestTag);
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
	//模式
	*pos++=mCurrentTboxPattern;
	//判断权限
	*pos++=0;
	return (uint16_t)(pos-pData); 
}

uint16_t FactoryPattern_Communication::pack_Detect_data(uint8_t *pData, int len, uint8_t TestTag)
{
	uint16_t dataLen = 0;
	switch (TestTag)
	{
		case TAGID_CONFIG:
			dataLen=pack_Config_data(pData,len);
			break;
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
		case TAGID_GPSInterrupt:
			dataLen = pack_GPSSHORT_data(pData,len);
			break;
		case TAGID_GPSSIGN:
			dataLen = pack_GPSSIGN_data(pData,len);
			break;
		case TAGID_MAINPOWER:   //主电电源
			dataLen = pack_MAINPOWER_data(pData,len);
			break;
		case TAGID_RESERVEPOWER://备用电源
			dataLen = pack_RESERVEPOWER_data(pData,len);
			break;
		case TAGID_ACCOFF:		//Acc off
			dataLen = pack_ACCOFF_data(pData,len);
			break;
		case TAGID_TBOXINFO:	//Tbox信息
			dataLen = pack_TBOXINFO_data(pData,len);
			break;
		default:
			FACTORYPATTERN_LOG("tag error");
			break;
	}
	return dataLen;
}

uint16_t FactoryPattern_Communication::pack_Config_data(uint8_t *pData, int len)
{
	uint8_t *pos = pData;
	*pos++=TAGID_CONFIG_Rep;
	*pos++=0;
	*pos++=1;
	//判断是否成功
	*pos++=1;
	return (uint16_t)(pos-pData); 
}

uint16_t FactoryPattern_Communication::pack_CANSTATUS_data(uint8_t *pData, int len)
{
	uint8_t *pos = pData;
	*pos++=TAGID_CANSTATUS_Rep;
	*pos++=0;
	*pos++=1;
	//判断是否成功
	*pos++=1;
	return (uint16_t)(pos-pData); 
}
uint16_t FactoryPattern_Communication::pack_CANDATA_data(uint8_t *pData, int len)
{
	uint8_t *pos = pData;
	*pos++=TAGID_CANDATA_Rep;
	*pos++=0;
	*pos++=1;
	//判断是否成功
	*pos++=1;
	return (uint16_t)(pos-pData); 
}
uint16_t FactoryPattern_Communication::pack_SIXSENSOR_data(uint8_t *pData, int len)
{
	uint8_t *pos = pData;
	uint8_t sixsensor[6];
	memset(sixsensor,0,6);
	//
	*pos++=TAGID_SIXSENSOR_Rep;
	*pos++=0;
	*pos++=6;
	//加速度与角速度
	memcpy(pos,sixsensor,6);
	pos+=6;
	return (uint16_t)(pos-pData); 
}
uint16_t FactoryPattern_Communication::pack_Emmc_data(uint8_t *pData, int len)
{	
	uint8_t *pos = pData;
	*pos++=TAGID_Emmc_Rep;
	*pos++=0;
	*pos++=1;
	//判断是否成功
	*pos++=1;
	return (uint16_t)(pos-pData); 
}
uint16_t FactoryPattern_Communication::pack_WIFI_data(uint8_t *pData, int len)
{
	uint8_t *pos = pData;
	*pos++=TAGID_WIFI_Rep;

	
	*pos++=0;
	*pos++=1;
	//判断是否成功
	*pos++=1;

	return (uint16_t)(pos-pData);
}
uint16_t FactoryPattern_Communication::pack_BT_data(uint8_t *pData, int len)
{
	uint8_t *pos = pData;
	*pos++=TAGID_BT_Rep;


	*pos++=0;
	*pos++=1;
	//判断是否成功
	*pos++=0;
	return (uint16_t)(pos-pData);
}
uint16_t FactoryPattern_Communication::pack_IVISTATUS_data(uint8_t *pData, int len)
{
	uint8_t *pos = pData;
	*pos++=TAGID_IVISTATUS_Rep;
	*pos++=0;
	*pos++=1;
	//判断是否成功
	*pos++=1;
	return (uint16_t)(pos-pData);
}
uint16_t FactoryPattern_Communication::pack_APN2_data(uint8_t *pData, int len)
{
	uint8_t *pos = pData;
	*pos++=TAGID_APN2_Rep;
	*pos++=0;
	*pos++=1;
	//判断是否成功
	*pos++=1;
	return (uint16_t)(pos-pData);
}
uint16_t FactoryPattern_Communication::pack_ECall_data(uint8_t *pData, int len)
{
	uint8_t *pos = pData;
	*pos++=TAGID_ECall_Rep;
	*pos++=0;
	*pos++=1;
	//判断是否成功
	*pos++=1;
	return (uint16_t)(pos-pData);
}
uint16_t FactoryPattern_Communication::pack_GPSOpen_data(uint8_t *pData, int len)
{
	uint8_t *pos = pData;
	*pos++=TAGID_GPSOpen_Rep;
	*pos++=0;
	*pos++=1;
	//判断是否成功
	*pos++=1;
	return (uint16_t)(pos-pData);
}
uint16_t FactoryPattern_Communication::pack_GPSSHORT_data(uint8_t *pData, int len)
{
	uint8_t *pos = pData;
	*pos++=TAGID_GPSInterrupt_Rep;
	*pos++=0;
	*pos++=1;
	//判断是否成功
	*pos++=1;
	return (uint16_t)(pos-pData);
}
uint16_t FactoryPattern_Communication::pack_GPSSIGN_data(uint8_t *pData, int len)
{
	uint8_t *pos = pData;
	*pos++=TAGID_GPSSIN_Rep;
	*pos++=0;
	*pos++=1;
	//判断是否成功
	*pos++=0;
	return (uint16_t)(pos-pData);
}
uint16_t FactoryPattern_Communication::pack_MAINPOWER_data(uint8_t *pData, int len)
{
	uint8_t *pos = pData;
	*pos++=TAGID_MAINPOWER_Rep;
	*pos++=0;
	*pos++=2;
	//主电源数据
	uint16_t mainpower=24;
	*pos++ = (mainpower >> 8)&0xFF;
	*pos++ = (mainpower>> 0)&0xFF;
	return (uint16_t)(pos-pData);
}
uint16_t FactoryPattern_Communication::pack_RESERVEPOWER_data(uint8_t *pData, int len)
{
	uint8_t *pos = pData;
	*pos++=TAGID_RESERVEPOWER_Rep;
	*pos++=0;
	*pos++=1;
	//备用电源电压
	*pos++=12;
	return (uint16_t)(pos-pData);
}
uint16_t FactoryPattern_Communication::pack_ACCOFF_data(uint8_t *pData, int len)
{
	uint8_t *pos = pData;
	*pos++=TAGID_ACCOFF_Rep;
	*pos++=0;
	*pos++=1;
	//判断是否成功
	*pos++=1;
	return (uint16_t)(pos-pData);
}
uint16_t FactoryPattern_Communication::pack_TBOXINFO_data(uint8_t *pData, int len)
{
	/*uint8_t *pos = pData;
	*pos++=TAGID_TBOXINFO_Rep;

	uint16_t len = 90;
	*pos++ = (len >> 8)&0xFF;
	*pos++ = (len>> 0)&0xFF;
	 
	 //tbox info

	TBox_PatterInfo m_tboxinfo = {0};

	memcpy(pos,m_tboxinfo.VIN,17);
	pos+=17;

	*pos++=m_tboxinfo.VehicleType
	memcpy(pos,m_tboxinfo.Sim_IMEI,15);
	pos+=15;
	memcpy(pos,m_tboxinfo.ICCID,20);
	pos+=20;
	memcpy(pos,m_tboxinfo.Sim_IMSI,15);
	pos+=15;
	memcpy(pos,m_tboxinfo.Sim_Num,11);
	pos+=11;
	memcpy(pos,m_tboxinfo.Supply_PN,11);
	pos+=11;

	return (uint16_t)(pos-pData);*/
}
