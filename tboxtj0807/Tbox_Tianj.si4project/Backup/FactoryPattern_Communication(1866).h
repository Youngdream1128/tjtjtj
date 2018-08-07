#ifndef _FACTORYPATTERN_COMMUNICATION_H_
#define _FACTORYPATTERN_COMMUNICATION_H_
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdint.h>
#include <sys/time.h>
#include <sys/select.h>
#include <netinet/in.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <unistd.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <pthread.h>
#include <arpa/inet.h>
#include <errno.h>
#include <netdb.h>
#include <sys/epoll.h>
#include <time.h>

#include "TBoxDataPool.h"
#include "VoiceCall.h"
#include "WiFiControl.h"
#include "dsi_netctrl.h"
#include "LTEModuleAtCtrl.h"
#include <sys/syslog.h>
#include <sys/prctl.h>



#define FACTORYPATTERN_DEBUG_EN  1

#if FACTORYPATTERN_DEBUG_EN
	#define FACTORYPATTERN_LOG(format,...) printf("### IVI ### %s, %s[%d] "format"\n",__FILE__,__FUNCTION__,__LINE__,##__VA_ARGS__)
	#define FACTORYPATTERN_ERROR(format,...) fprintf(stderr, "### IVI ### %s, %s[%d] "format"\n",__FILE__,__FUNCTION__,__LINE__,##__VA_ARGS__)
	#define FACTORYPATTERN(format,...) printf(format,##__VA_ARGS__)
#else
	#define FACTORYPATTERN_LOG(format,...)
	#define FACTORYPATTERN_ERROR(format,...)
	#define FACTORYPATTERN(format,...)
#endif




#define FACTORYPATTERN_SERVER            "192.168.100.1"
#define FACTORYPATTERN_PORT              20000
#define LISTEN_BACKLOG        1
#define MAX_EVENT_NUMBER      10
#define BUFFER_SIZE           100

/* Protocol Related */
#define MSG_HEADER_A_ID     0x55AA	//平台发给终端
#define MSG_HEADER_T_ID     0xAA55	//终端发给平台

#define	MSG_COMMAND_ModeID		0x01	//模式选择
#define MSG_COMMAND_Config		0x02	//生产配置模式
#define MSG_COMMAND_Detecting	0x03	//生产测试模式

#define Pattern_FILE            "/data/PatternCfg"


#define	TAGID_CANSTATUS			0x02
#define	TAGID_CANSTATUS_Rep		0x62
#define	TAGID_CANDATA			0x03
#define	TAGID_CANDATA_Rep		0x63
#define	TAGID_SIXSENSOR			0x04
#define	TAGID_SIXSENSOR_Rep		0x64
#define	TAGID_Emmc				0x05
#define	TAGID_Emmc_Rep			0x65
#define	TAGID_WIFI				0x06
#define	TAGID_WIFI_Rep			0x66
#define	TAGID_BT				0x07
#define	TAGID_BT_Rep			0x67
#define	TAGID_IVISTATUS			0x08
#define	TAGID_IVISTATUS_Rep		0x68
#define	TAGID_APN2				0x09
#define	TAGID_APN2_Rep			0x69
#define	TAGID_ECall				0x0A
#define	TAGID_ECall_Rep			0x6A
#define	TAGID_GPSOpen			0x0B
#define	TAGID_GPSOpen_Rep		0x6B
#define	TAGID_GPSSHORT			0x0C
#define	TAGID_GPSSHORT_Rep		0x6C
#define	TAGID_GPSSIGN			0x0D
#define	TAGID_GPSSIN_Rep		0x6D



//生产配置
typedef struct
{
	uint8_t PowerDomain_Len;//电源域长度
	uint8_t PowerDomain;	//电源域
	uint8_t TBoxVIN_Len;	//VIN长度
	uint8_t Tbox_VIN[17];	//VIN
	uint8_t SK_Len;			//SK长度
	uint8_t SK[6];			//SK
	uint8_t SupplierSN_Len;	//SN长度
	uint8_t SupplierSN[11];	//供应商序列号SN
}TBox_Config_ST;
//生产测试指标检测
typedef struct
{
	uint8_t 	CanCommunication_Status;//测试CAN通讯连通状态
	uint8_t 	CanCommunication_Data;	//发送CAN通讯测试数据
	uint8_t 	SixAxesSensor;	//测试六轴传感器
	uint8_t 	EmmcTrigger;	//测试EMMC
	uint8_t 	WifiTrigger;	//测试wifi
	uint8_t 	BTTrigger;		//测试蓝牙
	uint8_t		IVICommunication_Status;//测试IVI联网
	uint8_t		APN2Trigger;	//测试APN2
	uint8_t		ECallTrigger;	//测试安全气囊
	uint8_t		GPSOpenTrigger;	//测试GPS开路
	uint8_t		GPSShortTrigger;//测试GPS短路
	uint8_t		GPSSignTrigger;	//测试GPS信号
	uint8_t		MainPowerTrigger;//测试主电电源
	uint8_t		ReservePowerTrigger;//测试备用电源
	uint8_t		AccOffTrigger;	//测试ACC OFF
	uint8_t		TBoxInfoTrigger;//获取TBOX信息
}TBox_Detecting_ST;

//TLV数据格式
typedef struct
{
	uint8_t		TagID;
	uint8_t		TLVLen;
	uint8_t		TLVData[128];
}TBox_TLV_ST;




class FactoryPattern_Communication
{
public:
    FactoryPattern_Communication();
    ~FactoryPattern_Communication();
    int FactoryPattern_Communication_Init();
    int socketConnect();
	void et_process(struct epoll_event *events, int number, int epoll_fd, int socketfd);
    int set_non_block(int fd);
    void add_socketFd(int epoll_fd, int fd);



	uint8_t checkSum(uint8_t *pData, int len);
    uint8_t checkSum_BCC(uint8_t *pData, uint16_t len);
    uint8_t unpack_Protocol_Analysis(uint8_t *pData, int len);	
	//切换生产模式
	uint8_t unpack_ChangeTboxMode(uint8_t *pData, int len);
	//生产配置
	uint8_t unpack_Analysis_A5ToConfig(uint8_t *pData, int len);
	//生产指标项测试
	uint8_t unpack_Analysis_A5ToDetecting(uint8_t *pData, int len);
	
	uint8_t pack_hande_data_and_send(uint16_t MsgID, uint8_t state);
	
	uint16_t pack_Protocol_Data(uint8_t *pData, int len, uint16_t MsgID, uint8_t state);
	uint16_t pack_mode_data(uint8_t *pData, int len);
	uint16_t pack_Config_data(uint8_t *pData, int len);
	uint16_t pack_Detect_data(uint8_t *pData, int len, uint8_t state);

	uint16_t pack_CANSTATUS_data(uint8_t *pData, int len);
	uint16_t pack_CANDATA_data(uint8_t *pData, int len);
	uint16_t pack_SIXSENSOR_data(uint8_t *pData, int len);
	uint16_t pack_Emmc_data(uint8_t *pData, int len);
	uint16_t pack_WIFI_data(uint8_t *pData, int len);
	uint16_t pack_BT_data(uint8_t *pData, int len);
	uint16_t pack_IVISTATUS_data(uint8_t *pData, int len);
	uint16_t pack_APN2_data(uint8_t *pData, int len);
	uint16_t pack_ECall_data(uint8_t *pData, int len);
	uint16_t pack_GPSOpen_data(uint8_t *pData, int len);
	uint16_t pack_GPSSHORT_data(uint8_t *pData, int len);
	uint16_t pack_GPSSIGN_data(uint8_t *pData, int len);



private:
    int sockfd;
    int accept_fd;
    struct sockaddr_in serverAddr;

	
	uint8_t MsgSecurityVerion;

};

extern TBoxDataPool *dataPool;
extern LTEModuleAtCtrl *LteAtCtrl;


#endif

