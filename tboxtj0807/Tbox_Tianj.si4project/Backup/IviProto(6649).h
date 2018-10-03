#ifndef __IVIPROTO_H__
#define __IVIPROTO_H__
#include <list>

#include <stdio.h>

typedef unsigned int u32;
typedef unsigned char u8;
typedef unsigned short u16;
#define NOTICE 0
#define WARNING 1
#define ERROR 2
 
#if 0
#define LOG(outstream,LEVEL, fmt, ...)\
	do {\
        if (NOTICE == LEVEL)\
            fprintf(outstream, "NOTICE:[%s]:[%d]:"fmt"\n", __FILE__, __LINE__ , ##__VA_ARGS__);\
        else if (WARNING == LEVEL)\
            fprintf(outstream, "WARNING:[%s]:[%d]:"fmt"\n", __FILE__, __LINE__ , ##__VA_ARGS__);\
        else if (ERROR == LEVEL)\
            fprintf(outstream, "ERROR:[%s]:[%d]:"fmt"\n", __FILE__, __LINE__ , ##__VA_ARGS__);\
        fflush(outstream);\
         } while(0)
#else
#define LOG(outstream,LEVEL, fmt, ...) 
#endif
#if 0
#define SATL_BUF_LEN 1024;
typedef struct nts_satl_ctx_1
{
	/*
	   nts_satl_ctx_t *ctx;
	   nts_satl_t *s;
	   nts_bio_t *rbio;
	   nts_bio_t *wbio;
	   char recv_buff[SATL_BUF_LEN];
	   char send_buff[SATL_BUF_LEN];
	   nts_u32 tlen;
	   */
	int http_head_len;
	int http_body_len;
	//int rb_len = sizeof(recv_buff);
	//int wb_len = sizeof(send_buff);
	int rlen;
	int recvd ;
	int wlen;
	int data_len ;
	int total_len;
	int ret;
}NTS_SATL_CTX_1;


#endif 

typedef enum aid {
	CALLAID=0x2101,
	NETADID=0x2201,
	VINAID=0x2301,
	NUMAID=0x2401,
	ECALLAID=0x2501,
	TBOXAID=0x2601,
	WIFIAID=0x2701,
}AID;
typedef struct netstate
{
	u8 rssi;
	u8 state;
}NETSTATE;
typedef struct message_header{
		u8  header[4];
		u8  flag;
		u16 size;
		u16 dm_length;
		u8  version;
}MSGHEAD;
typedef  struct dispatcher_message{
	u32 eventTime;
	u8 eventId;
	u16 appId;
	u16 msgId;
	u8 msgCout;
	u8 msgCout1;
	u16 appdatalength;
	u16 result;
}DISMSG;
typedef struct task{
	void * data;
	u32 length;
	u8 type;
}TASK;
typedef struct respMsg
{
	message_header h;
	DISMSG  d;
	void * appdata;
}RESP;
static void * handler(void * arg);
static void  delete_list();
static void * handler1(void * arg);
static void * handler2(void * arg);
class IviProto
{
	public:
		IviProto();
		~IviProto();
	//	NTS_SATL_CTX_1 nts;
		void run();
		void process(void);
		void process1(void);
		void process2(void);
		void GET_TBOX_Wifi(void);
		void voiceCallState(u8 *cb_usr_data);
	private:
		u8 voice_flag;
		void unpack();
		void pack(void);
		void freetask(TASK *task);
		bool checksum(void);
		int mem_and_enum_f(RESP **,AID,u8**,int );
		int resq_push_back(RESP **);
		void TBOX_CallCommandReq(u8 *appdata);
		void TBOX_NetworkStateReq(void);
		void TBOX_VINCodeReq(void);
		void TBOX_PhoneNumberReq(void);
		void TBOX_ECallStatusReq(void);
		void TBOX_GeneralInfoReq();
		void GET_TBOX_Wifi(void);
		void SET_TBOX_Wifi(char * appdata);
		int msgcount;
		int callid;
};
#endif
