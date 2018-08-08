#include "common.h"


const unsigned char sysVerNumber[3] = {1, 0, 0};
unsigned char mcuVerNumber[3] = {1, 0, 0};


/*****************************************************************************
* Function Name : getDate
* Description   : 获取当前日期
* Input			: char *pDest
*                 int size
* Output        : None
* Return        : None
* Auther        : ygg
* Date          : 2018.01.18
*****************************************************************************/
void getDate(char *pDest, int size)
{
	unsigned char i;
	unsigned char sysMonth = 0;
	char *pos = pDest;
	char year[5] = {0};
	char month[5] = {0};
	char day[5] = {0};
	const char *months[] = {"Jan", "Feb", "Mar", "Apr", "May", "Jun", "Jul", "Aug","Sep", "Oct", "Nov", "Dec"};
	char date[] = __DATE__;
	char time[] = __TIME__;

	//printf("Date:%s,Time:%s\n", date, time);

	sscanf(date, "%s %s %s", month, day, year);
	//printf("month:%s,day:%s,year:%s\n", month, day, year);

	for(i = 0; i<12; i++)
	{
		if(strncmp(month, months[i], 3) == 0)
		{
			sysMonth = i+1;
			break;
		}
	}
	//printf("%02x\n", sysMonth);

	memset(pDest, 0, size);
	memcpy(pos, year, 4);
	pos = pos + 4;

	*pos++ = (char)((sysMonth%100)/10+0x30);
	*pos++ = (char)(sysMonth%10+0x30);

	//printf("day len %d\n",(int)strlen(day));
	if(strlen(day) < 2)
	{
		*pos++ = 0x30; // 0x30-> ASCII: 0
		*pos++ = day[0];
	}
	else
	{
		memcpy(pos, day, 2);
		pos += 2;
	}
	
	//*pos++ = 0x5F;    // 0x5F-> ASCII: _
	//memcpy(pos, time, 8);
	
	//printf("pDest:%s\n", pDest);
}

#if 0
/*****************************************************************************
* Function Name : getSoftwareVerion
* Description   : 获取当前日期
* Input			: char *pBuff
*                 unsigned int size
* Output        : None
* Return        : 0:success
* Auther        : ygg
* Date          : 2018.01.18
*****************************************************************************/
int getSoftwareVerion(char *pBuff, unsigned int size)
{
	char sysVerion[12] = {0};
	char sysDateTime[32] = {0};

	memset(pBuff, 0, size);
	
	strcat(pBuff, RELEASE_NOTE);
	sprintf(sysVerion, "V%d.%d.%d", sysVerNumber[0], sysVerNumber[1],sysVerNumber[2]);
	strcat(pBuff, sysVerion);
	strcat(pBuff, BUILD_NOTE);
	getDate(sysDateTime, sizeof(sysDateTime));
	strcat(pBuff, sysDateTime);

	return 0;
}

#endif


/*****************************************************************************
* Function Name : getSoftwareVerion
* Description   : 获取当前日期
* Input			: char *pBuff
*                 unsigned int size
* Output        : None
* Return        : 0:success, -1:failed
* Auther        : ygg
* Date          : 2018.04.09
*****************************************************************************/
int getSoftwareVerion(char *pBuff, unsigned int size)
{
	char sysDateTime[32] = {0};
	if(pBuff == NULL)
		return -1;
	
	memset(pBuff, 0, size);
	strcat(pBuff, PROJECT_NAME);
	strcat(pBuff, ".");
//	strcat(pBuff, PLATFORM_CODE);
//	strcat(pBuff, ".");
	strcat(pBuff, SUPPORT_GBT32960_VER);
	strcat(pBuff, ".");
	strcat(pBuff, SUPPORT_ACP_VER);
	strcat(pBuff, ".");
//	strcat(pBuff, FILE_VER_PREFIX);
	strcat(pBuff, FILE_VERSION);
	strcat(pBuff, ".");
	getDate(sysDateTime, sizeof(sysDateTime));
	strcat(pBuff, sysDateTime);
	
	return 0;
}
