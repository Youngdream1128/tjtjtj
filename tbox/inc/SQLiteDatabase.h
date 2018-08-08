#ifndef SQLITEDATABASE_H_
#define SQLITEDATABASE_H_
#include <stdio.h>
#include <stdlib.h>
#include <sqlite3.h>
#include <string.h>
#include "common.h"



#define SQLITE_DATABASE     "/media/card/database.db"    //"./database.db"
#define TABLE_NAME          "ReportData"



class SqliteDatabase
{
public:
	SqliteDatabase();
	~SqliteDatabase();
	int initDatabase();
	int insertData(int id, unsigned char *pData, int len, int dev, int isSend);
	/****************************************************************
	* Function Name: queryCount
	* Auther	   : yingaoguo
	* Date		   : 2017-08-4
	* Description  : query total num, flag = 0: query all data.
	*                flag = 1: query less than id total number.
	*                flag = 2: query greater than id total number.
	* Return	   : total number
	*****************************************************************/
	unsigned int queryCount(int flag, unsigned int id);
	int queryID(int times);
	unsigned int queryMaxID();
	int queryMaxTimes();
	unsigned int queryMinisSendID();
	int readData(unsigned char *pBuff, int *len, int id);
	/****************************************************************
	* Function Name: deleteData
	* Auther	   : yingaoguo
	* Date		   : 2017-08-4
	* Description  : delete data, if id = 0 then delete all data.
	* Return	   : 0
	*****************************************************************/
	int deleteData(int id);
	int queryMaxDevandID(int &id, unsigned int &dev);
	int updataisSend(int isSend, int id);
	unsigned int queryMaxdevID();
	int test();

private:
	sqlite3 *db;

};

extern SqliteDatabase *pSqliteDatabase;


#endif

