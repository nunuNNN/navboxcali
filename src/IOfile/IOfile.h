#ifndef _IOFILE_H
#define _IOFILE_H

#include <cstdio>
#include <cstring>
#include <ctime>
#include "unistd.h"

#include "typeReDefine.h"
#include "CMath.hpp"
#include "SINS.h"
#include "Kalman.h"

#pragma pack(4)


class CFileRdWt
{
	static char dirIn[256], dirOut[256];
public:
	FILE *rwf;
	char fname[256], line[512], sstr[64*4];
	double buff[64];
	float buff32[64];
	int columns, linek;
	long totsize, remsize;

	static void DirI(const char *dirI);
	static void DirO(const char *dirO);
	static void Dir(const char *dirI, const char *dirO=(const char*)NULL);
	CFileRdWt(void);
	CFileRdWt(const char *fname0, int columns0=0);
	~CFileRdWt();
	void Init(const char *fname0, int columns0=0);
	int load(int lines=1, BOOL txtDelComma=1);
	int loadf32(int lines=1);
	long load(BYTE *buf, long bufsize);
	void bwseek(int lines, int mod=SEEK_CUR);
	long filesize(int opt=1);
	int getl(void);  // get a line
	BOOL waitfor(int columnk, double val=0.0, double eps=EPS);

#ifdef PSINS_IO_FILE
	CFileRdWt& operator<<(double d);
	CFileRdWt& operator<<(const Vect3 &v);
	CFileRdWt& operator<<(const Quat &q);
	CFileRdWt& operator<<(const Mat3 &m);
	CFileRdWt& operator<<(const Vect &v);
	CFileRdWt& operator<<(const Mat &m);

	CFileRdWt& operator<<(const SINS &sins);
	CFileRdWt& operator<<(Kalman &kf);

	CFileRdWt& operator>>(double &d);
	CFileRdWt& operator>>(Vect3 &v);
	CFileRdWt& operator>>(Quat &q);
	CFileRdWt& operator>>(Mat3 &m);
	CFileRdWt& operator>>(Vect &v);
	CFileRdWt& operator>>(Mat &m);
#endif // PSINS_IO_FILE
};


#pragma pack()

#endif // _IOFILE_H
