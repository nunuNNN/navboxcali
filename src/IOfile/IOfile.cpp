#include "IOfile.h"


//******************************  File Read or Write *********************************/
char* time2fname(void)
{
	static char PSINSfname[32];
	time_t tt;  time(&tt);
	tm *Time = localtime(&tt);
	strftime(PSINSfname, 32, "PSINS%Y%m%d_%H%M%S.bin", Time);
	return PSINSfname;
}

char CFileRdWt::dirIn[256] = {0}, CFileRdWt::dirOut[256] = {0};

void CFileRdWt::DirI(const char *dirI)  // set dirIN
{
	int len = strlen(dirI);
	memcpy(dirIn, dirI, len);
	if(dirIn[len-1]!='/') { dirIn[len]='/'; dirIn[len+1]='\0'; }
	if(access(dirIn,0)==-1) dirIn[0]='\0';
}

void CFileRdWt::DirO(const char *dirO)  // set dirOUT
{
	int len = strlen(dirO);
	memcpy(dirOut, dirO, len);
	if(dirOut[len-1]!='/') { dirOut[len]='/'; dirOut[len+1]='\0'; }
	if(access(dirOut,0)==-1) dirOut[0]='\0';
}

void CFileRdWt::Dir(const char *dirI, const char *dirO)  // set dirIN&OUT
{
	DirI(dirI);
	DirO(dirO?dirO:dirI);
}

CFileRdWt::CFileRdWt()
{
	rwf = 0;
}

CFileRdWt::CFileRdWt(const char *fname0, int columns0)
{
	rwf = 0;
	Init(fname0, columns0);
	memset(buff, 0, sizeof(buff));
}

CFileRdWt::~CFileRdWt()
{
	if(rwf) { fclose(rwf); rwf=(FILE*)NULL; } 
}

void CFileRdWt::Init(const char *fname0, int columns0)
{
	fname[0]='\0';
	int findc=0, len0=strlen(fname0);
	for(int i=0; i<len0; i++)	{ if(fname0[i]=='\\') { findc=1; break; } }
	columns = columns0;
	if(columns==0)		// file write
	{	if(dirOut[0]!=0&&findc==0)	{ strcat(fname, dirOut); } }
	else				// file read
	{	if(dirIn[0]!=0&&findc==0)	{ strcat(fname, dirIn); } }
	strcat(fname, fname0);

	if(rwf) this->~CFileRdWt();
	if(columns==0)				// bin file write
	{
		rwf = fopen(fname, "wb");
	}
	else if(columns<0)			// bin file read
	{
		rwf = fopen(fname, "rb");
		if(rwf==NULL)           /*判断文件是否打开成功*/
			printf("File open error\r\n");
	}
	else if(columns>0)			// txt file read
	{
		rwf = fopen(fname, "rt");
		if(!rwf) return;
		fpos_t pos;
		while(1)  // skip txt-file comments
		{
			/*
			* ftell与fseek一起使用，fsetpos与fgetpos一起使用。
			* ftell与fseek返回的是长整数
			* fsetpos与fgetpos返回一个新类型（fpos_t）
			*/
			// pos = ftell(rwf);
			fgetpos(rwf, &pos);
			fgets(line, sizeof(line), rwf);
			if(feof(rwf)) break;
			int allSpace=1, allDigital=1;
			for(int i=0; i<sizeof(line); i++)
			{
				char c = line[i];
				if(c=='\n') break;
				if(c!=' ') allSpace = 0;
				if( !(c==' '||c==','||c==';'||c==':'||c=='+'||c=='-'||c=='.'||c=='\t'
					||c=='e'||c=='E'||c=='d'||c=='D'||('9'>=c&&c>='0')) )
					allDigital = 0;
			}
			if(!allSpace && allDigital) break;
		}
		fsetpos(rwf, &pos);
		this->columns = columns;
		for(int i=0; i<columns; i++)
		{ sstr[4*i+0]='%', sstr[4*i+1]='l', sstr[4*i+2]='f', sstr[4*i+3]=' ', sstr[4*i+4]='\0'; } 
	}
	else
	{
		rwf = 0;
	}
	linek = 0;
	totsize = 0;
}

int CFileRdWt::load(int lines, BOOL txtDelComma)
{
	if(columns<0)			// bin file read
	{
		if(lines>1)	fseek(rwf, (lines-1)*(-columns)*sizeof(double), SEEK_CUR);
		fread(buff, -columns, sizeof(double), rwf);
		if(lines==0) fseek(rwf, columns*sizeof(double), SEEK_CUR);
	}
	else					// txt file read
	{
		for(int i=0; i<lines; i++)  fgets(line, sizeof(line), rwf);
		if(txtDelComma)
		{
			for(char *pc=line, *pend=line+sizeof(line); pc<pend; pc++)
			{
				if(*pc==','||*pc==';'||*pc==':'||*pc=='\t:') *pc=' ';
				else if(*pc=='\0') break;
			}
		}
		if(columns<10)
			sscanf(line, sstr,
				&buff[ 0], &buff[ 1], &buff[ 2], &buff[ 3], &buff[ 4], &buff[ 5], &buff[ 6], &buff[ 7], &buff[ 8], &buff[ 9]
				); 
		else if(columns<20)
			sscanf(line, sstr,
				&buff[ 0], &buff[ 1], &buff[ 2], &buff[ 3], &buff[ 4], &buff[ 5], &buff[ 6], &buff[ 7], &buff[ 8], &buff[ 9],
				&buff[10], &buff[11], &buff[12], &buff[13], &buff[14], &buff[15], &buff[16], &buff[17], &buff[18], &buff[19]
				); 
		else if(columns<40)
			sscanf(line, sstr,
				&buff[ 0], &buff[ 1], &buff[ 2], &buff[ 3], &buff[ 4], &buff[ 5], &buff[ 6], &buff[ 7], &buff[ 8], &buff[ 9],
				&buff[10], &buff[11], &buff[12], &buff[13], &buff[14], &buff[15], &buff[16], &buff[17], &buff[18], &buff[19],
				&buff[20], &buff[21], &buff[22], &buff[23], &buff[24], &buff[25], &buff[26], &buff[27], &buff[28], &buff[29],
				&buff[30], &buff[31], &buff[32], &buff[33], &buff[34], &buff[35], &buff[36], &buff[37], &buff[38], &buff[39]
				); 
	}
	linek += lines;
	if(feof(rwf))  return 0;
	else return 1;
}

int CFileRdWt::loadf32(int lines)	// float32 bin file read
{
	if(lines>1)
		fseek(rwf, (lines-1)*(-columns)*sizeof(float), SEEK_CUR);
	fread(buff32, -columns, sizeof(float), rwf);
	for(int i=0; i<-columns; i++) buff[i]=buff32[i];
	linek += lines;
	if(feof(rwf))  return 0;
	else return 1;
}

long CFileRdWt::load(BYTE *buf, long bufsize)  // load bin file
{
	long cur = ftell(rwf);
	fseek(rwf, 0L, SEEK_END);
	long rest = ftell(rwf)-cur;
	fseek(rwf, cur, SEEK_SET);
	if(bufsize>rest) bufsize=rest;
	fread(buf, bufsize, 1, rwf);
	return bufsize;
}

void CFileRdWt::bwseek(int lines, int mod)  // double-bin file backward-seek lines
{
	fseek(rwf, lines*(-columns)*sizeof(double), mod);
	linek -= lines;
}

long CFileRdWt::filesize(int opt)
{
	long cur = ftell(rwf);
	if(totsize==0)
	{
		fseek(rwf, 0L, SEEK_END);
		totsize = ftell(rwf);			// get total_size
		fseek(rwf, cur, SEEK_SET);
	}
	remsize = totsize-cur;
	return opt ? remsize : totsize;  // opt==1 for remain_size, ==0 for total_size
}

int CFileRdWt::getl(void)	// txt file get a line
{
	fgets(line, sizeof(line), rwf);
	return strlen(line);
}

BOOL CFileRdWt::waitfor(int columnk, double val, double eps)
{
	double wf=buff[columnk]-val;
	while(-eps<wf && wf<eps) {
		load(1);
		if(feof(rwf)) return 0;
		wf = buff[columnk] - val;
	}
	return 1;
}

#ifdef PSINS_IO_FILE

CFileRdWt& CFileRdWt::operator<<(double d)
{
	fwrite(&d, 1, sizeof(double), rwf);
	return *this;
}

CFileRdWt& CFileRdWt::operator<<(const Vect3 &v)
{
	fwrite(&v, 1, sizeof(v), rwf);
	return *this;
}

CFileRdWt& CFileRdWt::operator<<(const Quat &q)
{
	fwrite(&q, 1, sizeof(q), rwf);
	return *this;
}

CFileRdWt& CFileRdWt::operator<<(const Mat3 &m)
{
	fwrite(&m, 1, sizeof(m), rwf);
	return *this;
}

CFileRdWt& CFileRdWt::operator<<(const Vect &v)
{
	fwrite(v.dd, v.clm*v.row, sizeof(double), rwf);
	return *this;
}

CFileRdWt& CFileRdWt::operator<<(const Mat &m)
{
	fwrite(m.dd, m.clm*m.row, sizeof(double), rwf);
	return *this;
}

CFileRdWt& CFileRdWt::operator<<(const SINS &sins)
{
	if(sins.isOutlever)
		return *this<<sins.att<<sins.vnL<<sins.posL<<sins.eb<<sins.db<<sins.tk;
	else
		return *this<<sins.att<<sins.vn<<sins.pos<<sins.eb<<sins.db<<sins.tk;
}

CFileRdWt& CFileRdWt::operator<<(Kalman &kf)
{
	*this<<kf.Xk<<diag(kf.Pk)<<kf.Zk<<kf.Rt<<(double)kf.measflaglog<<kf.kftk;
	kf.measflaglog = 0;
	return *this;
}

CFileRdWt& CFileRdWt::operator>>(double &d)
{
	fread(&d, 1, sizeof(double), rwf);
	return *this;
}

CFileRdWt& CFileRdWt::operator>>(Vect3 &v)
{
	fread(&v, 1, sizeof(v), rwf);
	return *this;
}

CFileRdWt& CFileRdWt::operator>>(Quat &q)
{
	fread(&q, 1, sizeof(q), rwf);
	return *this;
}

CFileRdWt& CFileRdWt::operator>>(Mat3 &m)
{
	fread(&m, 1, sizeof(m), rwf);
	return *this;
}

CFileRdWt& CFileRdWt::operator>>(Vect &v)
{
	fread(v.dd, v.clm*v.row, sizeof(double), rwf);
	return *this;
}

CFileRdWt& CFileRdWt::operator>>(Mat &m)
{
	fread(m.dd, m.clm*m.row, sizeof(double), rwf);
	return *this;
}

#endif // PSINS_IO_FILE

