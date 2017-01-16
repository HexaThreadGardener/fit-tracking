#include "includes.h"

#define alpha 0.1			//alpha越接近1，下阈值越大				//***待修改***//
#define beta 2			//beta越接近1，上阈值越小，阈值宽度为学习阈值宽度，阈值宽度越大，对背景的变化容忍越大
#define Epsilon_1 130		//越小，对背景的变化容忍越差
#define Epsilon_2 150		//越小，对背景的变化容忍越差
#define cB_dropTime  80		//丢弃时间，80帧丢弃，帧数指视频原帧数
#define cBB_dropTime 20		//丢弃时间，20帧丢弃，帧数指视频原帧数
#define cBB_keepTime 35		//存活时间35帧

#define jumpframe 3

//码字结构体
//Y：0~255
//U：-122~122
//V：-157~157
typedef struct codeEle {
	int   YMin; // Y分量最小值
	int   YMax; // Y分量最大值
	int   UAve; // U分量平均值
	int   VAve; // V分量平均值
	int   fnum; // 匹配成功次数
	int   lambda; // 最大消极时间
	bool  enable; //是否属于背景
	int   first; // 首次访问时间
	int   last; // 最后访问时间
} code_element;

//码本结构体
typedef struct codeBok {
	code_element    **cb; // 码元的二维指针
	int             numEle; // 此码本中码元的数目 
} code_book;

void updateCodeBook(uchar *p, code_book &cB_c, int t);

uchar backgroundDiff(uchar *p, code_book &cB_c, code_book &cBB_c, int t);

void clearStaleEntries(code_book &c, code_book &cBB_c, int t);
