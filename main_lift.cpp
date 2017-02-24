// scut_vs2012_no_ui.cpp : 定义控制台应用程序的入口点。
//author: Jianhong Zou
//date: 2015-07-29
//本程序调好的参数只适用于2014年9月的录像

#include "Bsums.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <iostream>
#include <opencv2/objdetect/objdetect.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include <vector>
#include <algorithm>

#define CLOSE false
#define OPEN  true


using namespace std;
using namespace cv;

VideoCapture capture; 
double fps;//OpenCV读到的帧率
double totalFrameNumber;//OpenCV读到的总帧数 
double position;//移动的帧位置
Mat frame;//存储视频文件的每一帧

int frames_detected=0;//侦测到的连续的帧数
int frames_nondetected=0;//连续未检测到行人的帧数
vector<Point2f> centers;//连续侦测到行人的重心
Mat bkMat;//存储实时更新的背景图像（灰度图）
Mat frMat;//存储前景图像（灰度图）
Mat reMat;//存储差分二值化图像
Mat roiMat;//存储感兴趣区域（指定房门区域）的差分二值化图像
double width;//帧宽:1280
double height;//帧高:720


#define CV_CVX_WHITE    CV_RGB(0xff,0xff,0xff)
#define CV_CVX_BLACK    CV_RGB(0x00,0x00,0x00)
#define CV_CVX_ZIDINGYI  CV_RGB(110,220,180)

int dealwith(Mat src)
{
	
		Mat ra_region;//右上区域
	    Mat ce_region;//中心区域
	    src(cvRect(855,161,63,150)).copyTo(ra_region);
		src(cvRect(477,320,400,200)).copyTo(ce_region);
		int x1=bSums(ra_region);
		printf("x1=%d\n",x1);
		int x2=bSums(ce_region);
		printf("x2=%d\n",x2);
		int sum=x1+x2;
		printf("dianshu%d",sum);
	    if(sum>3000)
			return 2;//代表进入的是车
		else if(sum>100&&sum<3000)
			return 1;//代表人出去了
		else return 0;//代表人进去了
	
	

}

int main(int argc, char* argv[])
{
	
	
	printf("Start.\n");

	capture.open(argv[1]);
	
	if (!capture.isOpened())//若打开失败
	{
		printf("fail in opening the video file.\n");
	}
	else//若打开成功
	{
		printf("Succeed in opening the video file.\n");
	}
	
	fps = capture.get(CV_CAP_PROP_FPS);
	printf("Frame rate: %f frames per second.\n",fps);
	totalFrameNumber = capture.get(CV_CAP_PROP_FRAME_COUNT);
	printf("Total frame number: The video file contains %f frames.\n",totalFrameNumber);
	width = capture.get(CV_CAP_PROP_FRAME_WIDTH);
	printf("Frame width: %f pixels.\n",width);
	height = capture.get(CV_CAP_PROP_FRAME_HEIGHT);
	printf("Frame height:%f pixels.\n",height);
	
	//Mat trackMat(height,width,CV_8UC3,Scalar(0,0,0));//初始化人运动轨迹
//	Mat blackMat(height,width,CV_8UC3,Scalar(0,0,0));//全黑图像
Mat frMatShow(360,640,CV_8UC1,Scalar(0,0));//缩小显示的的背景图像
Mat reMatShow(360,640,CV_8UC1,Scalar(0,0));//缩小显示的的前景图像

  // IplImage*  ipl;
//	ipl=cvLoadImage ("G:\\实验程序_线控监测\\1.1.JPG",-1);
	/*if (ipl==NULL)//若打开失败
	{
		printf("fail in opening.\n");
	}
	else//若打开成功
	{
		printf("Succeed in opening.\n");
	}*/
	//Mat ipl_1(ipl,true);
	//bkMat=ipl_1;
	Mat ipl_1;
	capture.set(1,500);
	capture>>ipl_1;
	bkMat=ipl_1;
	cvtColor(bkMat,bkMat,CV_BGR2GRAY);

	//capture.set(1,90000);
	Mat left_elavator;  //左电梯图像
	Mat right_elevator;  //右电梯图像
	/*电梯中人的属性判断方法，根据人的行为分析，人一般会在电梯打开一半的时候开始进
	在电梯快要开始关闭时就可以认为人已经进完了，本题中，整个电梯都打开时的大小大概是
	730左右，所以从400时记为人开始进，小于660时证明人已经进完了*/
	bool openflag_left=CLOSE; //电梯开闭标志
	bool openflag_right=CLOSE;

	bool left_in=false;
	bool right_in=false;
	int elevator_in_people=0;//从电梯进入的人数
	int elevator_out_people=0;//从电梯出去的人数
	int circle=0;
	long long completeb[3];
	for(int i=0;i<3;i++)
		completeb[i]=0;
	int countframe=0;
	int countframe2=0;
	int max_in=0;//记录电梯峰值
	int *temp_b=new int[10];
    for(int i=0;i<10;i++)
		temp_b[i]=0;

	bool laststate=false;
	bool laststate2=false;
	while(1)//顺序播放视频
	{
		capture>>frame;
		circle++;
		position = capture.get(CV_CAP_PROP_POS_FRAMES);//position=[1,67698]

		cvtColor(frame,frMat,CV_BGR2GRAY);
		GaussianBlur(frMat,frMat,Size(5,5),0,0);//先对frMat高斯滤波
		
        
		//printf("Frame number: %f\n",position);
        //cv::resize(frMat,frMatShow,frMatShow.size());
		//cv::imshow("视频",frMatShow);
		absdiff(frMat,bkMat,reMat);//reMat为相减之后的Mat
        threshold(reMat,reMat,30/*阈值*/,255.0,THRESH_BINARY/*THRESH_OTSU*/);
		erode(reMat,reMat,cv::Mat(),cv::Point(-1,-1),2);//形态学滤波：腐蚀
		reMat(cvRect(667,90,30,40)).copyTo(left_elavator); //得到左边电梯的开闭情况
		reMat(cvRect(793,108,30,40)).copyTo(right_elevator);  //得到右边电梯的开闭情况

		//右电梯进入开始
		long long  number_b=bSums(right_elevator);
		long long  number_bl=bSums(left_elavator);
		if(openflag_right==CLOSE)
			{
				if(number_b>400&&laststate==CLOSE)
			   {openflag_right=OPEN;laststate=OPEN;}
			   else {countframe=0;laststate=CLOSE;}
	     	}
		if(openflag_right==OPEN)
		{
			if(number_b>400)  //判定为人开始进入
                countframe++;
			else 
				if(laststate==OPEN)
				{right_in=true;printf("右电梯事件结束");}
			    

		}
	    
		if(right_in==true)
		{
			right_in=false;
			openflag_right=CLOSE;
			double temp=(countframe-20)/130;
			int peopleflow=((int)temp+1);
			printf("***%d***\n",peopleflow);
			int j=0;
			j=dealwith(reMat);
			if(j==0)
			elevator_in_people+=peopleflow;
			if(j==1)
		    elevator_out_people+=peopleflow;
			else
				printf("有车");
		}
		if(number_b<50)
			laststate=CLOSE;
		//右电梯结束

		//左电梯开始
		if(openflag_left==CLOSE)
			{
				if(number_bl>400&&laststate2==CLOSE)
			   {openflag_left=OPEN;laststate2=OPEN;}
			   else {countframe2=0;laststate2=CLOSE;}
	     	}
		if(openflag_left==OPEN)
		{
			if(number_bl>400)  //判定为人开始进入
                countframe2++;
			else 
				if(laststate2==OPEN)
				{left_in=true;printf("左电梯进入结束");}
			    

		}
	    
		if(left_in==true)
		{
			left_in=false;
			openflag_left=CLOSE;
			//根据时间计算进入电梯的人数
			double temp=(countframe-20)/130;
			int peopleflow=((int)temp+1);
			printf("***%d***\n",peopleflow);
			int j1=0;
			j1=dealwith(reMat);
			if(j1==0)
			elevator_in_people+=peopleflow;
			if(j1==1)
		    elevator_out_people+=peopleflow;
			else
				printf("有车");
		
		}
		if(number_bl<50)
			laststate2=CLOSE;
		//左电梯结束

		printf("%d\n",elevator_in_people-elevator_out_people);
		//cv::resize(bkMat,reMatShow,reMatShow.size());
	   // cv::imshow("背景",reMatShow);
		addWeighted(bkMat,0.999,frMat,0.001,0,bkMat); 
		rectangle(reMat,cvPoint(477,298),cvPoint(890,518),cvScalar(255,255,255));
		rectangle(reMat,cvPoint(855,160),cvPoint(928,311),cvScalar(255,255,255));
		//背景更新方法，每帧加权法		
		 cv::resize(reMat,reMatShow,reMatShow.size());
		 cv::imshow("视频",frMat);
		 cv::imshow("结果",reMatShow);
	//	cv::imshow("背景",bkMat);


		if(waitKey(1)>=0)
		{
			break;
		}
		
	}
	
	return 0;
}
