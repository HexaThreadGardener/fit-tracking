// scut_vs2012_no_ui.cpp : 定义控制台应用程序的入口点。
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

#include "ConnectedComponents.h"

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

void dealwith(Mat src,int *temp,int position)
{
	if(position%15==0)
	{
		Mat ra_region;//右上区域
	    Mat ce_region;//中心区域
	    src(cvRect(863,161,63,176)).copyTo(ra_region);
		src(cvRect(459,134,300,200)).copyTo(ce_region);
		int x1=bSums(ra_region);
		int x2=bSums(ce_region);
		int sum=x1+x2;
		for(int i=0,j=1;i<9;i++,j++)
			temp[i]=temp[j];
		if(sum>30000)
			temp[9]=0;
		else temp[9]=sum;
	}

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
	//capture.set(1,1000);
	Mat ipl_1;
	capture.set(1,160);
	capture>>ipl_1;
    Rect * boundings=new Rect[10];
	Rect * boundings3=new Rect[4];
	Rect * boundings4=new Rect[4];
	Rect * boundings5=new Rect[4];
	Rect * boundings6=new Rect[4];
	//ipl=cvLoadImage ("G:\\实验程序_线控监测\\1.1.JPG",-1);
	/*if (ipl==NULL)//若打开失败
	{
		printf("fail in opening.\n");
	}
	else//若打开成功
	{
		printf("Succeed in opening.\n");
	}*/
	//Mat ipl_1(ipl,true);
	bkMat=ipl_1;
	cvtColor(bkMat,bkMat,CV_BGR2GRAY);
	//197800  199400
	capture.set(1,199800);
	Mat Area1;  //1区域
	Mat Area2;  //2区域
	Mat Area2_1;//2区域分块
	Mat Area3;  //3区域
	Mat Area4;
	Mat Area5;
	Mat Area6;
	Mat Area7;  
	//Mat Area4;  //4区域
	/*电梯中人的属性判断方法，根据人的行为分析，人一般会在电梯打开一半的时候开始进
	在电梯快要开始关闭时就可以认为人已经进完了，本题中，整个电梯都打开时的大小大概是
	730左右，所以从400时记为人开始进，小于660时证明人已经进完了*/
	bool openflag_left=CLOSE; //电梯开闭标志
	bool openflag_right=CLOSE;

	bool left_in=false;
	bool right_in=false;
	int in_people=0;//从楼梯进入停车库的人数
	int out_people=0;//从停车库的楼梯出去的人数
	int p_count=0;//当前楼梯内人数
	int p_countbuf=0;
	int c3=0;
	int c4=0;
	int c5=0;
	int c6=0;
	int c3_buf=0;
	int c4_buf=0;
	int c5_buf=0;
	int c6_buf=0;
	int dir[10];
	bool is_car=0;
	int carc=0;
	int car_count=0;//左转的车子的数量;

	int c_buf[5];//人数的缓存，用来滤掉高频分量
	int l=0;
	for(int q=0;q<10;q++)dir[q]=0;
	double p_count1=0;
	int circle=0;
	long long completeb[3];
	for(int i=0;i<3;i++)
		completeb[i]=0;
	int countframe=0;
	int max_in=0;//记录电梯峰值
	int *temp_b=new int[10];
    for(int i=0;i<10;i++)
		temp_b[i]=0;

	bool laststate=false;
	while(1)//顺序播放视频
	{
		for(int t=0;t<2;t++)
			capture>>frame;
		circle++;
		position = capture.get(CV_CAP_PROP_POS_FRAMES);//position=[1,67698]

		cvtColor(frame,frMat,CV_BGR2GRAY);
		GaussianBlur(frMat,frMat,Size(3,3),0,0);//先对frMat高斯滤波
		
        
		//printf("Frame number: %f\n",position);

       
		Scalar color = CV_RGB(0,0,255);
	//	 rectangle(	 frMatShow,	cvPoint(89,375),cvPoint(1248,700),color,3,8,0);
		
		absdiff(frMat,bkMat,reMat);//reMat为相减之后的Mat
        threshold(reMat,reMat,40/*阈值*/,255.0,THRESH_BINARY/*THRESH_OTSU*/);
		
		dealwith(reMat,temp_b,position);
		reMat(cvRect(241,124,50,50)).copyTo(Area1); //
		reMat(cvRect(412,394,499,316)).copyTo(Area2);  //
		reMat(cvRect(403,505,517,13)).copyTo(Area3); //
		reMat(cvRect(418,434,487,16)).copyTo(Area4); //
		reMat(cvRect(394,582,531,11)).copyTo(Area5); //
		reMat(cvRect(390,656,487,18)).copyTo(Area6); //
		reMat(cvRect(495,297,50,50)).copyTo(Area7); 
		erode(Area2,Area2,cv::Mat(),cv::Point(-1,-1),1);//形态学滤波：腐蚀
		dilate(Area2,Area2,cv::Mat(),cv::Point(-1,-1),5);
	//	reMat(cvRect(412,394,499,316)).copyTo(Area2);  //
		//右电梯进入开始
		long long number_b=bSums(Area1);
		long long number_t=bSums(Area7);
	//	long long number_c=bSums(Area2);
//		long long number_d=bSums(Area3)+bSums(Area4)+bSums(Area5)+bSums(Area6);
		
		//无车灯情况，直接对敏感区域采点。
		if(number_b==0&&number_t<1000)
		{
			if(carc>60)
			{
				if(is_car==true)car_count++;
				is_car=false;
				
			p_countbuf=p_count;
			ConnectedComponents(Area2, 0, 10000, p_count, 1, boundings, Point(-1, -1));
			
			int temp2=p_count;
			
			for(int i=0;i<p_count;i++)
				rectangle(frMat,cvPoint(boundings[i].x+412,boundings[i].y+394),cvPoint(boundings[i].x+boundings[i].width+412,boundings[i].y+boundings[i].height+394),cvScalar(0,0,255),3);
			
			for(int i=0;i<temp2;i++)
			{
				if(boundings[i].width>360)
					p_count++;
				/*if(boundings[i].height>300)
					p_count++;*/
			}
			temp2=p_count;

			


			p_count1=(p_count1+double(p_count))/2;
			int temp=p_count1;
			int temp1=p_count;
			if(p_count1-temp>0.8)p_count=temp+1;
			else p_count=temp;





			//p_count=temp2;

			c_buf[l]=p_count;
			l=(l+1)%5;

			if(p_count-p_countbuf==1)//假定人数连续变化
			{
				for(int l=0;l<temp1;l++)
				{
					if(boundings[l].y>60)
					{
						in_people++;
						break;
					}
					else
						if(boundings[l].y<20)
						{
							out_people++;
							break;
						}
				}
			}
			}
			else
				carc++;
		}
		else
			if(number_b==2500)//舍弃过度区域
		{
			is_car=true;
			carc=0;
			c3_buf=c3;
			c6_buf=c6;
			ConnectedComponents(Area3, 0, 200, c3, 1, boundings3, Point(-1, -1));
			ConnectedComponents(Area3, 0, 100, c4, 1, boundings4, Point(-1, -1));
			ConnectedComponents(Area3, 0, 100, c5, 1, boundings5, Point(-1, -1));
			ConnectedComponents(Area3, 0, 200, c6, 1, boundings6, Point(-1, -1));
			p_countbuf=p_count;
			p_count=c3+c6;
			if(p_count-p_countbuf==1)
			{
				if(c3-c3_buf==1)
					out_people++;
				if(c6-c6_buf==1)
					in_people++;
			}
		}

	/*	if(openflag_right==CLOSE)
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
				{right_in=true;printf("进入结束");}
			    

		}
	    
		if(right_in==true)
		{
			right_in=false;
			openflag_right=CLOSE;
			double temp=(countframe-20)/130;
			int peopleflow=((int)temp+1);
			printf("***%d***\n",peopleflow);
			int j=0;
			for(;j<10;j++)
				if(temp_b[j]!=0)
					break;
			if(j<10)
			in_people+=peopleflow;
			if(j==10)
		    out_people+=peopleflow;
		
		}
		if(number_b<50)
			laststate=CLOSE;
			*/
		cv::resize(frMat,frMatShow,frMatShow.size());
		char buffer[256];
		printf("The number of people in sight is :%d",p_count);
		putText(frMatShow,buffer,cvPoint(5,325),CV_FONT_HERSHEY_COMPLEX,0.8,cvScalar(20,20,240),2,8);
		printf("The number of people inside is :%d",in_people);
		putText(frMatShow,buffer,cvPoint(5,340),CV_FONT_HERSHEY_COMPLEX,0.8,cvScalar(20,20,240),2,8);
		printf("The number of people outside is :%d",out_people);
		putText(frMatShow,buffer,cvPoint(5,355),CV_FONT_HERSHEY_COMPLEX,0.8,cvScalar(20,20,240),2,8);
		cv::imshow("视频",frMatShow);
		printf("%d ",number_b);
		printf("%d  ",p_count);
		printf(" in人数：%d",in_people);
		printf(" out人数：%d", out_people);
		printf("当前内部人数：%d",in_people-out_people);
		printf("左转的车数：%d\n",car_count);
		cv::resize(bkMat,reMatShow,reMatShow.size());
	//    cv::imshow("背景",reMatShow);
		addWeighted(bkMat,0.999,frMat,0.001,0,bkMat);    //背景更新方法，每帧加权法		
		 cv::resize(reMat,reMatShow,reMatShow.size());
		 
		 cv::imshow("结果",reMatShow);
		
		if(waitKey(1)>=0)
		{
			break;
		}
		
	}
	
	return 0;
}

