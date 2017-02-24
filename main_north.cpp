// scut_vs2012_no_ui.cpp : 定义控制台应用程序的入口点。
//author: 冯健威
//西北门人员检测算法

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
#include"Track.h"


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

int sum_pixels=0;//运动目标的像素数
Moments moment;//矩
int re_x=0;//人体重心的横坐标
int re_y=0;//人体重心的纵坐标
int countframe=0;//计数

#include "ConnectedComponents.h"

#include "Bsums.h"

void refine(Rect &r)
{
	if(r.x<1)
		r.x=1;
	if(r.x>958)
		r.x=958;
	if(r.y<1)
		r.y=1;
	if(r.y>574)
		r.y=574;
	if(r.x+r.width>958)
		r.width=958-r.x;
	if(r.y+r.height>574)
		r.height=574-r.y;
}

double Estimate(double height, int mode)		//mode为-1，下界估计，mode为0，中值估计，mode为1，上界估计
{
	switch(mode)
	{
		case(-1):{return (58.52*height-4618);break;}
		case(0):{return (93.21*height-13440);break;}
		case(1):{return (136.1*height-23954);break;}
	}
}

double round(double val)
{
    return (val> 0.0) ? floor(val+ 0.5) : ceil(val- 0.5);
}



int main(int argc, char* argv[])
{
	Rect *boundings=new Rect[20];
	Rect *storage=new Rect[20];
	Mat display;
	Mat srcMat;
	Mat result;
	int counter=0;
	int i=0,j=0;
	int total;

		double position;//移动的帧位置
	CvPoint *centerofpeople=new CvPoint[10]; //人的中心

    IplImage *former_image;
    IplImage *curr_image;

	  //进出人数记录
   int in_num=0;
   int out_num=0;
   int fit_flag=-1;//fit背景更新的标志
   int fit_count=0;


   //视野中的总运动物体数，就是Mov_head链表的长度
   int insight_num=0;
   int people_in_sight=0;//视野中的总人数
   int countframe=0;//计数
   Movingobject*Mov_head=NULL;
   former_image=cvCreateImage(Size(width,height),8,3);					//用于camshift


	
	printf("Start.\n");


	IplImage *fit_bk;
	fit_bk=cvLoadImage("fit.png",CV_LOAD_IMAGE_UNCHANGED);
	IplImage *fit_bk1;
    fit_bk1=cvLoadImage("fit.png",CV_LOAD_IMAGE_UNCHANGED);

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
	
Mat frMatShow(360,640,CV_8UC1,Scalar(0,0));//缩小显示的的背景图像
Mat reMatShow(360,640,CV_8UC1,Scalar(0,0));//缩小显示的的前景图像

Mat formerimage;
	capture>>frame;
	cvtColor(frame,frMat,CV_BGR2GRAY);
	while(1)//顺序播放视频
	{
		position = capture.get(CV_CAP_PROP_POS_FRAMES);
		if(position==1) {
			IplImage img = IplImage(frame);
			former_image = curr_image = &img;
		}
		frMat.copyTo(formerimage);
		capture>>frame;
		capture>>frame;
		capture>>frame;
		
		cvtColor(frame,frMat,CV_BGR2GRAY);
		GaussianBlur(frMat,frMat,Size(5,5),0,0);//先对frMat高斯滤波
		position = capture.get(CV_CAP_PROP_POS_FRAMES);//position=[1,67698]
		
		printf("Frame number: %f\n",position);
		absdiff(frMat,formerimage,reMat);//reMat为相减之后的Mat

		threshold(reMat,reMat,10,255.0,THRESH_BINARY);
		erode(reMat,reMat,cv::Mat(),cv::Point(-1,-1),1);//形态学滤波：腐蚀
		dilate(reMat,reMat,cv::Mat(),cv::Point(-1,-1),10);//形态学滤波：膨胀
		
		

		int numbersd=0;
		 ConnectedComponents(reMat,1,250,numbersd,1,storage,Point(-1,-1));

		 cv::resize(reMat,reMatShow,reMatShow.size());
		 cv::imshow("reMat",reMatShow);
		 frame.copyTo(display);
		for (j=0,i = 0; i < numbersd; i++)
		{
			Rect r = storage[i];
			refine(r);
			if(counter>0&&r.area()>10&&(r.y+0.5*r.height)>280&&(r.y+0.5*r.height)<420)		//counter>0仅仅是为了舍弃第一帧
			{
				Mat tmp=reMat(r);
				int number=bSums(tmp);
				double estimate_l=Estimate(r.y+0.5*r.height,-1);
				double estimate_m=Estimate(r.y+0.5*r.height,0);
				if((double)number<estimate_l*0.8)
					continue;
				double divide=(double)number/estimate_m;
				int div=(int)round(divide);
				for(int k=0;k<div;k++,j++)
				{
					boundings[j].x=(int)(r.x+k*r.width/div);
					boundings[j].y=r.y;
					boundings[j].width=(int)((double)r.width/div);
					boundings[j].height=r.height;
				}	
			}
		}
		total=j;
		if(total!=0)
			printf("total:%d\n",total);
		
        
		 //跟踪代码起始位置
		frame.copyTo(srcMat);
		srcMat.copyTo(result);
		 IplImage img = IplImage(srcMat);
		 curr_image=&img;
		 numbersd=total;		//替换

		 if(insight_num==0)  //此时运动物体链表为空
		 {
			 Movingobject *temp_a;
			 for(int k=0;k<numbersd;k++)
			 {
				 temp_a=InitMovingobject();
				 temp_a->points[0]=cvPoint(boundings[k].x,boundings[k].y);
				 temp_a->points[1]=cvPoint(boundings[k].x+boundings[k].width,boundings[k].y+boundings[k].height);
				 temp_a->length=1;
				 temp_a->head->x=(int)(boundings[k].x+boundings[k].width/2);
				 temp_a->head->y=(int)(boundings[k].y+boundings[k].height/2);
				 temp_a->head->next=NULL;
				 temp_a->tail=temp_a->head;
				 Mov_head=add_Movobj(Mov_head,temp_a);
			 }
			 sort_obj(Mov_head);
			 insight_num=numbersd;
		 }

		 else 
		 {
			 int *Label_rec=new int[numbersd];
			 int *Label_rec_f=new int[numbersd];
			 int label100=0;
			 int label200=0;//记录100以内矩形框的个数
			 int Center_dis=0;
			 Movingobject*temp_c=Mov_head;
			 //初始化标记数组
			 for(int i=0;i<numbersd;i++)
	              Label_rec[i]=Label_rec_f[i]=0;

			 while(temp_c!=NULL)
			 {
				 CvPoint prediction_pos=track_kalman_predict(temp_c);  //卡尔曼预测得到坐标点
				 for(int i=0;i<numbersd;i++)
	              Label_rec[i]=0;
				 for(int j=0;j<numbersd;j++)  //循环扫描未匹配的矩形
				 {
					 if(Label_rec_f[j]==0)
					 {
						 Center_dis=cal_distance(boundings[j].x+boundings[j].width/2,boundings[j].y+boundings[j].height/2,
							                     prediction_pos.x,prediction_pos.y);
						 if(Center_dis<50)
						 { Label_rec[j]=1; label100++;}
						 if(Center_dis>=50&&Center_dis<100)
						 { Label_rec[j]=2; label200++;}
					 }
				 }
				 if(label100==1)
				 {
					 int m;
					 for(m=0;m<numbersd;m++)
					 {
						 if(Label_rec[m]==1)
							 break;
					 }
					 temp_c->area=0;
					 temp_c->points[0]=cvPoint(boundings[m].x,boundings[m].y);
					 temp_c->points[1]=cvPoint(boundings[m].x+boundings[m].width,boundings[m].y+boundings[m].height);
					 temp_c->length++;
					 ObjectList *temp_e=new ObjectList();
					 temp_e->x=(int)(boundings[m].x+boundings[m].width/2);
					 temp_e->y=(int)(boundings[m].y+boundings[m].height/2);
					 temp_e->next=NULL;
					 temp_c->tail->next=temp_e;
					 temp_c->tail=temp_e;
					 track_predict_update(temp_c,cvPoint(temp_e->x,temp_e->y));
					 Label_rec_f[m]=1;

				 }
				 else if(label200==1)  //单独写成，方便为跳帧提供参数
				 {
					 int m;
					 temp_c->area=0;
					 for(m=0;m<numbersd;m++)
					 {
						 if(Label_rec[m]==2)
							 break;
					 }
					 temp_c->points[0]=cvPoint(boundings[m].x,boundings[m].y);
					 temp_c->points[1]=cvPoint(boundings[m].x+boundings[m].width,boundings[m].y+boundings[m].height);
					 temp_c->length++;
					  ObjectList *temp_e=new ObjectList();
					 temp_e->x=(int)(boundings[m].x+boundings[m].width/2);
					 temp_e->y=(int)(boundings[m].y+boundings[m].height/2);
					 temp_e->next=NULL;
					 temp_c->tail->next=temp_e;
					 temp_c->tail=temp_e;
					 track_predict_update(temp_c,cvPoint(temp_e->x,temp_e->y));
					 Label_rec_f[m]=1;
				 }
				 else if(label100>1||(label200>1&&label100>1))
				 {
					 temp_c->area=0;
					 Rect temp_cam=track_camshift(temp_c,former_image,curr_image);
					 temp_cam=ChangeRect(cvRect(0,0,winWidth,winHeight),temp_cam);
					 if(temp_cam.width*temp_cam.height>0)
					 {
						 int m, min_dis=9999,min_dis_label=0;
						 for(m=0;m<numbersd;m++)
							 if(Label_rec_f[m]==0)
							 {
								 int dis=cal_distance(boundings[m].x+boundings[m].width/2,boundings[m].y+boundings[m].height/2,
								                    temp_cam.x+temp_cam.width/2,temp_cam.y+temp_cam.height/2);
							     if(dis<min_dis)  { min_dis=dis; min_dis_label=m; }
							 }
						  m=min_dis_label;
						  temp_c->points[0]=cvPoint(boundings[m].x,boundings[m].y);
						  temp_c->points[1]=cvPoint(boundings[m].x+boundings[m].width,boundings[m].y+boundings[m].height);
						  temp_c->length++;
					       ObjectList *temp_e=new ObjectList();
						  temp_e->x=(int)(boundings[m].x+boundings[m].width/2);
					      temp_e->y=(int)(boundings[m].y+boundings[m].height/2);
					      temp_e->next=NULL;
					      temp_c->tail->next=temp_e;
					      temp_c->tail=temp_e;
					      track_predict_update(temp_c,cvPoint(temp_e->x,temp_e->y));
					      Label_rec_f[m]=1;
					 }
				 }
					
				 else   //未检索到
				 {
					  if(temp_c->area==5)
						 {
							 Movingobject*temp_f=temp_c;
							 temp_c=temp_c->next;
							 flags=false;
							 Mov_head=delete_Movobj(Mov_head,temp_f,&in_num,&out_num,&fit_flag);
							 sort_obj(Mov_head);
							 insight_num--;
						 }
						 else 
						 { 
							 temp_c->area++;
						 }
				 }
				 if(flags==true)
				 temp_c=temp_c->next;
				 else flags=true;
				 label100=0;
				 label200=0;
				 shorten_objlist(Mov_head);
				 }

				 //将剩余的矩形全部初始化为运动对象
			Movingobject *temp_a;
			for(int k=0;k<numbersd;k++)
		   {
			 if(Label_rec_f[k]==0)
	            {
                 temp_a=InitMovingobject();
				 temp_a->points[0]=cvPoint(boundings[k].x,boundings[k].y);
				 temp_a->points[1]=cvPoint(boundings[k].x+boundings[k].width,boundings[k].y+boundings[k].height);
				 temp_a->length=1;
				 temp_a->head->x=(int)(boundings[k].x+boundings[k].width/2);
				 temp_a->head->y=(int)(boundings[k].y+boundings[k].height/2);
				 temp_a->head->next=NULL;
				 temp_a->tail=temp_a->head;
				 Mov_head=add_Movobj(Mov_head,temp_a);
				 insight_num++;
				 }
			 }
				
		sort_obj(Mov_head);
		for(int i=0;i<numbersd;i++)  //标记数组归零
		  Label_rec[i]=Label_rec_f[i]=0;
	

		}
		//开始划线和矩形框
		draw_rectangle(result,Mov_head);
	
		former_image=curr_image;
		//文字输出，输出视野中的总人数

		people_in_sight=cal_people_in_sight(Mov_head);
		char buffer[256];
		printf("The number of people in sight is :%d",people_in_sight);
		putText(result,buffer,cvPoint(5,485),CV_FONT_HERSHEY_COMPLEX,0.8,cvScalar(20,20,240),2,8);
		printf("The number of people inside is :%d",in_num);
		putText(result,buffer,cvPoint(5,500),CV_FONT_HERSHEY_COMPLEX,0.8,cvScalar(20,20,240),2,8);
		printf("The number of people outside is :%d",out_num);
		putText(result,buffer,cvPoint(5,515),CV_FONT_HERSHEY_COMPLEX,0.8,cvScalar(20,20,240),2,8);
		Mat result_narrow(360,640,CV_8UC3,Scalar(0,0,0));//缩小显示的的前景图像
		resize(result,result_narrow,result_narrow.size());
		imshow("result_narrow",result_narrow); 

		//fit实时更新
		CvFont font;
		cvInitFont(&font,CV_FONT_HERSHEY_COMPLEX_SMALL,1,1,(0,0),2,8);

		if(position==3)
		{ 
			printf("The number of people in FIT is :%d",in_num);
			cvPutText(fit_bk,buffer,cvPoint(199,58),&font,CV_RGB(0,0,0)); 
		}

		if(fit_flag!=-1)
			{  
			
				Fit_bk(fit_bk,fit_flag);
				fit_count++;
			}
		if(fit_count>=5)
		{
			fit_count=0;
			fit_flag=-1;
			fit_bk=cvCloneImage(fit_bk1);
		
			printf("The number of people in FIT is :%d",in_num);
			cvPutText(fit_bk,buffer,cvPoint(199,58),&font,CV_RGB(0,0,0)); 
		}
		cvShowImage("FIT",fit_bk);
		printf("楼内人数为：");
		printf("\n");



	
		
		imshow("srcMat", srcMat);
		moveWindow("srcMat", 0, 0);


		if(waitKey(1)>=0)
		{
			break;
		}
		counter++;
	}

	return 0;
}
