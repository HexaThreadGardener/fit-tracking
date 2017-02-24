// Codebook1.0.cpp : 定义控制台应用程序的入口点。
//

#include "includes.h"
#include "codebook.h"
#include "Track.h"
#include <string>
#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/objdetect/objdetect.hpp>
#include <opencv2/ml/ml.hpp>

using namespace std;
using namespace cv;

VideoCapture capture;
code_book*   cB;
code_book*   cBB;
double       frameNum;
double       width;
double       height;
Mat          temp;
Mat          srcMat;
Mat          bkMat;
int          imageLen;
uchar*       bkPtr;
uchar*       frPtr;

#define CV_CVX_WHITE    CV_RGB(0xff,0xff,0xff)
#define CV_CVX_BLACK    CV_RGB(0x00,0x00,0x00)
#define CV_CVX_ZIDINGYI  CV_RGB(110,220,180)

void ConnectedComponents(Mat &mask_process, int poly1_hull0, float scale, int &p, int number,
	Rect * &bounding_box, const Point &contour_centers = Point(-1, -1))
{
	/*下面4句代码是为了兼容原函数接口，即内部使用的是c风格，但是其接口是c++风格的*/
	IplImage mask = IplImage(mask_process);
	int *num = &number;
	CvRect *bbs = (CvRect *)bounding_box;
	CvPoint centers = CvPoint(contour_centers);
	static CvMemStorage*    mem_storage = NULL;
	static CvSeq*            contours = NULL;

	//CLEAN UP RAW MASK
	//开运算作用：平滑轮廓，去掉细节,断开缺口
	cvMorphologyEx(&mask, &mask, NULL, NULL, CV_MOP_OPEN, 1);//对输入mask进行开操作，CVCLOSE_ITR为开操作的次数，输出为mask图像
	cvErode(&mask, &mask, NULL, 2);
	cvMorphologyEx(&mask, &mask, NULL, NULL, CV_MOP_CLOSE, 1);//对输入mask进行闭操作，CVCLOSE_ITR为闭操作的次数，输出为mask图像

	if (mem_storage == NULL)
		mem_storage = cvCreateMemStorage(0);
	else
		cvClearMemStorage(mem_storage);
	//CV_RETR_EXTERNAL=0是在types_c.h中定义的，CV_CHAIN_APPROX_SIMPLE=2也是在该文件中定义的

	//寻找连通域，外包矩形
	CvContourScanner scanner = cvStartFindContours(&mask, mem_storage, sizeof(CvContour), CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
	CvSeq* c;
	int numCont = 0;
	CvRect rect_temp;
	//该while内部只针对比较大的轮廓曲线进行替换处理
	while ((c = cvFindNextContour(scanner)) != NULL)
	{
		double area = cvContourArea(c);
		if (area < scale) // 太小
		{
			cvSubstituteContour(scanner, NULL);
			continue;//用NULL代替原来的那个轮廓
		}
		rect_temp = cvBoundingRect(c);
		if ((rect_temp.y + rect_temp.height) < 30/*360*/ || rect_temp.y > 330/*450*/)
		{
			cvSubstituteContour(scanner, NULL);
			continue;//用NULL代替原来的那个轮廓
		}

		CvSeq* c_new;
		if (poly1_hull0)
			c_new = cvApproxPoly(c, sizeof(CvContour), mem_storage, CV_POLY_APPROX_DP, 2, 0);
		else //Convex Hull of the segmentation
			c_new = cvConvexHull2(c, mem_storage, CV_CLOCKWISE, 1);
		cvSubstituteContour(scanner, c_new); //最开始的轮廓用凸包或者多项式拟合曲线替换
		numCont++;

	}
	contours = cvEndFindContours(&scanner);    //结束轮廓查找操作
	p = numCont;
	// PAINT THE FOUND REGIONS BACK INTO THE IMAGE
	cvZero(&mask);
	IplImage *maskTemp;
	//CALC CENTER OF MASS AND OR BOUNDING RECTANGLES
	if (*num != 0)
	{
		int N = *num, numFilled = 0, i = 0;
		CvMoments moments;
		double M00, M01, M10;
		maskTemp = cvCloneImage(&mask);
		for (i = 0, c = contours; c != NULL; c = c->h_next, i++)        //h_next为轮廓序列中的下一个轮廓
		{
			if (i < N) //Only process up to *num of them
			{
				//CV_CVX_WHITE在本程序中是白色的意思
				cvDrawContours(maskTemp, c, CV_CVX_WHITE, CV_CVX_WHITE, -1, CV_FILLED, 8);
				//Find the center of each contour
				if (centers.x != -1 || centers.y != -1)
				{
					cvMoments(maskTemp, &moments, 1);    //计算mask图像的最高达3阶的矩
					M00 = cvGetSpatialMoment(&moments, 0, 0); //提取x的0次和y的0次矩
					M10 = cvGetSpatialMoment(&moments, 1, 0); //提取x的1次和y的0次矩
					M01 = cvGetSpatialMoment(&moments, 0, 1); //提取x的0次和y的1次矩
					(&centers)[i].x = (int)(M10 / M00);    //利用矩的结果求出轮廓的中心点坐标
					(&centers)[i].y = (int)(M01 / M00);
				}
				//Bounding rectangles around blobs
				/*         if(bbs != &CvRect())
				{
				bbs[i] = cvBoundingRect(c); //算出轮廓c的外接矩形
				}*/
				cvZero(maskTemp);
				numFilled++;
			}
			//Draw filled contours into mask
			cvDrawContours(&mask, c, CV_CVX_ZIDINGYI, CV_CVX_WHITE, -1, CV_FILLED, 8); //draw to central mask
//			if (bbs != CvRect())
//			{
				bbs[i] = cvBoundingRect(c); //算出轮廓c的外接矩形
//			}
		} //end looping over contours
		*num = numFilled;
		cvReleaseImage(&maskTemp);
	}
	//ELSE JUST DRAW PROCESSED CONTOURS INTO THE MASK
	else
	{
		for (c = contours; c != NULL; c = c->h_next)
		{
			cvDrawContours(&mask, c, CV_CVX_WHITE, CV_CVX_BLACK, -1, CV_FILLED, 8);
		}
	}
}



int main(int argc, char* argv[])
{
	int numbersd = 0;
	Rect *boundings;
	boundings = new Rect[10];

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

    Mat result;

	capture.open(argv[1]);

	if (!capture.isOpened()) //若打开失败
	{
		printf("fail in opening the video file.\n");
	}
	else //若打开成功
	{
		printf("Succeed in opening the video file.\n");
	}

	IplImage *fit_bk;
	fit_bk=cvLoadImage("fit.png",CV_LOAD_IMAGE_UNCHANGED);
	IplImage *fit_bk1;
    fit_bk1=cvLoadImage("fit.png",CV_LOAD_IMAGE_UNCHANGED);


	frameNum = capture.get(CV_CAP_PROP_FRAME_COUNT);
	printf("Total frame number: The video file contains %f frames.\n", frameNum);
	width = capture.get(CV_CAP_PROP_FRAME_WIDTH);
	printf("Frame width: %f pixels.\n", width);
	height = capture.get(CV_CAP_PROP_FRAME_HEIGHT);
	printf("Frame height:%f pixels.\n", height);

	width = (int)width / 2;
	height = (int)height / 2;
	imageLen = width * height;

	Mat frMat(height, width, CV_8UC1, Scalar(0));//前景矩阵

	Mat bkMat;
	capture >> bkMat;
	resize(bkMat, bkMat, Size(width, height), INTER_NEAREST);
	cB = new code_book[imageLen]; //得到与图像像素数目长度一样的一组永久背景码本
	cBB = new code_book[imageLen]; //得到与图像像素数目长度一样的一组缓存码本
	
	int i = 1;
	for(int c=0;c<imageLen;c++)
	{
		cB[c].cb=NULL;
		cB[c].numEle=0;
		cBB[c].cb=NULL;
		cBB[c].numEle=0;
	}

	former_image=cvCreateImage(Size(width,height),8,3);
	Movingobject*Mov_head=NULL;


	while (1)
	{
        position = capture.get(CV_CAP_PROP_POS_FRAMES);//position=[1,67698]
		i++;
		printf("%d\n", i);
		capture >> srcMat;
		if(position==1) {
			IplImage img = IplImage(srcMat);
			former_image = curr_image = &img;
		}
		resize(srcMat, srcMat, Size(width, height), INTER_AREA);
		srcMat.copyTo(result);
		if (i % jumpframe == 0)
		{
			GaussianBlur(srcMat, srcMat, Size(3, 3), 0, 0); //对 bkMat 高斯滤波

			if (i > 100 && i % 40 == 0)
			{
				for (int c = 0; c < imageLen; c++)
				{
					clearStaleEntries(cB[c], cBB[c], i);
				}
			}

			if (i > 100)
			{
				bkPtr = srcMat.data;
				frPtr = frMat.data;
				for (int c = 0; c < imageLen; c++)
				{
					*frPtr = backgroundDiff(bkPtr, cB[c], cBB[c], i);
					frPtr += 1;
					bkPtr += 3;
				}
			}

			if (i > 60 && i < 100)
			{
				bkPtr = srcMat.data;
				for (int c = 0; c < imageLen; c++)
				{
					updateCodeBook(bkPtr, cB[c], i); //对每个像素,调用此函数,捕捉背景中相关变化图像
					bkPtr += 3;
				}
			}

			dilate(frMat, frMat, cv::Mat(), cv::Point(-1, -1), 1);//形态学滤波：膨胀
			erode(frMat, frMat, cv::Mat(), cv::Point(-1, -1), 3);//形态学滤波：腐蚀
			dilate(frMat, frMat, cv::Mat(), cv::Point(-1, -1), 5);//形态学滤波：膨胀

			ConnectedComponents(frMat, 0, CC_SCALE, numbersd, 1, boundings, Point(-1, -1));

 //跟踪代码起始位置

         IplImage img = IplImage(srcMat);
		 curr_image=&img;

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
				 CvPoint prediction_pos=CvPoint(track_kalman_predict(temp_c));  //卡尔曼预测得到坐标点
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
	putText(result,buffer,cvPoint(5,325),CV_FONT_HERSHEY_COMPLEX,0.8,cvScalar(20,20,240),2,8);
	printf("The number of people inside is :%d",in_num);
	putText(result,buffer,cvPoint(5,340),CV_FONT_HERSHEY_COMPLEX,0.8,cvScalar(20,20,240),2,8);
	printf("The number of people outside is :%d",out_num);
	putText(result,buffer,cvPoint(5,355),CV_FONT_HERSHEY_COMPLEX,0.8,cvScalar(20,20,240),2,8);
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




		}
		
		for (int i = 0; i < numbersd; i++)
		{
			Rect r = boundings[i];
			rectangle(srcMat, r.tl(), r.br(), Scalar(0, 255, 0), 2);
			moveWindow("srcMat", 0, 0);
			waitKey(5);
		}

		imshow("srcMat", srcMat);
		moveWindow("srcMat", 0, 0);

		imshow("frMat", frMat);
		moveWindow("frMat", 0, 220);

		if (waitKey(1) >= 0)
		{
			break;
		}
	}


	cvDestroyAllWindows();
	delete[] cB;

	return 0;
}

