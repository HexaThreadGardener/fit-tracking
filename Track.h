
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <iostream>
#include <opencv2/objdetect/objdetect.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include <vector>
#include <algorithm>
#include <cstdio>
#include <opencv2/opencv.hpp>
#include <Math.h>

using namespace std;
using namespace cv;


const int winHeight=360;  
const int winWidth=640;
bool flags=true;

//�˶��켣��¼����

struct ObjectList{
	int x;
	int y;
	struct ObjectList*next;
};

//�˶�����ṹ��
struct Movingobject{
	int area;   //©�����
	int label;  //�������е�����λ��
	int index;  //�˶�����Ľ������
	Point points[2];//���ο�������Խǵ�
	int length;//�켣����
	struct ObjectList*head;  //�켣��ͷ���
	struct ObjectList*tail;
	CvKalman *kalman;//kalmanԤ�����ñ���
    CvMat* measurement;  //��Ҫ���������е��ó�ʼ���������г�ʼ��
	struct Movingobject*next;
};

//���ߺ����������˶��켣�������˶��켣ȫ��������
void draw_line(Mat image,ObjectList *t)
{
	ObjectList *t1=t;
	while (t1!=NULL)
	{
		if(t1->next!=NULL)
		{
			line(image,cvPoint(t1->x,t1->y),cvPoint(t1->next->x,t1->next->y),cvScalar(255,0,255),2,8,0);
		}
		t1=t1->next;
	}

}

void draw_rectangle(Mat image,Movingobject*head)
{
	Movingobject*t=head;
	while(t!=NULL)
	{
		if(t->length>=3&&t->area==0)
		{
			CvRect p=cvRect(t->points[0].x,t->points[0].y,abs(t->points[1].x-t->points[0].x),abs(t->points[1].y-t->points[0].y));
			rectangle(image,p,cv::Scalar(0,255,0),2);
			draw_line(image,t->head);
		}
	    if(t->length>=3&&t->area!=0)
		{
			CvRect p=cvRect(t->points[0].x,t->points[0].y,abs(t->points[1].x-t->points[0].x),abs(t->points[1].y-t->points[0].y));
			rectangle(image,p,cv::Scalar(255,0,0),2);
			draw_line(image,t->head);
		}
		t=t->next;
	}
}

//ʹ�þ��ο�B��ȫ��Χ�ھ��ο�A��
CvRect ChangeRect(CvRect A,CvRect B)
{
if(B.x<A.x)
   B.x=A.x;
if(B.x+B.width>A.x+A.width)
   B.width=A.x+A.width-B.x;
if(B.y<A.y)
   B.y=A.y;
if(B.y+B.height>A.y+A.height)
   B.height=A.y+A.height-B.y;
if(B.height*B.width>100)
return(B);
else return CvRect();
}
//�ͷ��ض��˶�����Ĺ켣����
void release_Movobj(Movingobject *link)
{
	Movingobject *p=link;
	ObjectList *t1=p->head,*t2=NULL;
	while(t1!=NULL)
	{
		t2=t1;
		t1=t1->next;
		delete t2;

	}
}

//��ʼ��kalman����
Movingobject* InitKalman(Movingobject *p)
{

const float A[] = {1,0,1,0,
               0,1,0,1,
               0,0,1,0,
               0,0,0,1};
p->kalman = cvCreateKalman( 4, 2, 0 );
p->measurement=cvCreateMat( 2, 1, CV_32FC1 );
memcpy( p->kalman->transition_matrix->data.fl, A, sizeof(A));//A
cvSetIdentity( p->kalman->measurement_matrix, cvScalarAll(1) );//H
cvSetIdentity( p->kalman->process_noise_cov, cvScalarAll(1e-5) );//Q w ��
cvSetIdentity( p->kalman->measurement_noise_cov, cvScalarAll(1e-1) );//R v
cvSetIdentity( p->kalman->error_cov_post, cvScalarAll(1));//P
CvRNG rng = cvRNG(-1); 
cvRandArr(&rng,p->kalman->state_post,CV_RAND_UNI,cvRealScalar(0),cvRealScalar(winHeight>winWidth?winWidth:winHeight));
return p;
}

//��ʼ���˶�����ṹ��
Movingobject* InitMovingobject(void)
{
	Movingobject*p=new Movingobject();
	p->area=0;
	p->head=new ObjectList();
	p->tail=p->head;
	p->index=0;
	p->label=0;
	p->length=0;
	p->next=NULL;
	p=InitKalman(p);
	return p;
}

//ɾ��ָ��λ�õĽṹ��
Movingobject* delete_Movobj(Movingobject*head,Movingobject*p,int *in_num,int *out_num,int *fit)
{
	Movingobject *t=head;
	if(head==p)
		head=head->next;
	else
	{
	while(t->next!=p)
		t=t->next;
	t->next=p->next;
	}
	//��ɾ���˶������ͬʱ�����˶������ǽ����ǳ�
	//���õ������չ켣���λ�����ж��ģ�Ҫ��켣�㹻��
	//�����������ǲ�׼��
	if(p->length>5)				//�����ȳ���13�������˶�������			//***���޸�***//
	{
		if(p->head->y-p->tail->y<0)			//���ĵ�����������180���ж�Ϊ����
		{
			(*in_num)++;
			(*fit)=0;
		}

		else
		{
		(*out_num)++;
		    (* fit)=1;
		if((*(in_num))>0)
			(*in_num)--;
		}
	}
	release_Movobj(p);
	cvReleaseKalman(&p->kalman);
	delete p;
	return head;
}

//�����˶�����ṹ��
Movingobject* add_Movobj(Movingobject*head,Movingobject*p)
{
	Movingobject*t=head;
	if(head==NULL)
		head=p;
	else
	{
		while(t->next!=NULL)
		t=t->next;
	t->next=p;
	
	}
    return head;
}
//��������㺯��,�ú��������𵽼�������
int cal_distance(int x1,int y1,int x2,int y2)
{
	return (int)sqrt((x1-x2)*(x1-x2)+(y1-y2)*(y1-y2));
}

void shorten_objlist(Movingobject*head)
{
	if(head!=NULL)
	{
		Movingobject*t=head;
		while(t!=NULL)
		{
			if(t->length>200)				//�������ĳ���
			{
              ObjectList *t1=t->head->next,*t2=NULL;
	          while(t1!=NULL)
	          {
		       t2=t1;
		       t1=t1->next;
		       delete t2;
	           }
			  t->head->next=NULL;
			  t->tail=t->head;
			}
			t=t->next;
		}
	}
}
//���������������켣���Ƚϳ����˶��������ǰ�����д�������Ϊ�����Ÿ��ʴ�
//����ͨ�����岢���Ǻܶ࣬���Բ���ѡ������ķ���
void sort_obj(Movingobject *head)
{
	Movingobject *sorted,*unsorted,*max,*upmax;
	if(head)
	{
	unsorted=head;
	sorted=NULL;
	int labelnum;
	while(unsorted->next)
	{
		max=unsorted;
		Movingobject *temp=unsorted->next;
		while(temp)
		{
			if(max->length<temp->length)
				max=temp;
			temp=temp->next;
		}
		if(unsorted==max)
			sorted=unsorted; //����ѡ�����ķ���ǰ�漴��
		else
		{
			upmax=unsorted;
			while(upmax->next!=max)
				upmax=upmax->next;
			upmax->next=max->next;
			if(sorted==NULL)
			{
				max->next=head;
				head=sorted=max;
			}
			else 
			{
				sorted->next=max;
				max->next=unsorted;
				sorted=max;
			}
		}
		unsorted=sorted->next;
	}
	//Ϊ�˶������������е�λ�����±��
	sorted=head;
	labelnum=1;
	while(sorted)
		{
			sorted->label=labelnum;
	        labelnum++;
			sorted=sorted->next;
	    } 
	}
}

//������Ұ�е�������
int cal_people_in_sight(Movingobject*head)
{
	if(head==NULL)
		return 0;
	else
	{
		int number=0;
		Movingobject*t=head;
		while(t!=NULL)
		{
			if(t->length>=5)
				number++;
			t=t->next;
		}
		return number;
	}
}

//�켣Ԥ�⺯����Ԥ��ĵ����ں����ļ���
Point track_kalman_predict(Movingobject *p)
{
	Point pro_position;
	if(p->length<=3)
		return cvPoint((p->points[0].x+p->points[1].x)/2,(p->points[0].y+p->points[1].y)/2);
	else
	{
		//kalmanԤ��
	    const CvMat* prediction=cvKalmanPredict(p->kalman,0);  
        pro_position=cvPoint((int)prediction->data.fl[0],(int)prediction->data.fl[1]); 
		int dis=cal_distance((p->points[0].x+p->points[1].x)/2,(p->points[0].y+p->points[1].y)/2,
			                   pro_position.x,pro_position.y);
		if(dis>70)
			return cvPoint((p->points[0].x+p->points[1].x)/2,(p->points[0].y+p->points[1].y)/2);
		else 
		return pro_position;
	}
}

//kalman�������������ʵ�ʵõ�������������kalman����
void track_predict_update(Movingobject *p,Point real_position)
{
       p->measurement->data.fl[0]=(float)real_position.x;  
       p->measurement->data.fl[1]=(float)real_position.y;  
	   cvKalmanCorrect( p->kalman, p->measurement ); 
	
}

//camshift׷���㷨������Ϊ��׷�ٶ���ǰһ֡ͼ��͵�ǰ��׷��ͼ�񣬷���׷�ٵõ�������
CvRect track_camshift(Movingobject*p,IplImage *former_image,IplImage *curr_image)
{
	int hdims = 16;  //��ֵԽ��׷��yue׼ȷ������ʱ��Ҳ������
	//int vmin = 10, vmax = 256, smin = 30;  
	float hranges_arr[] = {0,180};  
	float* hranges = hranges_arr; 
	IplImage *image = 0, *hsv = 0,*hsv_curr=0, *hue = 0, *mask = 0, *backproject = 0, *histimg = 0;  
    CvHistogram *hist = 0; //��ɫֱ��ͼ  
	CvRect selection=cvRect(p->points[0].x,p->points[1].y,
		                      abs(p->points[1].x-p->points[0].x),abs(p->points[1].y-p->points[0].y));
	                          //����Ȥ��������ʵ����Movingobject�еľ��ο�����
	CvRect track_window;    //camshift�ĸ��ٴ���
	CvBox2D track_box;  
	CvConnectedComp track_comp;  
	
	//��ʼ��
	image = cvCreateImage( cvGetSize(former_image), 8, 3 );  
    image->origin = former_image->origin;  
    hsv = cvCreateImage( cvGetSize(former_image), 8, 3 ); 
	hsv_curr=cvCreateImage( cvGetSize(former_image), 8, 3 ); 
    hue = cvCreateImage( cvGetSize(former_image), 8, 1 );  
    mask = cvCreateImage( cvGetSize(former_image), 8, 1 );  
    backproject = cvCreateImage( cvGetSize(former_image), 8, 1 );  
    hist = cvCreateHist( 1, &hdims, CV_HIST_ARRAY, &hranges, 1 );//����ֱ��ͼ
	cvCopy( former_image, image, 0 );  
    cvCvtColor( image, hsv, CV_BGR2HSV );//ת����hsv��ɫ�ռ�
	//�������Ԫ���Ƿ���������֮��  
    cvInRangeS( hsv, cvScalar(0,30,MIN(10,256),0),cvScalar(180,256,MAX(10,256),0), mask ); 
	cvSplit( hsv, hue, 0, 0, 0 ); //����hsvͼ�����ͨ��
	//ѡ������Ȥ���������histֱ��ͼ
	float max_val = 0.f; 
	cvSetImageROI( hue, selection );  
    cvSetImageROI( mask, selection );  
    cvCalcHist( &hue, hist, 0, mask );  
    cvGetMinMaxHistValue( hist, 0, &max_val, 0, 0 );//��ȡֱ��ͼ�������ֵ  
    cvConvertScale( hist->bins, hist->bins, max_val ? 255. / max_val : 0., 0 );//ʹ�����Ա任ת�����飬ֱ��ͼ��һ��   
    cvResetImageROI( hue ); //�ͷ�ͼ��image�б��趨�ĸ���Ȥ����ROI����cvSetImageROI���Ӧ�� 
    cvResetImageROI( mask );  
    track_window = selection; 
	//������ͼ��ķ���ͶӰͼ
	cvCvtColor( curr_image, hsv_curr, CV_BGR2HSV );//ת����hsv��ɫ�ռ�
	cvInRangeS( hsv_curr, cvScalar(0,30,MIN(10,256),0),  
                        cvScalar(180,256,MAX(10,256),0), mask );
	cvSplit( hsv_curr, hue, 0, 0, 0 ); //����hsvͼ�����ͨ��  

	cvCalcBackProject( &hue, backproject, hist );  
    cvAnd( backproject, mask, backproject, 0 );//��λ��  
    //camshift�㷨Ӧ��  
    cvCamShift( backproject, track_window,  
                    cvTermCriteria( CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 10, 1 ),  
                         &track_comp, &track_box );  
    track_window = track_comp.rect; 
	//track_window=ChangeRect(cvRect(0,0,winWidth,winHeight),track_window);
	return track_window;      //����track�Ľ��     
}

//ƽ��ͼʵʱ��ʾ����
void Fit_bk (IplImage *bk, int flag)
{       
	    CvFont font;
		cvInitFont(&font,CV_FONT_HERSHEY_COMPLEX_SMALL,1,1,(0,0),2,8);   
	if(flag==0)
	{
		cvLine(bk,cvPoint(695,257),cvPoint(752,257),cvScalar(0,0,255),2,8,0);//��ͷ����
		cvLine(bk,cvPoint(695,257),cvPoint(700,262),cvScalar(0,0,255),2,8,0);
		cvLine(bk,cvPoint(695,257),cvPoint(700,252),cvScalar(0,0,255),2,8,0);
		cvCircle(bk,cvPoint(730,190),5,cvScalar(0,0,255),2);
		cvRectangle(bk,cvPoint(723,195),cvPoint(737,210),cvScalar(0,0,255),2);//����
		cvRectangle(bk,cvPoint(723,210),cvPoint(727,220),cvScalar(0,0,255),2);//����
		cvRectangle(bk,cvPoint(734,210),cvPoint(737,220),cvScalar(0,0,255),2);//����
		cvRectangle(bk,cvPoint(717,195),cvPoint(723,198),cvScalar(0,0,255),2);//���
		cvRectangle(bk,cvPoint(737,195),cvPoint(741,198),cvScalar(0,0,255),2);//�ұ�
	    cvPutText(bk,"in",cvPoint(695,350),&font,CV_RGB(0,0,0));

	}
	if(flag==1)
	{
		cvLine(bk,cvPoint(695,257),cvPoint(752,257),cvScalar(0,0,255));
		cvLine(bk,cvPoint(732,237),cvPoint(752,257),cvScalar(0,0,255));
		cvLine(bk,cvPoint(732,277),cvPoint(752,257),cvScalar(0,0,255));
		cvCircle(bk,cvPoint(730,190),15,cvScalar(0,0,255),2);
		cvRectangle(bk,cvPoint(710,205),cvPoint(750,235),cvScalar(0,0,255),2);//����
		cvRectangle(bk,cvPoint(710,235),cvPoint(720,255),cvScalar(0,0,255),2);//����
		cvRectangle(bk,cvPoint(740,235),cvPoint(750,255),cvScalar(0,0,255),2);//����
		cvRectangle(bk,cvPoint(700,205),cvPoint(710,210),cvScalar(0,0,255),2);//���
		cvRectangle(bk,cvPoint(750,205),cvPoint(760,210),cvScalar(0,0,255),2);//�ұ� 
	    cvPutText(bk,"out",cvPoint(695,350),&font,CV_RGB(0,0,0));

	}

}