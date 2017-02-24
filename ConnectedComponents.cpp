//
// Created by Peter Lu on 2/24/17.
//

#include <cv.h>
#include "ConnectedComponents.h"

void ConnectedComponents(Mat &mask_process, int poly1_hull0, float scale, int &p, int number,
	Rect * &bounding_box, const Point &contour_centers)
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

