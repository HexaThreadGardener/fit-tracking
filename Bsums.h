#include "opencv2/highgui/highgui.hpp"  
#include "opencv2/imgproc/imgproc.hpp"  
#include "opencv2/core/core.hpp"
#include <opencv/ml.h>
#include <iostream>
#include "cv.h"
#include "highgui.h"
#include <vector>
#include <math.h>
#include <string.h>
#include <fstream>
using namespace std;
using namespace cv;


long long bSums(Mat src)
{
	
	long long  counter = 0;
	//�������������ص�
	Mat_<uchar>::iterator it = src.begin<uchar>();
	Mat_<uchar>::iterator itend = src.end<uchar>();  
	for (; it!=itend; ++it)
	{
		if((*it)>30) counter+=1;//��ֵ�������ص���0����255
	}			
	return counter;
}