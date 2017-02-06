#include "codebook.h"

int   R;
int   G;
int   B;
int   Y;
int   U;
int   V;
int   Ylow;
int   Yhigh;
int   colordist;

void updateCodeBook(uchar *p, code_book &cB_c, int t)
{
	B = *p;
	G = *(p + 1);
	R = *(p + 2);
	Y = 0.114 * B + 0.587 * G + 0.299 * R;
	U = 0.500 * B - 0.3313 * G - 0.1687 * R + 127;
	V = -0.0813 * B - 0.1487 * G + 0.500 * R + 127;

	int i;
	for (i = 0; i < cB_c.numEle; i++)
	{
		Ylow = alpha * cB_c.cb[i]->YMin - 1;
		Yhigh = beta * cB_c.cb[i]->YMax + 1;
		if (Yhigh > (cB_c.cb[i]->YMax / alpha + 1))
			Yhigh = cB_c.cb[i]->YMax / alpha + 1;
		colordist = (U - cB_c.cb[i]->UAve) * (U - cB_c.cb[i]->UAve) + (V - cB_c.cb[i]->VAve) * (V - cB_c.cb[i]->VAve);

		if (Y >= Ylow && Y <= Yhigh && colordist <= Epsilon_1)
		{
			if (Y < cB_c.cb[i]->YMin)
				cB_c.cb[i]->YMin = Y;
			if (Y > cB_c.cb[i]->YMax)
				cB_c.cb[i]->YMax = Y;
			cB_c.cb[i]->UAve = (cB_c.cb[i]->fnum * cB_c.cb[i]->UAve + U) / (cB_c.cb[i]->fnum + 1);
			cB_c.cb[i]->VAve = (cB_c.cb[i]->fnum * cB_c.cb[i]->VAve + V) / (cB_c.cb[i]->fnum + 1);
			cB_c.cb[i]->fnum++;
			if ((t - cB_c.cb[i]->last) > cB_c.cb[i]->lambda)
				cB_c.cb[i]->lambda = t - cB_c.cb[i]->last;
			cB_c.cb[i]->last = t;
			break;
		}
	}

	if (i == cB_c.numEle)// p���ز�������뱾���κ�һ����Ԫ
	{
		code_element **foo = new code_element*[cB_c.numEle + 1];
		for (int ii = 0; ii < cB_c.numEle; ii++)
			foo[ii] = cB_c.cb[ii];

		foo[cB_c.numEle] = new code_element;
		if (cB_c.numEle)
			delete[] cB_c.cb;
		cB_c.cb = foo;

		cB_c.cb[cB_c.numEle]->YMin = Y;
		cB_c.cb[cB_c.numEle]->YMax = Y;
		cB_c.cb[cB_c.numEle]->UAve = U;
		cB_c.cb[cB_c.numEle]->VAve = V;
		cB_c.cb[cB_c.numEle]->fnum = 1;
		cB_c.cb[cB_c.numEle]->lambda = 0;
		cB_c.cb[cB_c.numEle]->enable = true;
		cB_c.cb[cB_c.numEle]->first = t;
		cB_c.cb[cB_c.numEle]->last = t;
		cB_c.numEle++;
	}
}

uchar backgroundDiff(uchar *p, code_book &cB_c, code_book &cBB_c, int t)
{
	B = *p;
	G = *(p + 1);
	R = *(p + 2);
	Y = 0.114 * B + 0.587 * G + 0.299 * R;
	U = 0.500 * B - 0.3313 * G - 0.1687 * R + 127;
	V = -0.0813 * B - 0.1487 * G + 0.500 * R + 127;

	int i;
	for (i = 0; i < cB_c.numEle; i++)
	{
		Ylow = alpha * cB_c.cb[i]->YMin - 1;
		Yhigh = beta * cB_c.cb[i]->YMax + 1;
		if (Yhigh > (cB_c.cb[i]->YMax / alpha + 1))
			Yhigh = cB_c.cb[i]->YMax / alpha + 1;
		colordist = (U - cB_c.cb[i]->UAve) * (U - cB_c.cb[i]->UAve) + (V - cB_c.cb[i]->VAve) * (V - cB_c.cb[i]->VAve);

		if (cB_c.cb[i]->lambda > cB_dropTime && cB_c.cb[i]->first > 100)
		{
			cB_c.cb[i]->enable = false;
		}
		/*else
		{
			cB_c.cb[i]->enable = true;
		}*/

		if (cB_c.cb[i]->enable == true && Y >= Ylow && Y <= Yhigh && colordist <= Epsilon_2)
		{
			if (Y < cB_c.cb[i]->YMin)
				cB_c.cb[i]->YMin = Y;
			if (Y > cB_c.cb[i]->YMax)
				cB_c.cb[i]->YMax = Y;
			cB_c.cb[i]->UAve = (cB_c.cb[i]->fnum * cB_c.cb[i]->UAve + U) / (cB_c.cb[i]->fnum + 1);
			cB_c.cb[i]->VAve = (cB_c.cb[i]->fnum * cB_c.cb[i]->VAve + V) / (cB_c.cb[i]->fnum + 1);
			cB_c.cb[i]->fnum++;
			if ((t - cB_c.cb[i]->last) > cB_c.cb[i]->lambda)
				cB_c.cb[i]->lambda = t - cB_c.cb[i]->last;
			cB_c.cb[i]->last = t;
			break;
		}
	}

	if (i == cB_c.numEle) // p���ظ�ͨ��ֵ���������ñ����뱾����һ��Ԫ
	{
		for (i = 0; i < cBB_c.numEle; i++) // �������汳���뱾�е�ÿ����Ԫ
		{
			Ylow = alpha * cBB_c.cb[i]->YMin - 1;
			Yhigh = beta * cBB_c.cb[i]->YMax + 1;
			if (Yhigh > (cBB_c.cb[i]->YMax / alpha + 1))
				Yhigh = cBB_c.cb[i]->YMax / alpha + 1;
			colordist = (U - cBB_c.cb[i]->UAve) * (U - cBB_c.cb[i]->UAve) + (V - cBB_c.cb[i]->VAve) * (V - cBB_c.cb[i]->VAve);

			if (cBB_c.cb[i]->lambda > cBB_dropTime)
			{
				cBB_c.cb[i]->enable = false;
			}
			/*else
			{
				cBB_c.cb[i]->enable = true;
			}*/

			if (cBB_c.cb[i]->enable == true && Y >= Ylow && Y <= Yhigh && colordist <= Epsilon_1)
			{
				if (Y < cBB_c.cb[i]->YMin)
					cBB_c.cb[i]->YMin = Y;
				if (Y > cBB_c.cb[i]->YMax)
					cBB_c.cb[i]->YMax = Y;
				cBB_c.cb[i]->UAve = (cBB_c.cb[i]->fnum * cBB_c.cb[i]->UAve + U) / (cBB_c.cb[i]->fnum + 1);
				cBB_c.cb[i]->VAve = (cBB_c.cb[i]->fnum * cBB_c.cb[i]->VAve + V) / (cBB_c.cb[i]->fnum + 1);
				cBB_c.cb[i]->fnum++;
				if ((t - cBB_c.cb[i]->last) > cBB_c.cb[i]->lambda)
					cBB_c.cb[i]->lambda = t - cBB_c.cb[i]->last;
				cBB_c.cb[i]->last = t;

				//�����뱾�������²�������Ԫ�أ����Ͻ�����Ϊ����Ԫ��ʹ��
				if ((cBB_c.cb[i]->last - cBB_c.cb[i]->first) > cBB_keepTime)
				{
					return(0);
				}
				/*if (cBB_c.cb[i]->enable == true)
				{
					return(0);
				}
				*/
				break;
			}
		}

		if (i == cBB_c.numEle)
		{
			code_element **foo = new code_element*[cBB_c.numEle + 1];
			for (int ii = 0; ii < cBB_c.numEle; ii++)
				foo[ii] = cBB_c.cb[ii];

			foo[cBB_c.numEle] = new code_element;
			if (cBB_c.numEle)
				delete[] cBB_c.cb;
			cBB_c.cb = foo;

			cBB_c.cb[cBB_c.numEle]->YMin = Y;
			cBB_c.cb[cBB_c.numEle]->YMax = Y;
			cBB_c.cb[cBB_c.numEle]->UAve = U;
			cBB_c.cb[cBB_c.numEle]->VAve = V;
			cBB_c.cb[cBB_c.numEle]->fnum = 1;
			cBB_c.cb[cBB_c.numEle]->enable = true;
			cBB_c.cb[cBB_c.numEle]->lambda = 0;
			cBB_c.cb[cBB_c.numEle]->first = t;
			cBB_c.cb[cBB_c.numEle]->last = t;
			cBB_c.numEle++;
		}

		return(255);
	}

	return(0);
}

void clearStaleEntries(code_book &cB_c, code_book &cBB_c, int t)
{
	int* cB_keep  = new int[cB_c.numEle]; //�����뱾�ı������
	int* cBB_keep = new int[cBB_c.numEle]; //�����뱾�ı������
	int  cB_dropCnt  = 0; //�����뱾��������Ԫ��Ŀ
	int  cBB_dropCnt = 0; //�����뱾��������Ԫ��Ŀ
	int  cB_addCnt = 0; //�����뱾���ӵ���Ԫ��Ŀ

	for (int i = 0; i < cB_c.numEle; i++) //���������뱾
	{
		if (cB_c.cb[i]->enable == false)
		{
			cB_keep[i] = 0; //����
			cB_dropCnt++;
		}
		else
		{
			cB_keep[i] = 1; //����
		}
	}

	for (int i = 0; i < cBB_c.numEle; i++) //���������뱾
	{
		if (cBB_c.cb[i]->enable == false)
		{
			cBB_keep[i] = 0; //����
			cBB_dropCnt++;
		}
		else if ((cBB_c.cb[i]->last - cBB_c.cb[i]->first) > cBB_keepTime)
		{
			cBB_keep[i] = 2; //����������뱾
			cB_addCnt++;
		}
		else
		{
			cBB_keep[i] = 1; //����
		}
	}
 
	//���������뱾
	code_element **foo = new code_element* [cB_c.numEle - cB_dropCnt + cB_addCnt];
	int k = 0;
	for (int ii = 0; ii < cB_c.numEle; ii++)
	{
		if (cB_keep[ii] == 1)
		{
			foo[k] = cB_c.cb[ii];
			k++;
		}
	}
	for (int ii = 0; ii < cBB_c.numEle; ii++)
	{
		if (cBB_keep[ii] == 2)
		{
			foo[k] = cBB_c.cb[ii];
			k++;
		}
	}

	delete[] cB_keep;
	if (cB_c.cb)
		delete[] cB_c.cb;
	cB_c.cb = foo; // ��foo ͷָ���ַ����c.cb
	cB_c.numEle = cB_c.numEle - cB_dropCnt + cB_addCnt;// ʣ�����Ԫ��ַ

	//���»����뱾
	foo = new code_element* [cBB_c.numEle - cBB_dropCnt - cB_addCnt];
	k = 0;
	for (int ii = 0; ii < cBB_c.numEle; ii++)
	{
		if (cBB_keep[ii] == 1)
		{
			foo[k] = cBB_c.cb[ii];
			k++;
		}
	}

	delete[] cBB_keep;
	if (cBB_c.cb)
		delete[] cBB_c.cb;
	cBB_c.cb = foo; // ��foo ͷָ���ַ����c.cb
	cBB_c.numEle = cBB_c.numEle - cBB_dropCnt - cB_addCnt;// ʣ�����Ԫ��ַ
}

