
/****************************************************************************/
/*                        北京博维恒信公司                                  */
/*                           MyClass.h                                      */
/*                        编写人：李瑞                                      */
/*                        日期：  二○○五年三月十五日                      */
/*                        用途：  描述三维点、向量、标记（四维）点、        */ 
/*                                颜色、二维纹理坐标  操作加减乘除叉积点积  */
/*                                矩阵计算（单 加 乘 逆 转 特征值 特征向量）*/                                                    
/*                                和变换矩阵（平移、绕三个轴旋转、          */
/*                                点绕任意轴旋转、矢量旋转到Z轴平行方向）   */
/****************************************************************************/
#ifndef   __MYCLASS_H__
#define   __MYCLASS_H__



////////////////////////////////////////////////////////////////////////////
//opengl 头文件
#include <gl\gl.h>//gl
#include <gl\glu.h>//glu
//#include <gl\glut.h>//glut

//////////////////////////////////////////////////////////////////////////
//STL  C++ template class  类模板
#include <math.h>
#include <vector>
#include <list>
#include <cstring>
#include <algorithm>

//#include <gl\glaux.h>//glaux
//////////////////////////////////////////////////////////////////////////
//opengl 库 library
#pragma comment(lib,"Opengl32.lib")//
#pragma comment(lib,"Glu32.lib")//
//#pragma comment(lib,"Glut32.lib")//


/////////////////////////////////////////////////////////////////////////////////
//namespace
using namespace std;

////////////////////////////////////////////////////////////////////////////////
//const parameter 常量参数
//const double PI=3.141592653589793;


//////////////////////////////////////////////////////////////////////////////////
//2d point  function  二维点距
double __declspec(dllexport) Distance(const CPoint& p0,const CPoint& p1);

/////////////////////////////////////////////////////////////////////////////////////
//3d point normal 4d
//////////////////////////////////////////////////////////////////////////
//class tagCVector
class __declspec(dllexport) tagVertex;//4d point normal 四维点
class __declspec(dllexport) tagCVector//3d point normal三维点、向量  AFX_EXT_CLASS
{
   public:
//////////////////////////////////////////////////////////////////////////
    tagCVector();
    tagCVector(const tagCVector& V);
    tagCVector(const tagVertex& V);
    tagCVector(const float& X,const float& Y,const float& Z);
    ~tagCVector();
   //////////////////////////////////////////////////////////////////////////
   //tagCVector  operator overloading  = + - * / ^  ||
	tagCVector  operator=(const tagCVector& V);
	tagCVector  operator=(const tagVertex& V);
	tagCVector  operator+(const tagCVector& V);
	tagCVector  operator-(const tagCVector& V);
	tagCVector  operator^(const tagCVector& V);//叉乘
	tagCVector  operator+=(const tagCVector& V);
	tagCVector  operator-=(const tagCVector& V);
	tagCVector  operator^=(const tagCVector& V);//叉乘
	float       operator*(const tagCVector& V);
	tagCVector  operator *(int u);
	tagCVector  operator*(float   u); 
	tagCVector  operator*(double   u);
	tagCVector  operator/(double   u); 
	tagCVector  operator/(float   u); 
	tagCVector  operator/(int   u); 
	tagCVector  operator *=(int u);
	tagCVector  operator*=(float   u); 
	tagCVector  operator*=(double   u);
	tagCVector  operator/=(double   u); 
	tagCVector  operator/=(float   u); 
	tagCVector  operator/=(int   u);
	void        normalize();
/////////////////////////////////////////////////////////////////////////*/
//     attributte 
   public:
       float  x;
       float  y;
       float  z;    
};
typedef  tagCVector  CNormal,CVector;//define normal vector
typedef  tagCVector  *CNormal3D,*CVector3D;//define vector pointer
//////////////////////////////////////////////////////////////////////////
// Extern operator function 
tagCVector __declspec(dllexport) operator*(const tagCVector& V,const float  Matrix[4][4]);//left hand 
tagCVector __declspec(dllexport) operator*(const tagCVector& V,const double Matrix[4][4]);//left hand 
tagCVector __declspec(dllexport) operator+(const tagCVector& V,const tagCVector& W);
tagCVector __declspec(dllexport) operator-(const tagCVector& V,const tagCVector& W);
float      __declspec(dllexport) operator*(const tagCVector& V,const tagCVector& W);
tagCVector __declspec(dllexport) operator^(const tagCVector& V,const tagCVector& W);//叉乘
tagCVector __declspec(dllexport) operator*(const int& u,const tagCVector& W);
tagCVector __declspec(dllexport) operator*(const float& u,const tagCVector& W);
tagCVector __declspec(dllexport) operator*(const double& u,const tagCVector& W);
bool       __declspec(dllexport) operator==(const tagCVector& V , const tagCVector& W);
double     __declspec(dllexport) Distance(const tagCVector& V,const tagCVector& W);
tagCVector __declspec(dllexport) CVector3DCNormal(const tagCVector& U,const tagCVector& V,const tagCVector& W);
//         三个三维点   return  (V-U)^(W-V)   单位化向量
bool       __declspec(dllexport) CPoint2DTo3D(const CPoint& point,const GLdouble modelMatrix[16],const GLdouble projMatrix[16],const GLint viewport[4],tagCVector* v);
//         point－二维点   modelMatrix － 模型矩阵   projMatrix － 投影矩阵   viewport － 视图矩阵     v － 返回三维点值
//         如果二维转化三维失败返回 false  否则 true
bool       __declspec(dllexport) CVector3DTo2D(const tagCVector& v,const GLdouble modelMatrix[16],const GLdouble projMatrix[16],const GLint viewport[4],CPoint* point);
//         v － 三维点   modelMatrix － 模型矩阵   projMatrix － 投影矩阵   viewport － 视图矩阵     point－返回二维点
//         如果三维转化二维失败返回 false  否则 true
bool       __declspec(dllexport) GetBoxSize(const tagCVector& BoxCenter1,const tagCVector& BoxSize1,const tagCVector& BoxCenter2,const tagCVector& BoxSize2,tagCVector& BoxCenter3,tagCVector& BoxSize3);
//         包围盒求交    BoxCenter1  BoxSize1－ 第一个包围盒    BoxCenter2  BoxSize2－ 第二个包围盒  BoxCenter3  BoxSize3－ 返回第三个包围盒
//         如果包围盒一和二相交返回 true  否则 false
//////////////////////////////////////////////////////////////////////////
//class  tagVertex 四维点  AFX_EXT_CLASS
class  __declspec(dllexport) tagVertex : public tagCVector
{
public:
    tagVertex();
    tagVertex(const tagCVector& V);
    tagVertex(const tagVertex& V);
    tagVertex(const float& X,const float& Y,const float& Z);
    tagVertex(const float& X,const float& Y,const float& Z,const BYTE& W);
    ~tagVertex();
//////////////////////////////////////////////////////////////////////////
    tagVertex  operator=(const tagVertex& V);
    tagVertex  operator=(const tagCVector& V);
public:
     BYTE  w;   /*** 标示符 ***********************/
	            /*<10	 -useless          ********/
                /*>9     -useful	       ********/
	            /*=10    -show             ********/
				/*=11,12 -select           ********/
				/*=13,14 -hide             ********/
				/*=?     -volume compute   ********/ 
				/*=?     -cutting data     ********/
};


//typedef  vector<tagVertex>  CVertexList;//
//////////////////////////////////////////////////////////////////////////
// function 
double     __declspec(dllexport)  Distance(const tagVertex& V,const tagVertex& W);
tagCVector __declspec(dllexport) CVector4DCNormal(const tagVertex& U,const tagVertex& V,const tagVertex& W);
//         三个四维点   return  (V-U)^(W-V)   三维单位化向量
bool       __declspec(dllexport) CPoint2DTo4D(const CPoint& point,const GLdouble modelMatrix[16],const GLdouble projMatrix[16],const GLint viewport[4],tagVertex* v);
//         point－二维点   modelMatrix － 模型矩阵   projMatrix － 投影矩阵   viewport － 视图矩阵     v － 返回四维点值
//         如果二维转化四维失败返回 false  否则 true
bool       __declspec(dllexport) CVector4DTo2D(const tagVertex& v,const GLdouble modelMatrix[16],const GLdouble projMatrix[16],const GLint viewport[4],CPoint* point);
//         v － 四维点   modelMatrix － 模型矩阵   projMatrix － 投影矩阵   viewport － 视图矩阵     point－返回二维点
//         如果四维转化二维失败返回 false  否则 true
//////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////
//struct tagColor 颜色 AFX_EXT_CLASS
class  __declspec(dllexport) tagColor
{
public:
   tagColor();
   tagColor(const tagColor& pColor);
   tagColor(const GLubyte& r,const GLubyte& g,const GLubyte& b);
   ~tagColor();
   tagColor  operator+(const tagColor& C);
   tagColor  operator-(const tagColor& C);
   tagColor  operator*(int  u);
   tagColor  operator/(int  u);

   GLubyte r;
   GLubyte g;
   GLubyte b;
};
typedef tagColor CColor,*CColor3D;//define class  and  pointer
//typedef  vector<tagColor>  CColorList;//
//////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////
//struct  tagTexture 二维纹理 AFX_EXT_CLASS
class  __declspec(dllexport) tagTexture
{
public:
   tagTexture();
   tagTexture(const tagTexture& T);
   tagTexture(const float& S,const float& T);
   ~tagTexture();

   tagTexture  operator+(const tagTexture& T);
   tagTexture  operator-(const tagTexture& T);
   tagTexture  operator*(int  u);
   tagTexture  operator/(int  u);

   float  s;
   float  t;
};
typedef tagTexture CTexture,*CTexture2D;//define class  and  pointer
//typedef  vector<tagTexture>  CTextureList;//
//////////////////////////////////////////////////////////////////////////


/************************************************************************/
/*          matrix  function  define all                                */
/************************************************************************/
//matrix = I 方阵单位化
template<class T>
void __declspec(dllexport)  MatrixIdentity(T* matrix,int nrows)
{ //  matrix -  nrows*nrows
   memset(matrix,0,nrows*nrows*sizeof(T));
   for(int i=0;i<nrows;++i)
     matrix[i*nrows+i]=1;
}

//matrix[3][4] change to  matrix0[4][3] 矩阵转置
template<class T,class T0>
void __declspec(dllexport) MatrixTranspose(T* matrix,T0* matrix0,int nrows,int ncols)
{// matrix - nrows*ncols    matrix0 - ncols*nrows
   for(int i=0;i<nrows;++i)
     for(int j=0;j<ncols;++j)
        matrix0[j*nrows+i]=matrix[i*ncols+j];
}

//matrix  M1=M  (double , float) =  (float , double)  矩阵相等
template<class TT1,class TT2>//class TT3>
void  __declspec(dllexport) MatrixEqual(TT1* M,TT2* M1,int nrows,int ncols)
{//M - nrows*ncols    M1 - nrows*ncols
   for(int i=0;i<nrows;++i)
    for(int j=0;j<ncols;++j)
      M1[i*ncols+j]=M[i*ncols+j];     
}

//matrix  M2=M+M1   (double , float) + (double , float) = (double , float) 矩阵相加
template<class TT1,class TT2,class TT3>
void  __declspec(dllexport) MatrixAdd(TT1* M,TT2* M1,TT3* M2,int nrows,int ncols)
{//M - nrows*ncols    M1 - nrows*ncols   M2 - nrows*ncols
   for(int i=0;i<nrows;++i)
    for(int j=0;j<ncols;++j)
       M2[i*ncols+j]=M[i*ncols+j]+M1[i*ncols+j];   
}

// (float,double) * (float,double) =  (double , float) 矩阵相乘
template<class TT1,class TT2,class TT3>
void  __declspec(dllexport) MultMatrix(TT1* M,TT2* M1,TT3* M2,int rows,int cols)
{//M - rows*cols    M1 - cols*rows   M2 - rows*rows
  for(int i=0;i<rows;++i)
    for(int j=0;j<rows;++j)
   {
       M2[i*rows+j]=0.0;
       for(int k=0;k<cols;++k)
         M2[i*rows+j]+=M[i*cols+k]*M1[k*rows+j];
   }  
}

// (float,double) * (float,double) =  (double , float) 矩阵相乘
template<class TT1,class TT2,class TT3>
void  __declspec(dllexport) MultMatrix(TT1* M,TT2* M1,TT3* M2,int rows,int soms,int cols)
{
//M - rows*soms    M1 - soms*cols   M2 - rows*cols
 for(int i=0;i<rows;++i)
    for(int j=0;j<cols;++j)
   {
       M2[i*cols+j]=0.0;
       for(int k=0;k<soms;++k)
         M2[i*cols+j]+=M[i*soms+k]*M1[k*cols+j];
   } 
}

///Gauss max elements solve equations 高斯列主元消元法求解多元一次线性方程组
template<class TT1,class TT2,class TT3>//>
void __declspec(dllexport)  Gauss(TT1* Aa,TT2* Bb,TT3* x,int rows)
{// Aa * x = Bb    root - x[rows]
   int i,j,k;
   int ik;
   double temp,mik;
   double sum,aa;
   double* a=new double[rows*rows];
   double* b=new double[rows];   
   
   for(i=0;i<rows;++i)
   {
     b[i]=Bb[i]; 
	 for(j=0;j<rows;++j)
	   a[i*rows+j]=Aa[i*rows+j];	 
	}
   for(k=0;k<rows-1;++k)
   {
	   ik=k;
	   aa=a[k*rows+k];
	   for(i=k;i<rows;++i)
       {		   
         if(fabs(aa)<fabs(a[i*rows+k]))
		 { 
		   aa=a[i*rows+k];
		   ik=i;			   
		 }
	   }
	   if(ik!=k)
       {
		 for(j=k;j<rows;++j)
		 {
		   temp=a[k*rows+j];
		   a[k*rows+j]=a[ik*rows+j];
		   a[ik*rows+j]=temp;
		 }
	 	temp=b[k];
	 	b[k]=b[ik];
	 	b[ik]=temp;
	   }
       for(i=k+1;i<rows;++i)
	   {
	     mik=a[i*rows+k]/a[k*rows+k];
	     for(j=k+1;j<rows;j++)
	 	   a[i*rows+j]=a[i*rows+j]-mik*a[k*rows+j];
	     b[i]=b[i]-mik*b[k];
	   }
	}
    x[rows-1]=b[rows-1]/a[rows*rows-1];
	for(k=rows-2;k>-1;--k)
	{
	  sum=0.0;
	  for(j=k+1;j<rows;++j)
		  sum+=a[k*rows+j]*x[j];
	  x[k]=(b[k]-sum)/a[k*rows+k];
	}

    delete[]a;
    delete[]b;
    a=NULL;
    b=NULL;	  
}
//////////////////////////////////////////////////////////////////////////

//Doolittle max elements solve equations 
//Doolittle列主元消元法求解多个多元一次线性方程组  求解矩阵 
template<class TT1,class TT2,class TT3>//>
void __declspec(dllexport)  Doolittle(TT1* aa,TT2* bb,TT3* xx,int rows)
{// aa * xx = bb        root - xx[rows][rows]
   int k,i,j,t,ik;
   int* M=new int[rows];
   double  *s,*l,*u,*a,*b;
   double temp,smax=0,*y,*x;
   s=new double[rows];
   l=new double[rows*rows];
   u=new double[rows*rows];
   a=new double[rows*rows];
   b=new double[rows];
   y=new double[rows];
   x=new double[rows];
//  QA  =  LU
   for(i=0;i<rows;++i)
   {
     M[i]=0;
     for(j=0;j<rows;++j)
	 {
       a[i*rows+j]=aa[i*rows+j];
	 }
	}
   for(k=0;k<rows;++k)
   {
     for(i=k;i<rows;++i)
	 {
	   s[i]=a[i*rows+k];
	   for(t=0;t<k;++t)
		   s[i]-=l[i*rows+t]*u[t*rows+k];
	   if(i==k)
		{
		     smax=s[i];
			 ik=i;
		 }
       if(fabs(smax)<fabs(s[i]))
		{
		     smax=s[i];
			 ik=i;
		}
	 }
	 M[k]=ik;
	 if(ik!=k)
	 {
       for(t=0;t<k;++t)
		{
           temp=l[k*rows+t];
		   l[k*rows+t]=l[ik*rows+t];
		   l[ik*rows+t]=temp;
		}
       for(t=k;t<rows;++t)
		{
           temp=a[k*rows+t];
		   a[k*rows+t]=a[ik*rows+t];
		   a[ik*rows+t]=temp;
		}
       temp=s[k];
	   s[k]=s[ik];
	   s[ik]=temp;
	 }
     u[k*rows+k]=s[k];
	 if(k<rows-1)
	 {
	    for(j=k+1;j<rows;++j)
		{
		  u[k*rows+j]=a[k*rows+j];
		  for(t=0;t<k;++t)
			  u[k*rows+j]-=l[k*rows+t]*u[t*rows+j];
		}
		for(i=k+1;i<rows;++i)
		  l[i*rows+k]=s[i]/u[k*rows+k];
	 }
   }
   //Qb  =  Ly   AND   Ux  =   y
   for(j=0;j<rows;++j)
   {
      for(i=0;i<rows;++i)
        b[i]=bb[i*rows+j];
      for(k=0;k<rows-1;++k)
      {
         t=M[k];
	     temp=b[k];
	     b[k]=b[t];
	     b[t]=temp;
      }
      y[0]=b[0];
      for(i=1;i<rows;++i)
      {
       y[i]=b[i];
       for(t=0;t<i;++t)
         y[i]-=l[i*rows+t]*y[t];     
       }
      x[rows-1]=y[rows-1]/u[rows*rows-1];
      for(i=rows-2;i>-1;--i)
      {
        x[i]=y[i];
        for(t=i+1;t<rows;++t)
           x[i]-=u[i*rows+t]*x[t];
        x[i]/=u[i*rows+i];
      } 
      for(i=0;i<rows;++i)
      {
        xx[i*rows+j]=x[i];
      }           
   }
   delete[]M;
   delete[]s;
   delete[]l;
   delete[]u;
   delete[]a;
   delete[]b;
   delete[]y;
   delete[]x;
   M=NULL;
   s=NULL;
   l=NULL;
   u=NULL;
   a=NULL;
   b=NULL;
   y=NULL;
   x=NULL;
}
//////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////
// invertible matrice  矩阵求逆
template<class TT1,class TT2>//,class TT3>
void __declspec(dllexport)  MatrixAnti(TT1* Matrix,TT2* MatrixA,int rows)
{//  Matrix * MatrixA = I          I = E
   double* E=new double[rows*rows]; 
   MatrixIdentity(E,rows); 
/*   memset(E,0,rows*rows*sizeof(double)); 
   int i;
   for(i=0;i<rows;++i)
	   E[i*rows+i]=1.0;*/
/*//Gauss solution
 TT2* b=new TT2[rows];
 TT2* x=new TT2[rows];
 memset(b,0.0,rows*sizeof(TT2));
 memset(x,0.0,rows*sizeof(TT2));
 for(int j=0;j<rows;j++)
	{
       for(i=0;i<rows;i++)
		   b[i]=E[i*rows+j];
	   Gauss(Matrix,b,x,rows);       
	   for(i=0;i<rows;i++)
		   MatrixA[i*rows+j]=x[i];
	}
    delete[]b;
    delete[]x;
    b=NULL;
    x=NULL;*/
//Doolittle solution
   Doolittle(Matrix,E,MatrixA,rows); 
   delete[]E;
   E=NULL;
}

//////////////////////////////////////////////////////////////////////////
///Jacobi Solve Eigen of  Matrix 求解矩阵特征值及特征向量
template<class TT1,class TT2>
void __declspec(dllexport) EigenJacobi(TT1 *M1,TT2 *M2,int rows)
{//M1 - rows * rows   M2 - rows * rows 
  int i,j,p,q;
  double t,z,cos2w,sin2w,cosw,sinw;
  double *Upq,*D,maxa,*UTpq;
  Upq=new double[rows*rows];
  UTpq=new double[rows*rows];
  D=new double[rows*rows];
  MatrixIdentity(M2,rows); 
 /* memset(M2,0,rows*rows*sizeof(double));
  for(i=0;i<rows;++i)
    M2[i*rows+i]=1.0; */
  maxa=M1[0*rows+1];
  p=0;
  q=1;
  for(i=0;i<rows;++i)
   for(j=i+1;j<rows;++j)
     if(fabs(M1[i*rows+j])>fabs(maxa))
     {
       maxa=M1[i*rows+j];
       p=i;
       q=j;
     }
//
  do 
  {
   MatrixIdentity(Upq,rows); 
/*   memset(Upq,0,rows*rows*sizeof(double));
   for(i=0;i<rows;++i)
     Upq[i*rows+i]=1.0; */
   t=2.0*M1[p*rows+q]/(M1[p*rows+p]-M1[q*rows+q]);
   z=(M1[p*rows+p]-M1[q*rows+q])/(2.0*M1[p*rows+q]);
   if(fabs(t)<1.0)
   {
     cos2w=1.0/sqrt(1.0+t*t);
     sin2w=t/sqrt(1.0+t*t);
   }
   else
   {
     cos2w=fabs(z)/sqrt(1.0+z*z);
     sin2w=z/(fabs(z)*sqrt(1.0+z*z));
   }
   cosw=sqrt(0.5*(1+cos2w));
   sinw=0.5*sin2w/cosw;
   Upq[p*rows+p]=cosw;
   Upq[q*rows+q]=cosw;
   Upq[p*rows+q]=-sinw;
   Upq[q*rows+p]=sinw;
//
   MatrixTranspose(Upq,UTpq,rows,rows);
   MultMatrix(UTpq,M1,D,rows,rows);
   MultMatrix(D,Upq,M1,rows,rows); 
   MultMatrix(M2,Upq,D,rows,rows);
   MatrixEqual(D,M2,rows,rows);  
//             
   maxa=M1[0*rows+1];
   p=0;
   q=1;
   for(i=0;i<rows;++i)
   for(j=i+1;j<rows;++j)
     if(fabs(M1[i*rows+j])>fabs(maxa))
     {
       maxa=M1[i*rows+j];
       p=i;
       q=j;
     }
  } while(fabs(maxa)>1.0e-8);
  delete []Upq;
  Upq=NULL;
  delete []UTpq;
  UTpq=NULL;
  delete []D;
  D=NULL; 
}

//EigenVector of  max EigenValue matrix  求解最小特征值 特征向量
template<class TT1,class TT2>
void __declspec(dllexport)  EigenVector(TT1 *M,TT2 *M2,int rows)
{// M - rows * rows    M2 - rows 
   TT1 max;
   int i,p;
   TT2 *M1=new TT2[rows*rows];
   EigenJacobi(M,M1,rows);
   max=M[0*rows+0];
   p=0;
   for(i=1;i<rows;++i)
     if(max>M[i*rows+i])
     {
       p=i;
       max=M[i*rows+i];
     }
   for(i=0;i<rows;++i)
     M2[i]=M1[i*rows+p];
   delete []M1;
   M1=NULL;
}

//////////////////////////////////////////////////////////////////////////
//EigenVector Transform matrix  Best align   points pairs numbers more than four
//四元素描述 旋转矩阵
template<class TT1,class TT2>
void __declspec(dllexport) EigenMatrix(const tagCVector V[],TT1 *ME, TT2 *Matrix, int rows)
{//V  -  2     ME -  4     Matrix  - 16
   MatrixIdentity(Matrix,rows);
   Matrix[0]=ME[0]*ME[0]+ME[1]*ME[1]-ME[2]*ME[2]-ME[3]*ME[3];
   Matrix[5]=ME[0]*ME[0]-ME[1]*ME[1]+ME[2]*ME[2]-ME[3]*ME[3];
   Matrix[10]=ME[0]*ME[0]-ME[1]*ME[1]-ME[2]*ME[2]+ME[3]*ME[3];
 
   Matrix[1]=2.0*(ME[1]*ME[2]+ME[0]*ME[3]);//by rows   lefthand
   Matrix[4]=2.0*(ME[1]*ME[2]-ME[0]*ME[3]);
   Matrix[2]=2.0*(ME[1]*ME[3]-ME[0]*ME[2]);
   Matrix[8]=2.0*(ME[1]*ME[3]+ME[0]*ME[2]);
   Matrix[6]=2.0*(ME[2]*ME[3]+ME[0]*ME[1]);
   Matrix[9]=2.0*(ME[2]*ME[3]-ME[0]*ME[1]);
/*
   Matrix[4]=2.0*(ME[1]*ME[2]+ME[0]*ME[3]);
   Matrix[1]=2.0*(ME[1]*ME[2]-ME[0]*ME[3]);
   Matrix[8]=2.0*(ME[1]*ME[3]-ME[0]*ME[2]);
   Matrix[2]=2.0*(ME[1]*ME[3]+ME[0]*ME[2]);
   Matrix[9]=2.0*(ME[2]*ME[3]+ME[0]*ME[1]);
   Matrix[6]=2.0*(ME[2]*ME[3]-ME[0]*ME[1]);*/
   double  M1[4][4],M2[4][4],M3[4][4],M4[4][4];
   GetTranslateMatrix(tagCVector(0,0,0)-V[0],&M1[0][0]);
   GetTranslateMatrix(V[0]-V[1],&M2[0][0]);
   GetTranslateMatrix(V[0],&M3[0][0]);
   MultMatrix(&M2[0][0],Matrix,&M4[0][0],4,4);
   MultMatrix(&M1[0][0],&M4[0][0],&M2[0][0],4,4);
   MultMatrix(&M2[0][0],&M3[0][0],Matrix,4,4);  
}

template<class T,class T0>//旋转矩阵  绕X\Y\Z轴转
void __declspec(dllexport) GetRotateMatrix(const char& c,const T& angle,T0* Matrix)
{ //Rotate    around   axis    X   Y   Z  
    MatrixIdentity(Matrix,4);   
/*  memset(Matrix,0,16*sizeof(T0));
    for(int i=0;i<4;++i)
       Matrix[i*4+i]=1.0;*/
    float cangle=angle*PI/180.0;
    switch(c)
    {
      case 'X':
      case 'x':
        Matrix[5]=cos(cangle);
        Matrix[10]=Matrix[5];
        Matrix[6]=sin(cangle);
        Matrix[9]=-Matrix[6];
        break;
      case 'Y':
      case 'y':
        Matrix[0]=cos(cangle);
        Matrix[10]=Matrix[0];
        Matrix[2]=-sin(cangle);
        Matrix[8]=-Matrix[2];
        break;
      case 'Z':
      case 'z':
        Matrix[5]=cos(cangle);
        Matrix[0]=Matrix[5];
        Matrix[1]=sin(cangle);
        Matrix[4]=-Matrix[1];
        break;
      default:
        break;
    }
}

template<class T>//平移矩阵
void __declspec(dllexport) GetTranslateMatrix(const tagCVector& dxyz,T* Matrix)
{// dxyz  offset translate 
    MatrixIdentity(Matrix,4);   
/*    memset(Matrix,0,16*sizeof(T));
    for(int i=0;i<4;++i)
       Matrix[i*4+i]=1.0;*/
    Matrix[12]=dxyz.x;
    Matrix[13]=dxyz.y;
    Matrix[14]=dxyz.z;
}

template<class T,class T0>//绕任意轴旋转矩阵
void __declspec(dllexport) GetRotateMatrix(const T& Angle,const tagCVector& Origin,const tagCVector& Axis,T0* Matrix)
{//rotate anyaxis   Origin - origin point   Axis - axis direction
   double AngleX,AngleY,AngleZ;
   tagCVector OriginXYZ,axis=Axis;
   OriginXYZ.x=-Origin.x;
   OriginXYZ.y=-Origin.y;
   OriginXYZ.z=-Origin.z;
   axis.normalize();
   AngleZ=Angle;//*PI/180.0
   AngleY=-asin(axis.x)*180.0/PI;
   if(fabs(axis.z)>0.0)
   {
     AngleX=atan2(axis.y,axis.z)*180.0/PI;
//     if(Axis.z<0.0)
//       AngleX=AngleX+PI;
   }
   else
   {
     if(axis.y>0.0)
        AngleX=90.0;
     else if(axis.y<0.0)
        AngleX=-90.0;  
     else
        AngleX=0.0;      
   }   
   double  RX[4][4],RY[4][4],RZ[4][4],TT[4][4],mat[4][4],RXA[4][4],RYA[4][4];
   GetTranslateMatrix(OriginXYZ,&TT[0][0]);
   GetRotateMatrix('X',AngleX,&RX[0][0]);
   GetRotateMatrix('Y',AngleY,&RY[0][0]);
   GetRotateMatrix('Z',AngleZ,&RZ[0][0]);
   MultMatrix(&TT[0][0],&RX[0][0],&mat[0][0],4,4);
   MultMatrix(&mat[0][0],&RY[0][0],Matrix,4,4);
   MultMatrix(Matrix,&RZ[0][0],&mat[0][0],4,4);
   GetTranslateMatrix(Origin,&TT[0][0]);
//   GetRotateMatrix('X',-AngleX,&RX[0][0]);
//   GetRotateMatrix('Y',-AngleY,&RY[0][0]); 
   MatrixTranspose(&RX[0][0],&RXA[0][0],4,4);
   MatrixTranspose(&RY[0][0],&RYA[0][0],4,4);
   MultMatrix(&mat[0][0],&RYA[0][0],Matrix,4,4);
   MultMatrix(Matrix,&RXA[0][0],&mat[0][0],4,4);
   MultMatrix(&mat[0][0],&TT[0][0],Matrix,4,4);     
}

template<class T>//向量旋转到Z轴的旋转矩阵
void __declspec(dllexport) GetRotateToZAxisMatrix(const char& c,const tagCVector& Origin,const tagCVector& Axis,T* Matrix)
{// c='A' 'a'  antimatrix       c='P'  'p' positive matrix
   double AngleX,AngleY;
   tagCVector OriginXYZ,axis=Axis;
   OriginXYZ.x=-Origin.x;
   OriginXYZ.y=-Origin.y;
   OriginXYZ.z=-Origin.z;
   axis.normalize();
   AngleY=-asin(axis.x)*180.0/PI;
   if(fabs(axis.z)>0.0)
   {
       AngleX=atan2(axis.y,axis.z)*180.0/PI;//  if(Axis.z<0.0)//  AngleX=AngleX+PI;
   }
   else
   {
     if(axis.y>0.0)
        AngleX=90.0;
     else if(axis.y<0.0)
        AngleX=-90.0;  
     else
        AngleX=0.0;      
   }  
   double  RX[4][4],RY[4][4],TT[4][4],mat[4][4];
   switch(c)
   {
   case 'A':
   case 'a':
      GetTranslateMatrix(Origin,&TT[0][0]);
      GetRotateMatrix('X',-AngleX,&RX[0][0]);
      GetRotateMatrix('Y',-AngleY,&RY[0][0]); 
      MultMatrix(&RY[0][0],&RX[0][0],&mat[0][0],4,4);
      MultMatrix(&mat[0][0],&TT[0][0],Matrix,4,4);  
   	break;
   case 'P':
   case 'p':
   default:
      GetTranslateMatrix(OriginXYZ,&TT[0][0]);
      GetRotateMatrix('X',AngleX,&RX[0][0]);
      GetRotateMatrix('Y',AngleY,&RY[0][0]);
      MultMatrix(&TT[0][0],&RX[0][0],&mat[0][0],4,4);
      MultMatrix(&mat[0][0],&RY[0][0],Matrix,4,4);     
   	break;
   }       
}

/////////////////////////////////////////////////////////////////////////////////////////////
#endif