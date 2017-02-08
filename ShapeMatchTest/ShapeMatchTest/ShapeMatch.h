#pragma once
//#include "stdafx.h"

using namespace std;
using namespace cv;

#define		MAX_NUM_INSTANCES		300				//最大目标个数
#define		MIN_NUM_LEVELS				0					//最小金字塔级数
#define		MAX_NUM_LEVELS				5					//最大金字塔级数

const int K_CosineTable[24] =
{
	8192, 8172, 8112, 8012, 7874, 7697,
	7483, 7233, 6947, 6627, 6275, 5892, 
	5481, 5043, 4580, 4096, 3591, 3068,
	2531, 1981, 1422, 856,   285,   -285
};

//匹配结果结构体
struct MatchResultA
{
	int 			Angel;									//匹配角度
	int 			CenterLocX;							//匹配参考点X坐标
	int			CenterLocY;							//匹配参考点Y坐标
	float 		ResultScore;							//匹配的分
};

//特征信息结构体
struct ShapeInfo
{
	CvPoint			ReferPoint;					//模板重心坐标RefPoint
	CvPoint			*Coordinates;				//模板坐标数组
	float					*EdgeMagnitude;			//梯度导数
	short				*EdgeDerivativeX;			//X方向梯度
	short				*EdgeDerivativeY;			//Y方向梯度
	int 					ImgWidth;					//图像宽度
	int					ImgHeight;					//图像高度
	int					VectorSize;					//数组大小
	int					NoOfCordinates;			//轮廓点个数
	int					Angel;							//旋转角度
	int					PyLevel;						//金字塔级别
	int					AngleNum;					//角度个数
};

//模板文件结构体
struct shape_model
{
	int	ID;									//模板ID
	int 	m_NumLevels;					//金字塔级数
	int 	m_Contrast;						//高阈值
	int 	m_MinContrast;				//低阈值
	int 	m_AngleStart;					//模板旋转起始角度
	int 	m_AngleExtent;					//模板旋转角度幅度
	int 	m_AngleStep;					//角度步长
	int 	m_PointReduction;			//匹配加速因子
	int    m_ImageWidth;				//原模板图像宽度
	int    m_ImageHeight;				//原模板图像高度
	bool	m_IsInited;						//初始化标志

	ShapeInfo *m_pShapeInfoPyd1Vec;			//模板金字塔第1级图像的边缘信息
	ShapeInfo *m_pShapeInfoPyd2Vec;			//模板金字塔第2级图像的边缘信息
	ShapeInfo *m_pShapeInfoPyd3Vec;			//模板金字塔第3级图像的边缘信息
	ShapeInfo *m_pShapeInfoTmpVec;			//原模板图像的边缘信息
};

//搜索区域
struct search_region
{
	int 	StartX;							//X方向起点
	int 	StartY;							//y方向起点
	int 	EndX;							//x方向终点
	int 	EndY;							//y方向终点
	int 	AngleRange;					//搜索角度数目
	int    AngleStart;					//搜索预先角度
	int	AngleStop;					//搜索终止角度
	int    AngleStep;					//搜索角度步长
};

//边界点列表
struct edge_list
{
	CvPoint *EdgePiont;				//边缘坐标数组
	int 	ListSize;						//数组大小
	int	Granularity;					//边缘颗粒度
};

class CShapeMatch
{
public:
	CShapeMatch(void);
	~CShapeMatch(void);

	void gaussian_filter(uint8_t* corrupted, uint8_t* smooth, int width, int height);
		/*--------------------------------------------------------------------------------------------*/
	/*	函数名：gen_rectangle
		函数功能：生成ROI
		输入变量：Image 输入图像, Row1 左上角点的横坐标, Column1 左上角点的纵坐标
		返回变量：
		注释：		*/

	void gen_rectangle(IplImage *Image, IplImage *ModelRegion, int Row1, int Column1);
	/*--------------------------------------------------------------------------------------------*/
	/*	函数名：gen_rectangle
		函数功能：生成ROI
		输入变量：Image 输入图像, Row1 左上角点的横坐标, Column1 左上角点的纵坐标
		返回变量：
		注释：		*/

	void board_image(IplImage *SrcImg, IplImage *ImgBordered, int32_t xOffset, int32_t yOffset);
	/*--------------------------------------------------------------------------------------------*/
	/*	函数名：board_image
		函数功能：图像边界扩展
		输入变量：SrcImage 输入图像, DstImage 输出图像
		返回变量：
		注释：		*/

	void rotate_image (uint8_t *SrcImgData, uint8_t *MaskImgData, int srcWidth, int srcHeight, uint8_t *DstImgData, uint8_t *MaskRotData, int dstWidth, int dstHeight, int Angle);
	/*--------------------------------------------------------------------------------------------*/
	/*	函数名：rotate_image
		函数功能：图像旋转函数
		输入变量：SrcImage 输入图像, DstImage 输出图像, Angle旋转角度
		返回变量：DstImgData 旋转后的图像
		注释：		*/

	void rotateImage (IplImage* srcImage, IplImage* dstImage, int Angle);
	/*--------------------------------------------------------------------------------------------*/
	/*	函数名：rotate_image
		函数功能：图像旋转函数
		输入变量：SrcImage 输入图像, DstImage 输出图像, Angle旋转角度
		返回变量：DstImgData 旋转后的图像
		注释：		*/

	void image_pyramid(uint8_t *SrcImgData,  int srcWidth, int srcHeight, uint8_t *OutImgData);
	/*--------------------------------------------------------------------------------------------*/
	/*	函数名：imagePyramid
		函数功能：图像金字塔函数 2*2采样
		输入变量：SrcImage 输入图像, srcWidth 图像宽度， srcHeight 图像高度
		返回变量：OutImageData 金字塔图像
		注释：		*/

	void imagePyramid(uint8_t *SrcImgData,  int srcWidth, int srcHeight, uint8_t *OutImgData);
	/*--------------------------------------------------------------------------------------------*/
	/*	函数名：imagePyramid
		函数功能：图像金字塔函数 双线性插值
		输入变量：SrcImage 输入图像, srcWidth 图像宽度， srcHeight 图像高度
		返回变量：OutImageData 金字塔图像
		注释：		*/

	void initial_shape_model(shape_model *ModelID, int Width, int Height, int EdgeSize);
	/*--------------------------------------------------------------------------------------------*/
	/*	函数名：initial_shape_model
		函数功能：初始化模版资源
		输入变量：ModelID 模板文件
		返回变量：
		注释：		*/

	bool release_shape_model(shape_model *ModelID);
	/*--------------------------------------------------------------------------------------------*/
	/*	函数名：release_shape_model
		函数功能：释放模版资源
		输入变量：ModelID 模板文件
		返回变量：
		注释：		*/

	void extract_shape_info(uint8_t *ImageData, ShapeInfo *ShapeInfoData, int Contrast, int MinContrast, int PointReduction, uint8_t *MaskImgData);
	/*--------------------------------------------------------------------------------------------*/
	/*	函数名：extract_shape_info
		函数功能：提取形状信息
		输入变量：ImageData 图像数据, MinContrast 最小阈值, Contrast 阈值
		返回变量：shape_info 形状信息
		注释：		*/

	bool build_model_list(ShapeInfo *ShapeInfoVec, uint8_t *ImageData, uint8_t *MaskData, int Width, int Height, int Contrast, int MinContrast, int AngleStart, int AngleStop, int AngleStep, int Graininess);
	/*--------------------------------------------------------------------------------------------*/
	/*	函数名：build_model_list
		函数功能：创建角度模板序列
		输入变量：ShapeInfoVec 序列指针, MinContrast 最小阈值, Contrast 阈值, AngleStart 起始角度, AngleStop 终止角度, AngleStep 角度步长, Graininess 边缘颗粒度
		返回变量：ModelID 模板文件
		注释：		*/

	void train_shape_model(IplImage *Image, int Contrast, int MinContrast, int PointReduction, edge_list *EdgeList);
	/*--------------------------------------------------------------------------------------------*/
	/*	函数名：train_shape_model
		函数功能：训练形状模板
		输入变量：Image 输入图像, MinContrast 最小阈值, Contrast 阈值
		返回变量：EdgeList 边缘点序列
		注释：		*/

	bool create_shape_model(IplImage *Template, shape_model *ModelID);
	/*--------------------------------------------------------------------------------------------*/
	/*	函数名：create_shape_model
		函数功能：创建匹配模板
		输入变量：Template 模板图像, NumLevels 金字塔级数, AngleStart 起始角度, AngleExtent 角度范围, AngleStep 角度步长, PointReduction 边缘点缩减因子, Contrast
		返回变量：ModelID 模板文件
		注释：		*/

	void shape_match(uint8_t *SearchImage, ShapeInfo *ShapeInfoVec, int Width, int Height, int *NumMatches, int Contrast, int MinContrast, float MinScore, float Greediness, search_region *SearchRegion, MatchResultA *ResultList);
	/*--------------------------------------------------------------------------------------------*/
	/*	函数名：shape_match
		函数功能：形状匹配函数
		输入变量：SearchImage 待搜索图像数据,  ShapeInfo 模板形状信息, Width 图像宽度, Height 图像高度, NumMatches 匹配目标数 MinScore 最小评分, Greediness 贪婪度, SearchRegion 搜索范围
		返回变量：ResultList 匹配结果
		注释：		*/
	
	void shape_match_accurate(uint8_t *SearchImage, ShapeInfo *ShapeInfoVec, int Width, int Height, int Contrast, int MinContrast, float MinScore, float Greediness, search_region *SearchRegion, MatchResultA *ResultList);
	/*--------------------------------------------------------------------------------------------*/
	/*	函数名：shape_match
		函数功能：形状匹配函数
		输入变量：SearchImage 待搜索图像数据,  ShapeInfo 模板形状信息, Width 图像宽度, Height 图像高度, NumMatches 匹配目标数 MinScore 最小评分, Greediness 贪婪度, SearchRegion 搜索范围
		返回变量：ResultList 匹配结果
		注释：		*/

	void find_shape_model(IplImage *Image, shape_model *ModelID, float MinScore, int NumMatches, float Greediness, MatchResultA *ResultList);
	/*--------------------------------------------------------------------------------------------*/
	/*	函数名：find_shape_model
		函数功能：在搜索图中寻找模板目标
		输入变量：Image 待搜索图像,  ModelID 模板文件, AngleStart 起始角度, AngleExtent 角度范围, MinScore 最小评分, NumMatches 匹配目标数
		返回变量：Row 匹配参考点点X坐标, Column 匹配参考点点Y坐标, Angle 目标旋转角度, Score 评分
		注释：		*/

	int  ShiftCos(int y);
	/*--------------------------------------------------------------------------------------------*/
	/*	函数名：ShiftCos
		函数功能：余弦三角函数
		输入变量：y 角度
		返回变量：余弦值

		注释：		*/

	int  ShiftSin(int y);
	/*--------------------------------------------------------------------------------------------*/
	/*	函数名：ShiftSin
		函数功能：正弦三角函数
		输入变量：y 角度
		返回变量：正弦值
		注释：		*/

	float Q_rsqrt( float number );
	/*--------------------------------------------------------------------------------------------*/
	/*	函数名：Q_rsqrt
		函数功能：求解平方根倒数
		输入变量：number
		返回变量：平方根倒数
		注释：		*/

	float new_rsqrt(float f);
	/*--------------------------------------------------------------------------------------------*/
	/*	函数名：new_rsqrt
		函数功能：求解平方根倒数
		输入变量：f
		返回变量：平方根倒数
		注释：		*/

	void QuickSort(MatchResultA *s, int l, int r);
	/*--------------------------------------------------------------------------------------------*/
	/*	函数名：QuickSort
		函数功能：数组排序
		输入变量：s[] 结果数组，
		返回变量：
		注释：		*/

	int ConvertLength(int LengthSrc);
	/*--------------------------------------------------------------------------------------------*/
	/*	函数名：ConvertLength
		函数功能：长度转换
		输入变量：LengthSrc 待转换长度，
		返回变量：转换结果
		注释：		*/
};

