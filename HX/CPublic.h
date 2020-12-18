#pragma once
#include "Vision/opencv_include.h"
#include "Vision/STL_include.h"
using namespace std;

//相机参数，放缩比
enum
{
	PC_WIDTH = 5496,
	PC_HEIGHT = 3672,
	AS_RATIO = 10,
};

struct CPublic
{

	//返回一个存放着Mat的vector的引用
	//第一个存放着完全没缩放过的原图
	//第二个位置存放着缩放过的原图
	//第三个存放裁切后的原图
	enum
	{
		ORIGINAL,
		RESIZED,
		CROPPED_ORIGINAL,
		CROPPED_RESIZED,
	};
	static std::vector<cv::Mat>& Mat_Vec()
	{
		static std::vector<cv::Mat> Mat_Vec(4);
		return Mat_Vec;
	}


	//返回左显示窗口名称
	static constexpr char* LEFT_MATWINDOW_NAME()
	{
		return "view1";
	}
	static constexpr char* RIGHT_MATWINDOW_NAME()
	{
		return "view2";
	}

	//返回左相机SN
	static const string& LEFT_CAM_SN()
	{
		static const string LCSN("LT0200091521");
		return LCSN;
	}
	//返回右相机SN
	static const string& RIGHT_CAM_SN()
	{
		static const string RCSN("LT0200091520");
		return RCSN;
	}


	//返回相机1的ROI的指针句柄
	static cv::Rect*& ROI_1()
	{
		static cv::Rect* roi_1 = NULL;
		return roi_1;
	}

	//测试用，模拟相机1的ROI,直接返回一个Rect引用
	static cv::Rect& ROI1Temp()
	{
		static cv::Rect ro1(PC_WIDTH / 3, 0, PC_WIDTH / 3, PC_HEIGHT);
		return ro1;
	}

	static std::vector<bool>& allRuntimeFlag()
	{
		static std::vector<bool> b;
		return b;
	}
	 
	//返回相机2的ROI，暂时先不用做
	static cv::Rect* ROI_2();

	/* static bool& Selection_enabled()
	{
		static bool flag = false;
		return flag;
	}
	*/
	static bool leftSelectionEnabled;






};



