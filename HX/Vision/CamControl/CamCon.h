#pragma once

#include "../../pch.h"
#include "../opencv_include.h"
#include "../../CPublic.h"

using namespace std;
using namespace cv;


//初始化PitureControl控件
void PicConInit(HWND hWnd, int IDD, const char* windowname);



class CSampleDeviceOfflineEventHandler : public IDeviceOfflineEventHandler
{
public:

	void DoOnDeviceOfflineEvent(void* pUserParam);
};

//用户继承属性更新事件处理类

class CSampleFeatureEventHandler : public IFeatureEventHandler
{
public:
	void DoOnFeatureEvent(const GxIAPICPP::gxstring& strFeatureName, void* pUserParam)
	{
		cout << "收到曝光结束事件!" << endl;
	}
};

//用户继承采集事件处理类
class CSampleCaptureEventHandler : public ICaptureEventHandler
{
public:
	
	void DoOnImageCaptured(CImageDataPointer& objImageDataPointer, void* pUserParam);
	
};
class SingleCam;


/// <summary>
/// Cam总线取消，做成单独的相机总线初始化函数，库init + 扫描
/// 放入两个相机
/// </summary>
class CamCon //Cam总线
{
public:

	enum {
		LEFT,
		RIGHT,
	};


	CamCon() { CamInit(); };
	~CamCon() { CamUnInit(); };
	int init_all_cam(vector<shared_ptr<SingleCam>>& cam_vec);
	//初始化函数
	int CamInit();
	int CamUnInit();


	static shared_ptr<SingleCam> LEFT_CAM;
	static shared_ptr<SingleCam> RIGHT_CAM;

public:
	
private:
	//禁止复制
	CamCon(const CamCon&);
	CamCon operator=(const CamCon&);
	
	gxdeviceinfo_vector vectorDeviceInfo;//整个总线包含的相机信息
	Mat img;
	
};
class SingleCam//独立相机
{
	friend class CSampleCaptureEventHandler;
	friend class CamCon;
public:

	SingleCam(CGXDevicePointer m_ODP , gxstring m_serialNum, const char* windowname,cv::Rect roi = cv::Rect(PC_WIDTH / 3, 0, PC_WIDTH / 3, PC_HEIGHT) );
	~SingleCam();
public:
	void Record_start();
	void Record_stop();
	cv::Rect ROI;

	inline bool& IsOffline()
	{
		return m_bIsOffline;
	}

	gxstring GetSN() const
	{
		return serialNum;
	}
	string GetShowWindow() const
	{
		return showWindowName;
	}
	Mat GetSrc() const
	{
		return src;
	}

private:
	//禁止复制相机
	SingleCam(const SingleCam&);
	const SingleCam& operator=(const SingleCam&);
	//掉线处理
	void __ProcessOffline();
	//重连函数
	void __Recovery();
	//各种Handler
	IDeviceOfflineEventHandler *pDeviceOfflineEventHandler;///<掉线事件回调对象
	IFeatureEventHandler       *pFeatureEventHandler;///<远端设备事件回调对象
	ICaptureEventHandler       *pCaptureEventHandler;///<采集回调对象
	CGXFeatureControlPointer   ObjFeatureControlPtr;//设备属性控制器

	CGXDevicePointer ObjDevicePtr;//设备指针
	CGXStreamPointer ObjStreamPtr;//打开流

	GX_DEVICE_OFFLINE_CALLBACK_HANDLE hDeviceOffline; //掉线回调句柄

	//标志位，表示设备是否离线
	bool m_bIsOffline;
	bool m_bIsSnap;
	bool m_bIsOpened;

	//序列号
	gxstring serialNum;
	//画面显示窗口名称
	string showWindowName;
	
	//源图
	Mat src;
	//裁切后的原图像
	Mat croppedSrc;
	//缩小后的原图
	Mat resizedSrc;
	//缩小后的裁切图
	Mat resizedCropped;

};



//之后双相机会改的代码
int StartCam(std::shared_ptr<SingleCam> p);
int StopCam(std::shared_ptr<SingleCam> p);

static CamCon& CAMVEC()
{
	static CamCon c1;
	return c1;
}
static vector<shared_ptr<SingleCam>>& SCV()
{
	static vector<shared_ptr<SingleCam>> c1;
	return c1;
}



