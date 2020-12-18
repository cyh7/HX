#pragma once

#include "../opencv_include.h"
#include "../STL_include.h"

using namespace cv;
using namespace std;


const string CALLIB_PIC_DIR("Cam_Cal");
const string PREFIX("CAL");
class CalLib
{
public:
	//total picture number , 长边角点数量， 短边角点数量， 实际方块边长
	CalLib(size_t m_TP = 20 , size_t m_CLS = 11, size_t m_CSS = 8, size_t rwssl = 30, String m_configFileName = "calibration.xml") :
		TOTAL_PIC(m_TP), CORNERS_LS(m_CLS), CORNERS_SS(m_CSS), RWSSL(rwssl), configFileName(m_configFileName)
	{
		bisRectified = load_distortion_file();
	};
	//是否已经矫正
	bool bisRectified;
	
	bool cal_intrinsic_distortion(); //相机矫正函数

	//存入待矫正图像
	void CalLibGrabPic();

	//读取内参矩阵和畸变系数并对图片进行矫正, 警告，矫正耗时较长
	bool RectifyImg(Mat& input, Mat& output);
	
	//初始化Pnp
	bool initPnP(vector<Point2f> imagePoints, vector<Point3f> objectPoints);
	//获取实际位置坐标
	bool GetRealWorldCoordinate(const cv::Point2d& imagePoint, cv::Point3d& outputPoint); //获取实际世界坐标


private:
	//读取所有的畸变图片
	bool cal_lib_init();
	//载入畸变参数和内参矩阵
	bool load_distortion_file();

	//算出单应性矩阵以及对应的各个矩阵
	bool GetPnPParam(vector<Point2f> imagePoints, vector<Point3f> objectPoints);



	const size_t TOTAL_PIC;
	const size_t CORNERS_LS;
	const size_t CORNERS_SS;
	const size_t RWSSL;

	//a vector string that stores all the pic path
	vector<string>     filename;
	vector<cv::Mat>	   VecMat;
	string configFileName;

	//内参矩阵
	Mat camera_matrix;
	//畸变参数
	Mat distortion_coefficients;
	//外参矩阵，两种
	cv::Mat rvec;//旋转向量
	cv::Mat tvec;//平移向量
	cv::Mat rotationMatrix;   //通过以上两个获得
	cv::Mat leftSideMat;      //等式左侧的向量
	cv::Mat rightSideMat;     //等式右测的向量
	cv::Mat rotationMatrixInv;//旋转矩阵逆矩阵
	cv::Mat cameraMatrixInv;  //内参矩阵逆矩阵

	

	void take_picture();

	
};

