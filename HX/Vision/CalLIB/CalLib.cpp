#include "CalLib.h"







bool CalLib::load_distortion_file()
{
	FileStorage fs(configFileName, FileStorage::READ);
	if (!fs.isOpened())
	{
		cout << "Cant find " << configFileName << endl;
		cout << "Have you run the rectification yet?" << endl;
		return false;
	}
	
	distortion_coefficients = Mat();
	camera_matrix = Mat(3, 3, CV_32FC1);
	fs["intrinsic_matrix"] >> camera_matrix;
	fs["distortion_coeffs"] >> distortion_coefficients;
	fs.release();

	cout << camera_matrix;

	return true;
}
bool CalLib::GetPnPParam(vector<Point2f> imagePoints, vector<Point3f> objectPoints)
{
	if (camera_matrix.empty() || distortion_coefficients.empty())
	{
		bool flag = load_distortion_file();
		if (flag == false)
			return false;
	}
	rvec = Mat(1, 3, cv::DataType<double>::type);
	tvec = Mat(1, 3, cv::DataType<double>::type);
	rotationMatrix = Mat(3, 3, cv::DataType<double>::type);

	

	solvePnP(objectPoints, imagePoints, camera_matrix, distortion_coefficients, rvec, tvec);

	rotationMatrixInv = rotationMatrix.inv();
	cameraMatrixInv = camera_matrix.inv();

	leftSideMat  = rotationMatrixInv * cameraMatrixInv;
	rightSideMat = rotationMatrixInv * tvec;
	return true;
}
bool CalLib::RectifyImg(Mat& input, Mat& output)
{
	if (camera_matrix.empty())
	{
		if (!load_distortion_file())
		{
			return false;
		}
	}
	output = input.clone();

	undistort(input, output, camera_matrix, distortion_coefficients);
	
	return true;
}

double getDistance(cv::Point2d pointO, cv::Point2d pointA)
{
	double distance;
	distance = powf((pointO.x - pointA.x), 2) + powf((pointO.y - pointA.y), 2);

	return sqrtf(distance);
}


bool CalLib::initPnP(vector<Point2f> imagePoints, vector<Point3f> objectPoints)
{
	if (bisRectified)
	{
		GetPnPParam(imagePoints, objectPoints);
	}

	return true;
}

bool CalLib::GetRealWorldCoordinate(const cv::Point2d& imagePoint, cv::Point3d& outputPoint)
{
	Mat uvPoint = (Mat_<double>(3, 1) << imagePoint.x, imagePoint.y, 1);

	constexpr size_t height = 0;//所有的实物平面都默认在同一平面

	leftSideMat *= uvPoint;

	double s = (height + rightSideMat.at<double>(2, 0) / leftSideMat.at<double>(2, 0));

	cout << "P = " << rotationMatrixInv * (s * cameraMatrixInv * uvPoint - tvec) << std::endl;

	return true;
}

bool CalLib::cal_lib_init()
{
	//执行拍照函数，等伟哥
	//使用glob读取文件夹下的所有文件
	glob(CALLIB_PIC_DIR, filename);

	if (filename.empty())
	{
		cout << "cant find any chessboard, retification failed!" << endl;
		return false;
	}

	for (size_t i = 0; i < filename.size(); ++i)
	{
		cout << filename[i] << endl;
		Mat src = imread(filename[i]);
		if (!src.data)
			cerr << "Problem loading image!!!" << endl;
		VecMat.push_back(src);
	}

	

	return true;
}


bool CalLib::cal_intrinsic_distortion()
{
	
	if (!cal_lib_init())
		return false;
	//棋盘格的大小，每行/列的角点个数
	const Size boardsize = Size(CORNERS_LS, CORNERS_SS);
	//记录每幅图中的角点
	vector<vector<Point2f>> cor_pixpos_vec;
	//记录所有图像中的角点总和
	int pic_cor_count = 0;
	//棋盘格图像
	Mat srcImg;
	//图像的个数
	int image_count = 0;


	for (int i = 0; i < TOTAL_PIC; i++)
	{
		//记录每幅图像的名称
		//string Img_files = string("Cam_cal/left") +to_string(i) + string(".jpg");
		//读取图像，若读取失败则发送提示

		srcImg = VecMat[i];
		/*if (!srcImg.data)
		{
			cout << "读取" <<  << "失败，请确定目录下是否存在该图片？" << endl;
			waitKey();
			continue;
		}*/

		//转换为灰度图像
		Mat grayImg;
		cvtColor(srcImg, grayImg, COLOR_BGR2GRAY);

		//查找棋盘格图像角点
		vector<Point2f> corners;
		bool cornerfound = findChessboardCorners(grayImg, boardsize, corners, CALIB_CB_ADAPTIVE_THRESH);

		{
			//尝试输出第一幅图两个角点之间的像素距离推断内参矩阵是否正确
			if (i == 0)
			{
				Point2f &p1 = corners[0], &p2 = corners[1];
				Point2f& p3 = corners[2], & p4 = corners[3];
				cout << "point1 x " << p1.x << "point1 y" << p1.y << endl;
				cout << "point2 x " << p2.x << "point2 y" << p2.y << endl;
				cout << "pixel distance" << getDistance(p1, p2) << endl;
				cout << "pixel distance2" << getDistance(p1, p2) << endl;
			}
		}


		if (!cornerfound)
		{
			cout << "第 " << i + 1 <<" 个图片角点查找失败" << endl;
			continue;
		}
		Mat tmp = grayImg.clone();
		drawChessboardCorners(tmp, boardsize, corners, true);
		stringstream s; string fileName;
		s << i << ".bmp";
		s >> fileName;
		imwrite(fileName, tmp);
		//根据棋盘格图像角点寻找亚像素角点
		cornerSubPix(grayImg, corners, Size(5, 5), Size(-1, -1), TermCriteria(TermCriteria::EPS + TermCriteria::MAX_ITER, 30, 0.001));

		//累计角点的个数
		pic_cor_count += corners.size();
		//将本幅图像的角点添加到列表中
		cor_pixpos_vec.push_back(corners);
		//图像个数+1
		image_count++;
		cout << "完成图片 " << i <<" 信息收集" << endl;
	}
	Size square_size = Size(RWSSL, RWSSL);

	vector<Point3f> tempPointset;
	for (int h = 0; h < boardsize.height; h++)
	{
		for (int w = 0; w < boardsize.width; w++)
		{
			Point3f tempPoint;
			tempPoint.x = w * square_size.width;
			tempPoint.y = h * square_size.height;
			tempPoint.z = 0;
			tempPointset.push_back(tempPoint);
		}
	}
	vector<vector<Point3f>> object_Points(image_count, tempPointset);//由于我的标定板不变，所以所有的图片都是一样的

	//记录相机
	//   fx,  0, cx
	//    0, fy, cy
	//    0,  0, 1
	//内参矩阵的参数
	//Mat intrinsic_matrix = Mat(3, 3, CV_32FC1, Scalar::all(0));
	Mat intrinsic_matrix = Mat::eye(3, 3, CV_64F);
	//记录相机畸变的参数
	Mat distortion_coeffs = Mat(1, 5, CV_32FC1, Scalar::all(0));
	//记录每幅图像的旋转矩阵
	vector<cv::Mat> rotation_vectors;
	//记录每幅图像的位移矩阵
	vector<cv::Mat> translation_vectors;

	calibrateCamera(object_Points, cor_pixpos_vec, srcImg.size(), intrinsic_matrix, distortion_coeffs, rotation_vectors, translation_vectors, 0);

	double total_err = 0.0;
	for (int i = 0; i < image_count; i++)
	{
		vector<Point2f> image_points2;//反求出来的角点像素坐标

		//获取每幅图像世界坐标系下的角点坐标
		vector<Point3f> tempPointSet = object_Points[i];
		//根据相机的内参和外参计算得出图像坐标系下的角点坐标
		projectPoints(tempPointSet, rotation_vectors[i], translation_vectors[i], intrinsic_matrix, distortion_coeffs, image_points2);

		//记录真实图像坐标系下的角点坐标
		vector<Point2f> tempImagePoint = cor_pixpos_vec[i];

		//将计算得出的图像坐标系下的角点 和 真实图像坐标系下的角点坐标分别放入两个图像中
		Mat tempImagePointMat = Mat(1, tempImagePoint.size(), CV_32FC2);
		Mat image_points2Mat = Mat(1, image_points2.size(), CV_32FC2);
		for (size_t i = 0; i != tempImagePoint.size(); i++)
		{
			image_points2Mat.at<Vec2f>(0, i) = Vec2f(image_points2[i].x, image_points2[i].y);
			tempImagePointMat.at<Vec2f>(0, i) = Vec2f(tempImagePoint[i].x, tempImagePoint[i].y);
		}
		//计算两个图像的像素误差
		double err = norm(image_points2Mat, tempImagePointMat, NORM_L2);
		//汇总像素误差
		total_err += err /= (CORNERS_LS * CORNERS_SS);//point_counts[0]
		cout << "第" << i << "幅图像的平均误差：" << err << "像素" << endl;
	}
	cout << "总体平均误差：" << total_err / image_count << "像素" << endl << endl;


	//将相机标定参数写入xml文件中
	FileStorage fs1("calibration.xml", FileStorage::WRITE);
	fs1 << "intrinsic_matrix" << intrinsic_matrix;
	fs1 << "distortion_coeffs" << distortion_coeffs;

	camera_matrix = intrinsic_matrix.clone();
	distortion_coefficients = distortion_coeffs.clone();

	fs1.release();
}
