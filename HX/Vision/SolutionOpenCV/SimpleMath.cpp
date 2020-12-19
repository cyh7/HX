#include "SimpleMath.h"

cv::Point2d SimpleMath::GetCrossPoint(double x1, double y1, double x2, double y2,
    double x3, double y3, double x4, double y4 )
{
    /*定义直线参数方程如下
        x = x1 + t1*(x2 - x1);
        y = y1 + t1*(y2 - y1);

        解t为
        t1_1 = x3*(y4 - y3) + y1*(x4 - x3) - y3*(x4 - x3) - x1*(y4 - y3);
        t1_2 = (x2 - x1) * (y4 - y3) - (x4 - x3) * (y2 - y1)
        t = t1_1 / t1_2;

        return Point2d(x, y);
    */

    double x = 0.0, y = 0.0;
    double t1_1 = x3 * (y4 - y3) + y1 * (x4 - x3) - y3 * (x4 - x3) - x1 * (y4 - y3);
    double t1_2 = (x2 - x1) * (y4 - y3) - (x4 - x3) * (y2 - y1);

    double t = t1_1 / t1_2;

    x = x1 + t * (x2 - x1);
    y = y1 + t * (y2 - y1);

    return cv::Point2d(x, y);

}

cv::Point2d SimpleMath::GetCrossPoint(cv::Point2d l1Start, cv::Point2d l1End, cv::Point2d l2Start, cv::Point2d l2End)
{
    return SimpleMath::GetCrossPoint(l1Start.x, l1Start.y, l1End.x, l1End.y, l2Start.x, l2Start.y, l2End.x, l2End.y);
}

double SimpleMath::GetVectorAngle(double x1, double y1, double x2, double y2)
{
    /*double t = ((vector1.x * vector2.x) + (vector1.y * vector2.y)) / (sqrt(pow(vector1.x, 2) + pow(vector1.y, 2)) * sqrt(pow(vector2.x, 2) + pow(vector2.y, 2)));
        cout << "这两个向量的夹角为:" << acos(t) * (180 / PI) << "度" << endl;*/
    double t = ((x1 * x2) + (y1 * y2)) / (sqrt(pow(x1, 2) + pow(y1, 2)) * sqrt(pow(x2, 2) + pow(y2, 2)));
    t = acos(t) * (180 / 3.1415926535);
    return t;
}
