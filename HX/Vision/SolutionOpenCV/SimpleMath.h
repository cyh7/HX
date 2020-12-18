#pragma once
#include <opencv.hpp>





namespace SimpleMath
{

	cv::Point2d GetCrossPoint(double x1, double y1, double x2, double y2,
		double x3, double y3, double x4, double y4);
	cv::Point2d GetCrossPoint(cv::Point2d l1Start, cv::Point2d l1End, cv::Point2d l2Start, cv::Point2d l2End);
	double GetVectorAngle(double x1, double y1, double x2, double y2);
}

