/*
 * myFunctions.h
 *
 *  Created on: Jul 31, 2017
 *      Author: michael
 */

#ifndef CALIB_ZEDLIDAR_H_
#define CALIB_ZEDLIDAR_H_


#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <fstream>
#include <iostream>
#include <sstream>
#include <math.h>
#include <time.h>
#include <vector>
#include <cmath>
#include <signal.h>
#include "ceres/ceres.h"
#include "glog/logging.h"
using ceres::AutoDiffCostFunction;
using ceres::CostFunction;
using ceres::Problem;
using ceres::Solver;
using ceres::Solve;

#include <sl/Camera.hpp>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/calib3d.hpp>
#include <rplidar.h>

#ifndef _countof
#define _countof(_Array) (int)(sizeof(_Array) / sizeof(_Array[0]))
#endif

#ifdef _WIN32
#include <Windows.h>
#define delay(x)   ::Sleep(x)
#else
#include <unistd.h>
static inline void delay(_word_size_t ms) {
	while (ms >= 1000) {
		usleep(1000 * 1000);
		ms -= 1000;
	};
	if (ms != 0)
		usleep(ms * 1000);
}
#endif
const float PI = 3.1415927;

using namespace std;
using namespace sl;
using namespace cv;
using namespace rp::standalone::rplidar;


void getCheckerboardPlane(Camera &zed,Size &patternsize,cv::Mat &solution, vector<Point3f> &p_corners3D,vector<Point2f> &p_corners, bool drawNormal);
void writeCSV(string filename, cv::Mat m);
void loadFromCSV(const string& values, int opencv_type, cv::Mat &m);

void polar2cartesian(vector<vector<double>> &input, vector<Point3f> &output);
void readLidarPolar(RPlidarDriver * drv, vector<Point2f> &output, int n_cycles);
void readLidarCart(RPlidarDriver * drv, vector<Point3f> &output, int n_cycles);
void readLidarProjectedImage(Camera &zed, RPlidarDriver * drv,vector<Point2f> &output2D,vector<Point3f> &output3D, int n_cycles);
// LidarFunctions
void getLidarInfo(RPlidarDriver * drv, const char * opt_com_path, _u32 opt_com_baudrate,bool &support_express);
void connectLidar(RPlidarDriver * drv, const char * opt_com_path, _u32 opt_com_baudrate);
void startLidar(RPlidarDriver * drv);
void grabLidarDataComplete(RPlidarDriver * drv, vector<double> &distance, vector<double> &angle, vector<int> &startFlagBit, vector<double> &scanQuality, float &frequency, bool &support_express, bool &in4kMode);
void grabLidarDataBasic(RPlidarDriver * drv, vector<double> &distance, vector<double> &angle);

// ZED Functions
void zedTracking(Camera &zed, Pose &zed_pose);
void setCamera(Camera &zed);
void getProjectionMatrices(Camera &zed, cv::Mat &R, cv::Mat &t);
void getCameraMatrix(Camera &zed, cv::Mat &K, cv::Mat &dist);
cv::Mat slMat2cvMat(sl::Mat& input);


#endif /* CALIB_ZEDLIDAR_H_ */
/*
 * myFunctions.h
 *
 *  Created on: Jul 31, 2017
 *      Author: michael
 */
