#include <myFunctions.h>


struct CostFunctor {
   template <typename T>
   bool operator()(const T* const x, T* residual) const {
     residual[0] = T(10.0) - x[0];
     return true;
   }
};



int main() {

	Camera zed;	// Create a ZED camera object
	setCamera(zed);

	/*
	const char * opt_com_path = "/dev/rplidar"; // Surface
	_u32         opt_com_baudrate = 115200;
	u_result     op_result_scan;
	// create the driver instance 
	RPlidarDriver * drv = RPlidarDriver::CreateDriver(RPlidarDriver::DRIVER_TYPE_SERIALPORT);
	
	// Gets Device Infos
	connectLidar(drv,opt_com_path,opt_com_baudrate);
	startLidar(drv);

	vector<Point2f> output2D;
	vector<Point3f> output3D;
	readLidarProjectedImage(zed,drv,output2D, output3D,1);


	sl::Mat image(zed.getResolution(), MAT_TYPE_8U_C4);
	sl::Mat imageColor(zed.getResolution(), MAT_TYPE_8U_C4);
	sl::Mat image_depth(zed.getResolution(), MAT_TYPE_8U_C4);
	sl::Mat depth(zed.getResolution(), MAT_TYPE_8U_C4);

	if (zed.grab() == SUCCESS) {
		// A new image and depth is available if grab() returns SUCCESS
		zed.retrieveImage(image, VIEW_LEFT_GRAY); // Retrieve left image
		zed.retrieveImage(imageColor,VIEW_LEFT);
		zed.retrieveImage(image_depth, VIEW_DEPTH); // Retrieve left image
	}

	cv::Mat image_ocv 			= slMat2cvMat(image);
	cv::Mat image_depth_ocv 	= slMat2cvMat(image_depth);
	cv::Mat imageColor_ocv 		= slMat2cvMat(imageColor);
		
	for (int i = 0; i < output2D.size(); i++)
	{
		if(output2D[i].x < 0 || output2D[i].x > zed.getResolution().width || output2D[i].y < 0 || output2D[i].y > zed.getResolution().height)
		{

		}
		else
		{		
			int csize = 2;
			//cv::Point_<int> point = Point(u_l[i], v_l[i]);
			cv::circle(imageColor_ocv, output2D[i], csize, Scalar(0, 0, 255), -1);

		}
	}
	cv::imshow("VIEW", imageColor_ocv);
 	cv::waitKey(0);
	
	
	/*Size patternsize(6, 9); 			//interior number of corners
	cv::Mat planeParams(4,1,CV_32F);
	vector<Point2f> p_corners;
	vector<Point3f> p_corners3D;
	getCheckerboardPlane(zed,patternsize,planeParams,p_corners3D,p_corners,true);
	


	drv->stopMotor(); 
	RPlidarDriver::DisposeDriver(drv);

	*/
	zed.close();

	return 0;
}
 
