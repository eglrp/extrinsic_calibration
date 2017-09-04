
#include <myFunctions.h>


// Autor:			Michael Lüthy
// Beschreibung:	Funktionsdefinitionen

void getCheckerboardPlane(Camera &zed,Size &patternsize,cv::Mat &planeParameters, vector<Point3f> &p_corners3D,vector<Point2f> &p_corners, bool drawNormal)
{
	sl::Mat image(zed.getResolution(), MAT_TYPE_8U_C4);
	sl::Mat imageColor(zed.getResolution(), MAT_TYPE_8U_C4);
	sl::Mat image_depth(zed.getResolution(), MAT_TYPE_8U_C4);
	sl::Mat depth(zed.getResolution(), MAT_TYPE_8U_C4);
	//sl::Mat confidence_map(zed.getResolution(), MAT_TYPE_8U_C4);

	// Initialisieren der Datenstruktur für finden der Ebenenparameter
	cv::Mat A = cv::Mat(63,4,CV_32F);

	if (zed.grab() == SUCCESS) {
		// A new image and depth is available if grab() returns SUCCESS
		zed.retrieveImage(image, VIEW_LEFT_GRAY); // Retrieve left image
		zed.retrieveImage(imageColor,VIEW_LEFT);
		zed.retrieveImage(image_depth, VIEW_DEPTH); // Retrieve left image
	}

	cv::Mat image_ocv 			= slMat2cvMat(image);
	cv::Mat image_depth_ocv 	= slMat2cvMat(image_depth);
	cv::Mat imageColor_ocv 		= slMat2cvMat(imageColor);
	
	imshow("View", imageColor_ocv);
	cv::waitKey(0);

	//Checkerboard suchen
	bool patternfound = findChessboardCorners(image_ocv, patternsize, p_corners);
	if (patternfound)
	{
		cornerSubPix(image_ocv, p_corners, Size(11, 11), Size(-1, -1), TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1));
		printf("Checkerboard:\t\t\t gefunden\n");
		printf("Gefundene Ecken:\t\t %lu von %i\n", p_corners.size(),patternsize.width*patternsize.height);
		
		//Extrahieren der 3D-Punkte des Patterns	
 		sl::Mat point_cloud;
		sl::float4 point3D;

		zed.retrieveMeasure(depth, MEASURE_DEPTH); // Retrieve depth
		zed.retrieveMeasure(point_cloud, MEASURE_XYZRGBA);
		//zed.retrieveMeasure(confidence_map, MEASURE_CONFIDENCE);

		for (int i = 0; i<p_corners.size(); i++)
		{

			//confidence_map.getValue(corners[i].x,corners[i].y, &pointConf);
			// Array befüllen
			point_cloud.getValue(p_corners[i].x,p_corners[i].y,&point3D);
			p_corners3D.push_back(Point3f(point3D.x,point3D.y,point3D.z));
			A.at<float>(i,0) = point3D.x;
			A.at<float>(i,1) = point3D.y;
			A.at<float>(i,2) = point3D.z;
			A.at<float>(i,3) = 1;

		}
		// Kamera Kalibration holen
		cv::SVD::solveZ(A,planeParameters);
		printf("Ebenen Parameter: \t\t a: %f, b: %f, c: %f, d: %f\n",planeParameters.at<float>(0),planeParameters.at<float>(1),planeParameters.at<float>(2),planeParameters.at<float>(3));
		
		if(drawNormal)
		{
			float a = planeParameters.at<float>(0);
			float b = planeParameters.at<float>(1);
			float c = planeParameters.at<float>(2);
			float d = planeParameters.at<float>(3);

			vector<Point3f> axisX;
			axisX.push_back	(Point3f(A.at<float>(1,0),A.at<float>(1,1),A.at<float>(1,2)));
			axisX.push_back(Point3f(A.at<float>(2,0),A.at<float>(2,1),A.at<float>(2,2)));

			vector<Point3f> axisY;
			axisY.push_back(Point3f(A.at<float>(1,0),A.at<float>(1,1),A.at<float>(1,2)));
			axisY.push_back(Point3f(A.at<float>(7,0),A.at<float>(7,1),A.at<float>(7,2)));

			vector<Point3f> axisZ;
			axisZ.push_back(Point3f(A.at<float>(1,0),A.at<float>(1,1),A.at<float>(1,2)));
			axisZ.push_back(Point3f((A.at<float>(1,0)+a/10),(A.at<float>(1,1)+b/10),(A.at<float>(1,2)+c/10)));


			cv::Mat tvec  = cv::Mat::zeros(3,1,CV_32F);
			cv::Mat rvec  = cv::Mat::zeros(3,1,CV_32F);

			vector<Point2f> axisX2D;
			axisX2D.push_back(Point2f(0,0));
			axisX2D.push_back(Point2f(0,0));

			vector<Point2f> axisY2D;
			axisY2D.push_back(Point2f(0,0));
			axisY2D.push_back(Point2f(0,0));

			vector<Point2f> axisZ2D;
			axisZ2D.push_back(Point2f(0,0));
			axisZ2D.push_back(Point2f(0,0));

			cv::Mat K;
			cv::Mat dist;
			getCameraMatrix(zed,K,dist);
			projectPoints(axisX,rvec,tvec,K,dist,axisX2D);
			projectPoints(axisY,rvec,tvec,K,dist,axisY2D);
			projectPoints(axisZ,rvec,tvec,K,dist,axisZ2D);
			cout << axisX << endl;
			cout << axisY << endl;
			cout << axisZ << endl;
			cout << axisZ2D << endl;


			drawChessboardCorners(image_depth_ocv, patternsize, cv::Mat(p_corners), patternfound);
			drawChessboardCorners(imageColor_ocv, patternsize, cv::Mat(p_corners), patternfound);

			arrowedLine(imageColor_ocv,axisX2D[0],axisX2D[1],Scalar(0,0,255),5);
			arrowedLine(imageColor_ocv,axisY2D[0],axisY2D[1],Scalar(0,255,0),5);
			arrowedLine(imageColor_ocv,axisZ2D[0],axisZ2D[1],Scalar(255,0,0),5);


			//imshow("Depth", image_depth_ocv);
			imshow("View", imageColor_ocv);
			cv::waitKey(0);
		}
	}
	else
	{
		cout << "Checkerboard nicht gefunden" << endl;
		cout << "Es kann keine Lösung gesucht werden" << endl;
	}
}

void writeCSV(string filename, cv::Mat m)
{
	ofstream myfile;
	myfile.open(filename.c_str());
	myfile << cv::format(m, "CSV") << std::endl;
	myfile.close();
}

void loadFromCSV(const string& values, int opencv_type,cv::Mat &m)
{
	stringstream ss(values);
	string line;
	while (getline(ss, line))
	{
		vector<double> dvals;

		stringstream ssline(line);
		string val;
		while (getline(ssline, val, ','))
		{
			dvals.push_back(stod(val));
		}

		cv::Mat mline(dvals, true);
		transpose(mline, mline);

		m.push_back(mline);
	}

	int ch = CV_MAT_CN(opencv_type);

	m = m.reshape(ch);
	m.convertTo(m, opencv_type);
}

void calcShowBRISK(cv::Mat image)
{
	vector <KeyPoint> kpts;
	cv::Mat desc;

	int threshl = 30;
	int octaves = 3;
	float patternScale = 1.0f;

	cv::BRISK BRISKD(threshl,octaves,patternScale);
	BRISKD.detect(image, kpts, desc);
	BRISKD.compute(image,kpts,desc);
	cv::Mat imWiKpts;

	cv::drawKeypoints(image, kpts, imWiKpts);
	cv::imshow("Keypoints", imWiKpts);
	waitKey(0);
	cv::destroyWindow("Keypoints");
}

void readLidarPolar(RPlidarDriver * drv, vector<Point2f> &output,int n_cycles)
{
	vector<Point2f> lidartemp;
	vector<double> d_l;
	vector<double> phi_l;


	for (int j = 0; j < n_cycles; j++)
	{
		grabLidarDataBasic(drv, d_l, phi_l);
		lidartemp.resize(d_l.size());
		for (int i = 0; i < d_l.size(); i++)
		{
			lidartemp[i].x = d_l[i];
			lidartemp[i].y = phi_l[i];
		}
		output.insert(output.end(), lidartemp.begin(), lidartemp.end());

	}
}

void polar2cartesian(vector<Point2f> &input,vector<Point3f> &output)
{
	output.resize(input.size());
	for (int idx = 0; idx<output.size(); idx++)
	{
		output[idx].x = input[idx].x / 1000 * sin(input[idx].y * PI / 180.0);
		output[idx].y = 0;
		output[idx].z = input[idx].x / 1000 * cos(input[idx].y * PI / 180.0);
	}
}

void readLidarCart(RPlidarDriver * drv, vector<Point3f> &output,int n_cycles)
{
	vector<Point2f> tempdata;
	readLidarPolar(drv,tempdata,n_cycles);
	polar2cartesian(tempdata,output);
}

void readLidarProjectedImage(Camera &zed, RPlidarDriver * drv,vector<Point2f> &output2D,vector<Point3f> &output3D, int n_cycles)
{
// Get Camera Calibration
	cv::Mat K(3,4,CV_64F);
	cv::Mat R(3,3,CV_64F);
	cv::Mat t(3,1,CV_64F);
	cv::Mat dist(5,1,CV_64F);
	cv::Mat P(3,4,CV_64F);
	cv::Mat Pi(3,4,CV_64F);

	getCameraMatrix(zed,K,dist);
	getProjectionMatrices(zed,R,t);


	cv::Mat rvec(3,1,CV_64F);
	Rodrigues(R,rvec);
	cout << rvec << endl,

	readLidarCart(drv, output3D, n_cycles);


	output2D.resize(output3D.size());
	projectPoints(output3D,rvec,t,K,dist,output2D);



	int resu 	= zed.getResolution().width;
	int resv 	= zed.getResolution().height;
	for(int i=0;i<output2D.size();i++)
	{
		if(output3D[i].z <0.5 || output3D[i].z>2)
		{
			output2D[i].x = -1;
			output2D[i].y = -1;
		}
	}

}

void getLidarInfo(RPlidarDriver * drv, const char * opt_com_path, _u32 opt_com_baudrate, bool &support_express)
{
	//bool support_express;
	bool support_motorctr;
	u_result op_result_devinfo;
	u_result op_result_expresscan;
	u_result op_result_motorctr;

	//RPlidarDriver * drv = RPlidarDriver::CreateDriver(RPlidarDriver::DRIVER_TYPE_SERIALPORT);

	// retrieving the device info
	rplidar_response_device_info_t devinfo;
	op_result_devinfo = drv->getDeviceInfo(devinfo);
	op_result_expresscan = drv->checkExpressScanSupported(support_express);
	op_result_motorctr = drv->checkMotorCtrlSupport(support_motorctr);

	if (IS_FAIL(op_result_devinfo) || IS_FAIL(op_result_expresscan)) {
		fprintf(stderr, "Fehler, kann Geräteinformation nicht holen.\n");
		exit(-2);
	}


	// print out the device serial number, firmware and hardware version number..
	printf("RPLIDAR S/N: ");
	for (int pos = 0; pos < 16; ++pos) {
		printf("%02X", devinfo.serialnum[pos]);
	}

	printf("\n"
		"Firmware Ver: %d.%02d\n"
		"Hardware Rev: %d\n"
		, devinfo.firmware_version >> 8
		, devinfo.firmware_version & 0xFF
		, (int)devinfo.hardware_version);

	//cout << "express scan: " << support << endl;
	printf("Express Scan Support: %s\n", support_express ? "ja" : "nein");
	printf("Motor Ctrl Support: %s\n", support_motorctr ? "ja" : "nein");
	// print out the device serial number, firmware and hardware version number..
	printf("RPLIDAR S/N: ");
	for (int pos = 0; pos < 16; ++pos) {
		printf("%02X", devinfo.serialnum[pos]);
	}
	printf("\n\n");
}

void connectLidar(RPlidarDriver * drv, const char * opt_com_path, _u32 opt_com_baudrate)
{
	if (!drv) {
		fprintf(stderr, "Ungenügend Speicher\n");
		exit(-2);
	}

	// make connection...
	if (IS_FAIL(drv->connect(opt_com_path, opt_com_baudrate))) {
		fprintf(stderr, "Fehler, kann ang. Port nicht binden %s.\n", opt_com_path);
		exit(-2);
	}
}

void startLidar(RPlidarDriver * drv)
{
	const char * opt_com_path = "/dev/ttyUSB0"; // Surface
	_u32         opt_com_baudrate = 115200;
	_u16 f_t = 10; // Scanning Frequency in Hz
	int pwm = 550; // pwm signal
	float frequency = INFINITY;
	bool in4kMode;
	bool support_express;
	int  n_nodes; //dynamische Variable, welche Anzahl der tatsächlich aufgenommenen Messpunkte enthält.
	vector<double> d;
	vector<double> phi;
	vector<int> startFlagBit;
	
	vector<double> scanQuality;
	vector<double> pos;
	
	connectLidar(drv, opt_com_path, opt_com_baudrate);
	getLidarInfo(drv, opt_com_path, opt_com_baudrate, support_express);
																																																															
	drv->startMotor();
	drv->setMotorPWM(pwm);
	drv->startScan(); //Scanning mit immer gleichem Winkelabständen

	while (frequency <= f_t || frequency == INFINITY) {

		grabLidarDataComplete(drv, d, phi, startFlagBit, scanQuality, frequency, support_express, in4kMode);
		printf("Lidar startet, Frequenz: %f\r", frequency);
	}
	printf("Lidar Bereit, Frequenz: %f\n", frequency);
	grabLidarDataComplete(drv, d, phi, startFlagBit, scanQuality, frequency, support_express, in4kMode);
}

void grabLidarDataComplete(RPlidarDriver * drv, vector<double> &distance, vector<double> &angle, vector<int> &startFlagBit, vector<double> &scanQuality, float &frequency, bool &support_express, bool &in4kMode)
{
	rplidar_response_measurement_node_t nodes[720]; // initialisiert nodes mit maximal zu erwartender Anzahl Messungen pro Umdrehung
	u_result     						op_result_scan;									//?Buffer für Lidardaten?
	size_t   							count = _countof(nodes);						// enthält Anzahl geholter Messungen von Lidar
	op_result_scan = drv->grabScanData(nodes, count);									// holt Daten von Lidar
	op_result_scan = drv->getFrequency(support_express, count, frequency, in4kMode);	// gibt Scanfrequenz und 4k-Modus-Status aus

																						//Anpassen der Vektorgrössen entsprechend der Anzahl Messungen (count)
	distance.resize(count);
	angle.resize(count);
	startFlagBit.resize(count);
	scanQuality.resize(count);

	if (IS_OK(op_result_scan)) {
		drv->ascendScanData(nodes, count);
		for (int pos = 0; pos < (int)count; ++pos) {
			distance.at(pos) = nodes[pos].distance_q2 / 4.0f;
			angle.at(pos) = (nodes[pos].angle_q6_checkbit >> RPLIDAR_RESP_MEASUREMENT_ANGLE_SHIFT) / 64.0f;
			startFlagBit.at(pos) = nodes[pos].sync_quality & RPLIDAR_RESP_MEASUREMENT_SYNCBIT;
			scanQuality.at(pos) = nodes[pos].sync_quality >> RPLIDAR_RESP_MEASUREMENT_QUALITY_SHIFT;
		}
	}
}

void grabLidarDataBasic(RPlidarDriver * drv, vector<double> &distance, vector<double> &angle)
{
	rplidar_response_measurement_node_t nodes[720]; // initialisiert nodes mit maximal zu erwartender Anzahl Messungen pro Umdrehung
	u_result     						op_result_scan;									//?Buffer für Lidardaten?
	size_t   							count = _countof(nodes);						// enthält Anzahl geholter Messungen von Lidar
	op_result_scan = drv->grabScanData(nodes, count);									// holt Daten von Lidar

																						//Anpassen der Vektorgrössen entsprechend der Anzahl Messungen (count)
	distance.resize(count);
	angle.resize(count);

	if (IS_OK(op_result_scan)) {
		drv->ascendScanData(nodes, count);
		for (int pos = 0; pos < (int)count; ++pos) {
			distance.at(pos) = nodes[pos].distance_q2 / 4.0f;
			angle.at(pos) = (nodes[pos].angle_q6_checkbit >> RPLIDAR_RESP_MEASUREMENT_ANGLE_SHIFT) / 64.0f;
		}
	}
}

void zedTracking(Camera &zed, Pose &zed_pose)
{
	if (zed.grab() == SUCCESS) {
		// Get the pose of the camera relative to the world frame
		TRACKING_STATE state = zed.getPosition(zed_pose, REFERENCE_FRAME_WORLD);
		if (TRACKING_STATE_OK)
		{
			// Display translation and timestamp
			printf("	Translation: tx: %.3f, ty:  %.3f, tz:  %.3f, Orientation: ox: %.3f, oy:  %.3f, oz:  %.3f, ow: %.3f, Zeit: %llu\r",
				zed_pose.getTranslation().tx, zed_pose.getTranslation().ty, zed_pose.getTranslation().tz, zed_pose.getOrientation().ox, zed_pose.getOrientation().oy, zed_pose.getOrientation().oz, zed_pose.getOrientation().ow, zed_pose.timestamp);
			// Display orientation quaternion
			//printf("Orientation: ox: %.3f, oy:  %.3f, oz:  %.3f, ow: %.3f\r",
			//zed_pose.getOrientation().ox, zed_pose.getOrientation().oy, zed_pose.getOrientation().oz, zed_pose.getOrientation().ow);
		}
	}
	delay(50);
}

void setCamera(Camera &zed) {
	InitParameters init_params;
	RuntimeParameters runtime_params;
	TrackingParameters tracking_params;
	init_params.camera_resolution = RESOLUTION_HD2K;	// Use HD1080 video mode
	init_params.camera_fps = 30;					// Set fps at 30
	init_params.depth_mode = DEPTH_MODE_QUALITY;	// options: _QUALITY,MEDIUM,PERFORMANCE
	init_params.coordinate_units = UNIT_METER;		// Use millimeter units (for depth measurements)
	init_params.depth_minimum_distance = 0.3;					// Set the minimum depth perception distance 300mm,
	init_params.coordinate_system = COORDINATE_SYSTEM_IMAGE;															//init_params.coordinate_system = COORDINATE_SYSTEM_LEFT_HANDED_Y_UP; // Use a right-handed Y-up coordinate syste

	runtime_params.sensing_mode = SENSING_MODE_STANDARD; // Standard: NAN -> Occlusion, -Inf -> too close, Inf -> too far

														 // Open the camera
	ERROR_CODE err = zed.open(init_params);
	if (err != SUCCESS)
	{
		printf("Öffne Kamera:\t\t\t gescheitert\n");
		exit(-1);
	}
	if (zed.grab(runtime_params) == SUCCESS)
	{
		printf("Öffne Kamera:\t\t\t erfolgreich\n");
	}
	/*err = zed.enableTracking(tracking_params);
	if (err != SUCCESS)
	{
	printf("enabling tracking: failed\n");
	exit(-1);
	}
	*/
}

void getCameraMatrix(Camera &zed, cv::Mat& K, cv::Mat& dist)
{
	// Get Camera Calibration
	CalibrationParameters calibration_params = zed.getCameraInformation().calibration_parameters;
	double Ktemp[9] = { calibration_params.left_cam.fx,0,calibration_params.left_cam.cx,0, calibration_params.left_cam.fy, calibration_params.left_cam.cy,0.0,0.0,1.0 };
	double dist_l[5] = { calibration_params.left_cam.disto[0],calibration_params.left_cam.disto[1] ,calibration_params.left_cam.disto[2] ,calibration_params.left_cam.disto[3] ,calibration_params.left_cam.disto[4] };

	K 		= cv::Mat(3, 3, CV_64FC1, &Ktemp).clone();
	dist   	= cv::Mat(5, 1, CV_64FC1, &dist_l).clone();
}

void getProjectionMatrices(Camera &zed,cv::Mat &R, cv::Mat &t)
{
	cv::Mat K;
	cv::Mat dist;
	getCameraMatrix(zed,K,dist);

	double phi_lz 	= 2.75 * PI / 180.0;
	double Rtemp[9] = { 1,0,0,0,cos(phi_lz),-sin(phi_lz),0,sin(phi_lz),cos(phi_lz) };	// Rotation vom Lidar-KS zum Kamera-LS
	double Ctemp[3] = {-0.08000,-0.01600,-0.02200 };	// Lidar-Mittelpunkt aus Sicht der linken Kamera-Linse (entspricht bereits -Ctilde gemäss Formel)

	cv::Mat C(3,1,CV_64F,Ctemp);
	
	R = cv::Mat(3, 3, CV_64F, Rtemp).clone();
	t = cv::Mat(3,1,CV_64F);

	gemm(R, C, 1, noArray(), 0, t, 0);
}

cv::Mat slMat2cvMat(sl::Mat& input) {
	//convert MAT_TYPE to CV_TYPE
	int cv_type = -1;
	switch (input.getDataType()) {
	case sl::MAT_TYPE_32F_C1: cv_type = CV_32FC1; break;
	case sl::MAT_TYPE_32F_C2: cv_type = CV_32FC2; break;
	case sl::MAT_TYPE_32F_C3: cv_type = CV_32FC3; break;
	case sl::MAT_TYPE_32F_C4: cv_type = CV_32FC4; break;
	case sl::MAT_TYPE_8U_C1: cv_type = CV_8UC1; break;
	case sl::MAT_TYPE_8U_C2: cv_type = CV_8UC2; break;
	case sl::MAT_TYPE_8U_C3: cv_type = CV_8UC3; break;
	case sl::MAT_TYPE_8U_C4: cv_type = CV_8UC4; break;
	default: break;
	}

	// cv::Mat data requires a uchar* pointer. Therefore, we get the uchar1 pointer from sl::Mat (getPtr<T>())
	//cv::Mat and sl::Mat will share the same memory pointer
	return cv::Mat(input.getHeight(), input.getWidth(), cv_type, input.getPtr<sl::uchar1>(MEM_CPU));
}
