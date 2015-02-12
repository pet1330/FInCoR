#include <stdlib.h>
//#include "scitos_docking/CDump.h"
//#include "scitos_docking/CTimer.h"
#include "scitos_docking/CCircleDetect.h"
#include "scitos_docking/CTransformation.h"
//#include "scitos_docking/CChargingClient.h"
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
//#include <sensor_msgs/JointState.h>
//#include <geometry_msgs/Twist.h>
//#include <nav_msgs/Odometry.h>
//#include <actionlib/server/simple_action_server.h>
//#include <sensor_msgs/Joy.h>
//#include <move_base_msgs/MoveBaseAction.h>
//#include "scitos_docking/CChargingActions.h"


#define MAX_PATTERNS 100

float ptuPan = 0.0;
float ptuTilt = -15.0;
bool success = false;
int failedToSpotStationCount=0;
//scitos_docking::ChargingFeedback feedback;
//scitos_docking::ChargingResult result;
char response[3000];
char posString[3000];

int maxMeasurements = 100;
float dockingPrecision = 0.10;
float realPrecision,tangle,tdistance;

int stationSpotted = 0;
bool calibrated = true;
//CTimer timer;
int timeOut = 120;
int  defaultImageWidth= 320;
int  defaultImageHeight = 240;
float circleDiameter = 0.05;
float rotateBy = 0;
bool chargerDetected = false;

ros::NodeHandle *nh;
image_transport::Publisher imdebug;

int onBattery = 0;
int maxFailures=60;
//TLogModule module = LOG_MODULE_MAIN;
int numSaved = 0;
CRawImage *image;
int waitCycles = 0;
int ptupos = 0;
CCircleDetect *detectorArray[MAX_PATTERNS];
STrackedObject objectArray[MAX_PATTERNS];
SSegment currentSegmentArray[MAX_PATTERNS];
SSegment lastSegmentArray[MAX_PATTERNS];
CTransformation *trans;
//CChargingClient chargingClient;
bool positionUpdate = false;
bool headRestart = false;

//position injection related
//EState state = STATE_ROTATE;
//EState lastState = STATE_IDLE;

void cameraInfoCallBack(const sensor_msgs::CameraInfo &msg)
{
	trans->updateParams(msg.K[2],msg.K[5],msg.K[0],msg.K[4]);
}

int initComponents()
{
	failedToSpotStationCount = 0;
	image->getSaveNumber();
	calibrated = trans->loadCalibration();
}

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
    printf("%s\n","image_receaved");
		if (image->bpp != msg->step/msg->width || image->width != msg->width || image->height != msg->height){
			delete image;
			ROS_DEBUG("Readjusting image format from %ix%i %ibpp, to %ix%i %ibpp.",image->width,image->height,image->bpp,msg->width,msg->height,msg->step/msg->width);
			image = new CRawImage(msg->width,msg->height,msg->step/msg->width);
		}
		memcpy(image->data,(void*)&msg->data[0],msg->step*msg->height);

		//search image for circles
		for (int i = 0;i<3;i++){
			lastSegmentArray[i] = currentSegmentArray[i];
			currentSegmentArray[i] = detectorArray[i]->findSegment(image,lastSegmentArray[i]);
			objectArray[i].valid = false;
			if (currentSegmentArray[i].valid)objectArray[i] = trans->transform(currentSegmentArray[i]);
		}
		//and publish the result
		memcpy((void*)&msg->data[0],image->data,msg->step*msg->height);
		imdebug.publish(msg);
}

int main(int argc,char* argv[])
{
	ros::init(argc, argv, "charging");
	nh = new ros::NodeHandle;
	image_transport::ImageTransport it(*nh);
    //dump = new CDump(NULL,256,1000000);
	image = new CRawImage(defaultImageWidth,defaultImageHeight,4);
	trans = new CTransformation(circleDiameter,nh);
	for (int i = 0;i<MAX_PATTERNS;i++) detectorArray[i] = new CCircleDetect(defaultImageWidth,defaultImageHeight);

	initComponents();
	success = false;
    image_transport::Subscriber subim = it.subscribe("/left/rgb/image_mono", 1, imageCallback);
    imdebug = it.advertise("/charging/processedimage", 1);
    //ros::Subscriber subcamera = nh->subscribe("/left/rgb/camera_info", 1,cameraInfoCallBack);
    ROS_DEBUG("Server running");
    ros::spin();
    //while (ros::ok())
    //{
    //    printf("%s\n","ros running");
    //}
    delete image;
	for (int i = 0;i<MAX_PATTERNS;i++) delete detectorArray[i];
	delete trans;
	return 0;
}
