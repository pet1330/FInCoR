#include <stdlib.h>
#include "include/CDump.h"
#include "include/CTimer.h"
#include "include/CCircleDetect.h"
#include "include/CTransformation.h"
#include "include/CChargingClient.h"
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/JointState.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <include/ChargingAction.h>
#include <scitos_teleop/action_buttons.h>
#include <actionlib/server/simple_action_server.h>
#include <sensor_msgs/Joy.h>
#include <scitos_msgs/BatteryState.h>
#include <move_base_msgs/MoveBaseAction.h>
#include "include/CChargingActions.h"


#define MAX_PATTERNS 10 

float ptuPan = 0.0;
float ptuTilt = -15.0;
bool success = false;
int failedToSpotStationCount=0;
scitos_docking::ChargingFeedback feedback;
scitos_docking::ChargingResult result;
char response[3000];
char posString[3000];
typedef actionlib::SimpleActionServer<scitos_docking::ChargingAction> Server;
typedef actionlib::SimpleActionServer<move_base_msgs::MoveBaseAction> DockingServer;

int maxMeasurements = 100;
float dockingPrecision = 0.10;
float realPrecision,tangle,tdistance;

int stationSpotted = 0;
bool calibrated = true;
CTimer timer;
int timeOut = 120;
int  defaultImageWidth= 320;
int  defaultImageHeight = 240;
float circleDiameter = 0.05;
float rotateBy = 0;
bool chargerDetected = false;
CChargingActions *robot;

ros::NodeHandle *nh;
Server *server;
DockingServer *dockingServer;
DockingServer *undockingServer;

image_transport::Publisher imdebug;

int onBattery = 0;
int maxFailures=60;
TLogModule module = LOG_MODULE_MAIN;
int numSaved = 0;
CRawImage *image;
int waitCycles = 0;
int ptupos = 0;
CCircleDetect *detectorArray[MAX_PATTERNS];
STrackedObject objectArray[MAX_PATTERNS];
SSegment currentSegmentArray[MAX_PATTERNS];
SSegment lastSegmentArray[MAX_PATTERNS];
CTransformation *trans;
CChargingClient chargingClient;
bool positionUpdate = false;
bool headRestart = false;

//position injection related
EState state = STATE_ROTATE;
EState lastState = STATE_IDLE;

void cameraInfoCallBack(const sensor_msgs::CameraInfo &msg)
{
	trans->updateParams(msg.K[2],msg.K[5],msg.K[0],msg.K[4]);
}

int initComponents()
{
	image->getSaveNumber();
	calibrated = trans->loadCalibration();
}

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
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
	
void mainLoop()
{
	char status[1000];
	int a = 0;
	while (ros::ok()){
		if (timeOut < timer.getTime() && state != STATE_IDLE ) state = STATE_TIMEOUT;
		if (state != STATE_IDLE && state != STATE_HEAD_ON) robot->moveHead();
		ros::spinOnce();
		if (positionUpdate && robot->poseSet == false) robot->injectPosition(); 
		usleep(30000);
		if (state!=lastState){
			sprintf(status,"Charging service is %s",stateStr[state]);
			feedback.Progress = (int)robot->progress;
			feedback.Message = stateStr[state];
			if (server->isActive()) server->publishFeedback(feedback);
		}
		if (server->isActive()||undockingServer->isActive()||dockingServer->isActive()){
			feedback.Message = stateStr[state];
			feedback.Progress = (int)robot->progress;
			if (robot->actionStuck()) feedback.Level = 1; else feedback.Level = 0;
		}
		if (((server->isPreemptRequested() && server->isActive())  || (undockingServer->isActive() && undockingServer->isPreemptRequested())|| (dockingServer->isActive() && dockingServer->isPreemptRequested()))&& state != STATE_IDLE){
			state = STATE_PREEMPTED;
			robot->halt();
			ros::spinOnce();
			if (server->isActive()||undockingServer->isActive()||dockingServer->isActive()) result.Message = "Current action preempted by external request.";
		}
		state_cleanup();
		lastState = state;
	}
}

int main(int argc,char* argv[])
{
	ros::init(argc, argv, "charging");
	nh = new ros::NodeHandle;
	robot = new CChargingActions(nh);
	image_transport::ImageTransport it(*nh);

	dump = new CDump(NULL,256,1000000);
	image = new CRawImage(defaultImageWidth,defaultImageHeight,4);
	trans = new CTransformation(circleDiameter,nh);
	for (int i = 0;i<MAX_PATTERNS;i++) detectorArray[i] = new CCircleDetect(defaultImageWidth,defaultImageHeight);

	initComponents();
	success = false;
	image_transport::Subscriber subim = it.subscribe("head_xtion/rgb/image_mono", 1, imageCallback);
	image_transport::Subscriber subdepth = it.subscribe("head_xtion/depth/image_rect", 1, depthCallback);
	nh->param("positionUpdate",positionUpdate,false);
        imdebug = it.advertise("/charging/processedimage", 1);
	ros::Subscriber subodo = nh->subscribe("odom", 1, odomCallback);
	ros::Subscriber subcharger = nh->subscribe("battery_state", 1, batteryCallBack);
	ros::Subscriber subcamera = nh->subscribe("head_xtion/rgb/camera_info", 1,cameraInfoCallBack);
	ros::Subscriber joy_sub_ = nh->subscribe("/teleop_joystick/action_buttons", 10, joyCallback);
	ros::Subscriber ptu_sub_ = nh->subscribe("/ptu/state", 10, ptuCallback);
	ros::Subscriber robot_pose = nh->subscribe("/robot_pose", 1000, poseCallback);
	server = new Server(*nh, "chargingServer", boost::bind(&actionServerCallback, _1, server), false);
	dockingServer = new DockingServer(*nh, "docking", boost::bind(&dockingServerCallback, _1, dockingServer), false);
	undockingServer = new DockingServer(*nh, "undocking", boost::bind(&undockingServerCallback, _1, undockingServer), false);

	server->start();
	dockingServer->start();
	undockingServer->start();
	ROS_DEBUG("Server running");
	while (ros::ok()) mainLoop();
	delete image;
	for (int i = 0;i<MAX_PATTERNS;i++) delete detectorArray[i];
	delete trans;
	return 0;
}
