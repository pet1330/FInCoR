#include "CCircleDetect.h"
#include "CTransformation.h"
#include "ros/ros.h"
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <geometry_msgs/Pose.h>
#include <tf/LinearMath/QuadWord.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/Marker.h>
#include <tf/transform_broadcaster.h>

#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>


#define MAX_PATTERNS 10

int defaultImageWidth = 640;
int defaultImageHeight = 480;
//float circleDiameter = 0.037;
float circleDiameter = 0.0475;
float rotateBy = 0;

ros::NodeHandle *nh;
image_transport::Publisher imdebug;
tf::TransformListener* lookup;

ros::Publisher pubPose;
ros::Publisher vis_pub;

std::clock_t start;

std::string topic;

CRawImage *image;

CCircleDetect *detectorArray[MAX_PATTERNS];
STrackedObject objectArray[MAX_PATTERNS];
STrackedObject objectArray3D[MAX_PATTERNS];
SSegment currentSegmentArray[MAX_PATTERNS];
SSegment lastSegmentArray[MAX_PATTERNS];
CTransformation *trans;

void publishRVizMarker(const geometry_msgs::PoseStamped cp) {
    visualization_msgs::Marker marker;
    marker.header = cp.header;
    marker.id = 0;
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose = cp.pose;
    marker.scale.x = 0.1;
    marker.scale.y = 0.1;
    marker.scale.z = 1.0;
    marker.color.a = 1.0;
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    vis_pub.publish(marker);

}

std::string itemLookUp()
{


}

void imageCallback(const sensor_msgs::ImageConstPtr& msg) {
    if (image->bpp != msg->step / msg->width || image->width != msg->width || image->height != msg->height) {
        delete image;
        ROS_DEBUG("Readjusting image format from %ix%i %ibpp, to %ix%i %ibpp.", image->width, image->height, image->bpp, msg->width, msg->height, msg->step / msg->width);
        image = new CRawImage(msg->width, msg->height, msg->step / msg->width);
    }

    memcpy(image->data, (void*) &msg->data[0], msg->step * msg->height);

    //search image for circles
    for (int i = 0; i < MAX_PATTERNS; i++) {
        lastSegmentArray[i] = currentSegmentArray[i];
        currentSegmentArray[i] = detectorArray[i]->findSegment(image, lastSegmentArray[i]);
        objectArray[i].valid = false;
        if (currentSegmentArray[i].valid) {
            objectArray[i] = trans->transform(currentSegmentArray[i]);
            printf("Image Points:  X:%f  Y:%f Z: %f ratio: %f\n", objectArray[i].x, objectArray[i].y, objectArray[i].z, objectArray[i].bwratio);

            geometry_msgs::PoseStamped circlePose;
            circlePose.header.stamp = ros::Time::now();
            circlePose.header.frame_id = "/" + topic + "/" + topic + "_rgb_optical_frame";
            circlePose.pose.position.x = -objectArray[i].y;
            circlePose.pose.position.y = -objectArray[i].z;
            circlePose.pose.position.z = objectArray[i].x;
            tf::Quaternion q;
            q.setRPY(objectArray[i].roll, objectArray[i].pitch, objectArray[i].yaw);
            circlePose.pose.orientation.x = q.getX();
            circlePose.pose.orientation.y = q.getY();
            circlePose.pose.orientation.z = q.getZ();
            circlePose.pose.orientation.w = q.getW();
            
            try {
                lookup->waitForTransform("base", circlePose.header.frame_id, circlePose.header.stamp, ros::Duration(10));
                lookup->transformPose("/base", circlePose, circlePose);
            } catch (tf::TransformException ex) {
                ROS_WARN("%s\n", ex.what());
            }
            printf("X:%f  Y:%f Z: %f\n", circlePose.pose.position.x, circlePose.pose.position.y, circlePose.pose.position.z);
            
            publishRVizMarker(circlePose);
            
            pubPose.publish(circlePose);
        }
    }
    printf("%s\n", "--------------------");
    //and publish the result
    memcpy((void*) &msg->data[0], image->data, msg->step * msg->height);
    imdebug.publish(msg);
}

void cameraInfoCallBack(const sensor_msgs::CameraInfo &msg) {
    trans->updateParams(msg.K[2], msg.K[5], msg.K[0], msg.K[4]);
}

void depthCallback(const sensor_msgs::ImageConstPtr& msg) {

}

int main(int argc, char* argv[]) {

    ros::init(argc, argv, "circle_detector", ros::init_options::AnonymousName);
    nh = new ros::NodeHandle("~");

    if (nh->getParam("camera", topic)) {
        std::transform(topic.begin(), topic.end(), topic.begin(), ::tolower);
    } else {
        if (argc != 2) {
            std::cerr << "Please supply whether you are subscribing to the left or right camera (R/L)" << std::endl;
            return -1;
        } else {
            if (argv[1][0] == 'l' || argv[1][0] == 'L') {
                topic = "left";
            } else if (argv[1][0] == 'r' || argv[1][0] == 'R') {
                topic = "right";
            } else {
                std::cerr << "Please supply whether you are subscribing to the left or right camera (R/L)" << std::endl;
                return -1;
            }
        }
    }

    image_transport::ImageTransport it(*nh);
    ros::Subscriber subcamera = nh->subscribe("/" + topic + "/rgb/camera_info", 1, cameraInfoCallBack);
    image = new CRawImage(defaultImageWidth, defaultImageHeight, 4);
    trans = new CTransformation(circleDiameter, nh);
    for (int i = 0; i < MAX_PATTERNS; i++) {
        detectorArray[i] = new CCircleDetect(defaultImageWidth, defaultImageHeight);
    }

    image->getSaveNumber();
    image_transport::Subscriber subim = it.subscribe("/" + topic + "/rgb/image_mono", 1, imageCallback);
    imdebug = it.advertise("/circledetection/" + topic + "/rgb/processedimage", 1);
    pubPose = nh->advertise<geometry_msgs::PoseStamped>("/circledetection/pose", 0);
    vis_pub = nh->advertise<visualization_msgs::Marker>("circledetection/rviz_marker", 0);
    lookup = new tf::TransformListener();

    ROS_DEBUG("Server running");
    ros::spin();
    delete image;
    for (int i = 0; i < MAX_PATTERNS; i++) delete detectorArray[i];
    delete trans;
    return 0;
}
