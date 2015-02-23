#include "CCircleDetect.h"
#include "CTransformation.h"
#include "ros/ros.h"
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <string>

#define MAX_PATTERNS 15

bool calibrated = true;

int defaultImageWidth = 640;
int defaultImageHeight = 480;
//float circleDiameter = 0.037;
float circleDiameter = 0.0475;
float rotateBy = 0;

ros::NodeHandle *nh;
image_transport::Publisher imdebug;

CRawImage *image;

CCircleDetect *detectorArray[MAX_PATTERNS];
STrackedObject objectArray[MAX_PATTERNS];
STrackedObject objectArray3D[MAX_PATTERNS];
SSegment currentSegmentArray[MAX_PATTERNS];
SSegment lastSegmentArray[MAX_PATTERNS];
CTransformation *trans;

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

    std::string topic;

    if (argc != 2) {
        std::cout << argc << std::endl;
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

    ros::init(argc, argv, "circle_detector");
    nh = new ros::NodeHandle;
    image_transport::ImageTransport it(*nh);
    ros::Subscriber subcamera = nh->subscribe("/" + topic + "/rgb/camera_info", 1, cameraInfoCallBack);
    image = new CRawImage(defaultImageWidth, defaultImageHeight, 4);
    trans = new CTransformation(circleDiameter, nh);
    for (int i = 0; i < MAX_PATTERNS; i++) {
        detectorArray[i] = new CCircleDetect(defaultImageWidth, defaultImageHeight);
    }

    image->getSaveNumber();
    calibrated = trans->loadCalibration();
    image_transport::Subscriber subim = it.subscribe("/" + topic + "/rgb/image_mono", 1, imageCallback);
    imdebug = it.advertise("/circledetection/" + topic + "/rgb/processedimage", 1);
    ROS_DEBUG("Server running");
    ros::spin();
    delete image;
    for (int i = 0; i < MAX_PATTERNS; i++) delete detectorArray[i];
    delete trans;
    return 0;
}
