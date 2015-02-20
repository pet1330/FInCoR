#include "CCircleDetect.h"
#include "CTransformation.h"
#include "ros/ros.h"
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>

#define MAX_PATTERNS 15

bool calibrated = true;

int defaultImageWidth = 320;
int defaultImageHeight = 240;
float circleDiameter = 0.056;
float rotateBy = 0;

ros::NodeHandle *nh;
image_transport::Publisher imdebug;

CRawImage *image;

CCircleDetect *detectorArray[MAX_PATTERNS];
STrackedObject objectArray[MAX_PATTERNS];
SSegment currentSegmentArray[MAX_PATTERNS];
SSegment lastSegmentArray[MAX_PATTERNS];
CTransformation *trans;

void imageCallback(const sensor_msgs::ImageConstPtr& msg) {
    printf("%s\n", "image_receaved");
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
        if (currentSegmentArray[i].valid) objectArray[i] = trans->transform(currentSegmentArray[i]);

        if(currentSegmentArray[i].x !=0.0 && currentSegmentArray[i].y != 0.0)
        {
        	printf("X:%f  Y:%f  type: %d\n", currentSegmentArray[i].x, currentSegmentArray[i].y, currentSegmentArray[i].type);
        	//float x; float y; float angle,horizontal; int size;
        }
    }
    //and publish the result
    memcpy((void*) &msg->data[0], image->data, msg->step * msg->height);
    imdebug.publish(msg);
}

void depthCallback(const sensor_msgs::ImageConstPtr& msg)
{

}

int main(int argc, char* argv[]) {
    ros::init(argc, argv, "circle_detector");
    nh = new ros::NodeHandle;
    image_transport::ImageTransport it(*nh);
    image = new CRawImage(defaultImageWidth, defaultImageHeight, 4);
    trans = new CTransformation(circleDiameter, nh);
    for (int i = 0; i < MAX_PATTERNS; i++) {
        detectorArray[i] = new CCircleDetect(defaultImageWidth, defaultImageHeight);
    }

image->getSaveNumber();
calibrated = trans->loadCalibration();
image_transport::Subscriber subim = it.subscribe("/left/rgb/image_mono", 1, imageCallback);
imdebug = it.advertise("/charging/processedimage", 1);
//ros::Publisher objectPoints = n.advertise<std_msgs::String>("chatter", 1);

ROS_DEBUG("Server running");
ros::spin();
delete image;
for (int i = 0; i < MAX_PATTERNS; i++) delete detectorArray[i];
delete trans;
return 0;
}