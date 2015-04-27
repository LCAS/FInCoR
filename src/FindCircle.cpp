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
#include <visualization_msgs/MarkerArray.h>
#include <tf/transform_broadcaster.h>
#include "circle_detection/STrackedObject.h"
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <sensor_msgs/Image.h>
#include <math.h>


#define MAX_PATTERNS 6

int defaultImageWidth = 640;
int defaultImageHeight = 480;
float circleDiameter = 0.049;
float rotateBy = 0;

ros::NodeHandle *nh;
image_transport::Publisher imdebug;
tf::TransformListener* lookup;
ros::Publisher pubLeft, pubRight, vis_pub;

std::clock_t start;
std::string topic;

CRawImage *image;

CCircleDetect *detectorArray[MAX_PATTERNS];
STrackedObject objectArray[MAX_PATTERNS];
STrackedObject objectArray3D[MAX_PATTERNS];
SSegment currentSegmentArray[MAX_PATTERNS];
SSegment lastSegmentArray[MAX_PATTERNS];
CTransformation *trans;

std::string itemLookUp() {

    /*
     *
     * Give each circle a name or ID based on the ratio.
     * 
     */

}

void imageCallback(const sensor_msgs::ImageConstPtr& msg) {
    if (image->bpp != msg->step / msg->width || image->width != msg->width || image->height != msg->height) {
        delete image;
        ROS_DEBUG("Readjusting image format from %ix%i %ibpp, to %ix%i %ibpp.", image->width, image->height, image->bpp, msg->width, msg->height, msg->step / msg->width);
        image = new CRawImage(msg->width, msg->height, msg->step / msg->width);
    }

    memcpy(image->data, (void*) &msg->data[0], msg->step * msg->height);

    circle_detection::STrackedObject tracked_objects;
    tracked_objects.header = msg->header;
    visualization_msgs::MarkerArray marker_list;

    //search image for circles
    for (int i = 0; i < MAX_PATTERNS; i++) {
        lastSegmentArray[i] = currentSegmentArray[i];
        currentSegmentArray[i] = detectorArray[i]->findSegment(image, lastSegmentArray[i]);
        objectArray[i].valid = false;

        if (currentSegmentArray[i].valid) {
            objectArray[i] = trans->transform(currentSegmentArray[i]);
            
            if(isnan(objectArray[i].x)) continue;
            if(isnan(objectArray[i].y)) continue;
            if(isnan(objectArray[i].z)) continue;
            if(isnan(objectArray[i].roll)) continue;
            if(isnan(objectArray[i].pitch)) continue;
            if(isnan(objectArray[i].yaw)) continue;

            // printf("X:%f  Y:%f Z:%f R:%f\n", objectArray[i].x, objectArray[i].y, objectArray[i].z, objectArray[i].bwratio);
            //std::cout << "F: " << msg->header.frame_id << "  X: " << objectArray[i].x << "  Y: " << objectArray[i].y << "  Z: " << objectArray[i].z << "  R: " << objectArray[i].bwratio << std::endl;

            geometry_msgs::Pose circlePose;
            circlePose.position.x = -objectArray[i].y;
            circlePose.position.y = -objectArray[i].z;
            circlePose.position.z = objectArray[i].x;
            tf::Quaternion q;
            q.setRPY(objectArray[i].roll, objectArray[i].pitch, objectArray[i].yaw);
            circlePose.orientation.x = q.getX();
            circlePose.orientation.y = q.getY();
            circlePose.orientation.z = q.getZ();
            circlePose.orientation.w = q.getW();
            tracked_objects.pose.push_back(circlePose);
            tracked_objects.roundness.push_back(objectArray[i].roundness);
            tracked_objects.bwratio.push_back(objectArray[i].bwratio);
            tracked_objects.esterror.push_back(objectArray[i].esterror);

            visualization_msgs::Marker marker;
            marker.header = msg->header;
            marker.id = floor(objectArray[i].bwratio);
            marker.type = visualization_msgs::Marker::SPHERE;
            marker.action = visualization_msgs::Marker::MODIFY;
            marker.pose = circlePose;
            marker.scale.x = 0.01;
            marker.scale.y = 0.01;
            marker.scale.z = 0.1;
            marker.color.a = 1.0;
            marker.color.r = 0.0;
            marker.color.g = 1.0;
            marker.color.b = 0.0;
            marker.lifetime = ros::Duration(0.2);
            marker_list.markers.push_back(marker);
        }
    }

    if ((marker_list.markers.size() > 0) && (tracked_objects.pose.size() > 0)) {

        if (tracked_objects.header.frame_id.find("left") != std::string::npos) {
            pubLeft.publish(tracked_objects);
        } else {
            pubRight.publish(tracked_objects);
        }
        vis_pub.publish(marker_list);
    }
    //printf("%s\n", "--------------------");
    memcpy((void*) &msg->data[0], image->data, msg->step * msg->height);
    imdebug.publish(msg);
}

void cameraInfoCallBack(const sensor_msgs::CameraInfo &msg) {
    trans->updateParams(msg.K[2], msg.K[5], msg.K[0], msg.K[4]);
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

    pubLeft = nh->advertise<circle_detection::STrackedObject>("/circledetection/left_circleArray", 0);
    pubRight = nh->advertise<circle_detection::STrackedObject>("/circledetection/right_circleArray", 0);

    vis_pub = nh->advertise<visualization_msgs::MarkerArray>("/circledetection/rviz_marker", 0);
    lookup = new tf::TransformListener();

    ROS_DEBUG("Server running");
    ros::spin();
    delete image;
    for (int i = 0; i < MAX_PATTERNS; i++) delete detectorArray[i];
    delete trans;
    return 0;
}