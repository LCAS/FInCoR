#ifndef __FIND_CIRCLE_H__
#define __FIND_CIRCLE_H__

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
#include "circle_detection/detection_results.h"
#include "circle_detection/detection_results_array.h"
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <sensor_msgs/Image.h>
#include <math.h>
#include <boost/uuid/uuid.hpp>
#include <boost/uuid/uuid_generators.hpp>
#include <boost/uuid/uuid_io.hpp>
#include <map>
#include <string>

#define MAX_PATTERNS 16

class FindCircle {
public:

    int defaultImageWidth;
    int defaultImageHeight;
    float circleDiameter;
    std::string node_name;
    void loadConfig(void);
    void cameraInfoCallBack(const sensor_msgs::CameraInfo &msg);
    void imageCallback(const sensor_msgs::ImageConstPtr& msg);
    void init(void);
    FindCircle(void);
    ~FindCircle(void);

private:
    ros::NodeHandle *nh;
    image_transport::Publisher imdebug;
    tf::TransformListener* lookup;
    ros::Publisher tracks_pub, vis_pub;
    image_transport::Subscriber subim;
    
    CRawImage *image;
    CCircleDetect *detectorArray[MAX_PATTERNS];
    STrackedObject objectArray[MAX_PATTERNS];
    STrackedObject objectArray3D[MAX_PATTERNS];
    SSegment currentSegmentArray[MAX_PATTERNS];
    SSegment lastSegmentArray[MAX_PATTERNS];

    //3D transform code
    CTransformation *trans;
};

#endif