#include "ros/ros.h"

//  darknet_ros_msgs
#include <darknet_ros_msgs/BoundingBox.h>
#include <darknet_ros_msgs/BoundingBoxes.h>

//  messege of publisher
#include <vision_msgs/Detection2DArray.h>
#include <vision_msgs/Detection2D.h>
#include <vision_msgs/ObjectHypothesisWithPose.h>
#include <vision_msgs/BoundingBox2D.h>
#include <geometry_msgs/Pose2D.h>

//  std module
#include <cmath>
#include <vector>

// using namespace vision_msgs;
// using namespace darknet_ros_msgs;

ros::Publisher detection2D_pub;

void detectionCallback(const darknet_ros_msgs::BoundingBoxes& msg) {
    //  get only BoundingBox
    std::vector<darknet_ros_msgs::BoundingBox> bounding_boxes = msg.bounding_boxes;
    //  Detection2DArray
    vision_msgs::Detection2DArray pubMsg;
    
    for (darknet_ros_msgs::BoundingBox bb : bounding_boxes) {
        vision_msgs::Detection2D object;
        //  contain id,score
        vision_msgs::ObjectHypothesisWithPose ob_result;
        ob_result.id = bb.id;
        ob_result.score = bb.probability;
        object.results.push_back(ob_result);

        //  bounding box
        vision_msgs::BoundingBox2D bbox;
        //  geometory pose 2d
        geometry_msgs::Pose2D center;
        center.x = (bb.xmin + bb.xmax) / 2;
        center.y = (bb.ymin + bb.ymax) / 2;
        center.theta = 0.0;
        //  assemble
        bbox.center = center;
        //  size
        bbox.size_x = std::abs(bb.xmax - bb.xmin);
        bbox.size_y = std::abs(bb.ymax - bb.ymin);
        //  insert
        object.bbox = bbox;

        //  insert into vector
        pubMsg.detections.push_back(object);
    }

    //  push message
    detection2D_pub.publish(pubMsg);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "darknet_ros_message_converter");
    ros::NodeHandle nh("~");

    //  subscriber
    ros::Subscriber detectionSub = nh.subscribe("/darknet_ros/bounding_boxes", 10, detectionCallback);
    //  publisher
    detection2D_pub = nh.advertise<vision_msgs::Detection2DArray>("/darknet_ros/converted_message", 1000);

    ros::spin();
    return 0;
}