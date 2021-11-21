#include "ros/ros.h"

#include <iostream>
#include <cstdio>

#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

#include "sensor_msgs/Image.h"
#include <vision_msgs/Detection2DArray.h>
#include <vision_msgs/Detection2D.h>

//  for publishing image data (for debug)
//#include <image_transport/image_transport.h>

#include <std_msgs/UInt16MultiArray.h>

// For Yolov4-tiny_ball
static const std::vector<std::string> labelMap = {
    "ball", "house"};

//  iamge topic publisher
//image_transport::Publisher image_pub;

//  depth topic publisher
ros::Publisher depth_pub;

//  image
cv::Mat subscribed_image;

//  detected object's Data struct
struct DetectionData {
    //  label id
    int id = -1;
    //  center x,y
    double center_x;
    double center_y;
    //  size x,y
    double size_x;
    double size_y;
    // depth data
    double depth;
};

// detected objects
std::vector<DetectionData> datas;

//  initialize margin
/** Note that the unit of margin is a ratio.
 *  eg) if margin_x is 1, x size of the box will be twice.
 */
double margin_x = 0.0;
double margin_y = 0.0;

/**
 * Draw frame
 * @param img[cv::Mat]  image source
 * @param data[DetectionData] include information of detected objects
 */


void getDepth(cv::Mat img, std::vector<DetectionData> _datas)
{
    /**
     * Note: Mat's notation is not (x,y), but (row,col). 
     * O    row
     *   * - - - >
     * c |
     * o |
     * l |
     *   _
     * 
     */
    //  Note: both Mat size and max detection positon are 416
    const int MAX_SIZE = 416;

    //  clone depth image
    cv::Mat _img = img.clone();
    // ROS_INFO("Row:%d Col:%d", _img.rows, _img.cols);

    //  resize depth image
    cv::Mat resized_img;
    cv::resize(_img, resized_img, cv::Size(MAX_SIZE, MAX_SIZE), cv::INTER_LINEAR);
    
    //  define output depth array
    std_msgs::UInt16MultiArray depths_msg;
    //  resize depth array in accordance with the number of objects.
    depths_msg.data.resize(_datas.size());
    //ROS_INFO("number of objects: %d", _datas.size());

    //  define index number of depth data
    int index = 0;

    //  define output image
    //cv::Mat output_img = cv::Mat::zeros(resized_img.size(), resized_img.type());

    //  for each
    for (DetectionData _data : _datas) {
        if(_data.id != -1){
            //  Define mask image
            cv::Mat mask = cv::Mat::zeros(resized_img.size(), CV_8UC1);
            //  Define destination image of an object
            cv::Mat object_img = cv::Mat::zeros(resized_img.size(), resized_img.type());

             //  format object size
            double size_x = _data.size_x;
            double size_y = _data.size_y;

            //  center positon
            cv::Point centerPoint = cv::Point(_data.center_x, _data.center_y);

            //  box position
            double position_x_min = centerPoint.x - (size_x / 2);
            double position_x_max = centerPoint.x + (size_x / 2);
            double position_y_min = centerPoint.y - (size_y / 2);
            double position_y_max = centerPoint.y + (size_y / 2);

            //  draw filled rectangle on the mask image with 
            cv::rectangle(
                mask,
                cv::Point(position_x_min - margin_x*(size_x/2), position_y_min - margin_y*(size_y/2)),
                cv::Point(position_x_max + margin_x*(size_x/2), position_y_max + margin_y*(size_y/2)),
                1,
                -1
            );

            // Copy source image to destination imag with masking
            resized_img.copyTo(object_img, mask);

            // Get locations of non-zero pixels
            std::vector<cv::Point> nonZeroLocs;
            cv::findNonZero(object_img, nonZeroLocs);

            //  Initialize minimum depth using maximum value of uint16_t
            uint16_t depth_min = std::numeric_limits<uint16_t>::max(); 

            //  for each non-zero point
            for (cv::Point nonZeroPoint : nonZeroLocs) {
                uint16_t depth = resized_img.at<uint16_t>(nonZeroPoint);
                if (depth_min > depth) depth_min  = depth;
            }

            depths_msg.data[index] = depth_min;

            //ROS_INFO("Approximate distance[mm]: %d", depth_min);

            // Clone masked image as an output image
            //output_img = object_img.clone();

            index++;
        }
    }

    //  publish depth topic
    depth_pub.publish(depths_msg);

    //  convert
    //sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "16UC1", output_img).toImageMsg();
    //  publish image topic
    //image_pub.publish(msg);
}

void depthCallback(const sensor_msgs::Image& msg)
{
    try{
        subscribed_image = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_16UC1)->image;
        //  call drawing function
        getDepth(subscribed_image, datas);
    }
    catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
    }
}


void detectionCallback(const vision_msgs::Detection2DArray& msg)
{
    datas.clear();  //  clear buffer

    // Check if detections[] is empty or not
    if(!msg.detections.empty()) {
        //  insert dataros opencv
        for(vision_msgs::Detection2D _d : msg.detections) {  //  insert data
            DetectionData data = {(int)_d.results[0].id, _d.bbox.center.x, _d.bbox.center.y, _d.bbox.size_x, _d.bbox.size_y};
            datas.push_back(data);

            //  debug 
            ROS_INFO("whatis:%s position:%f/%f size:%f/%f", labelMap[_d.results[0].id].c_str(), _d.bbox.center.x, _d.bbox.center.x, _d.bbox.size_x, _d.bbox.size_y);
        }
    }
}

int main(int argc, char** argv){

    ros::init(argc, argv, "od_depth_node");
    ros::NodeHandle pnh("~");

    // Uses the margin values from param if passed or else no margins
    if (pnh.hasParam("margin_x"))
    {
      pnh.getParam("margin_x", margin_x);
    }
    if (pnh.hasParam("margin_y"))
    {
      pnh.getParam("margin_y", margin_y);
    }

    //  subscribers
    ros::Subscriber depthSub = pnh.subscribe("/depth_image", 10, depthCallback);
    ros::Subscriber detectionSub = pnh.subscribe("/object_detections", 10, detectionCallback);

    //  publishers
    //  for processed depth image
    //image_transport::ImageTransport it(pnh);
    //image_pub = it.advertise("/od_depth/resized_depth/", 10);
    //  for depth data
    depth_pub = pnh.advertise<std_msgs::UInt16MultiArray>("/od_depth/object_depths", 10);

    ros::spin();

    return 0;
}