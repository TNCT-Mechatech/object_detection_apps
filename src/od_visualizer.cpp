#include "ros/ros.h"

#include <iostream>
#include <cstdio>

#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

#include "sensor_msgs/Image.h"
#include <vision_msgs/Detection2DArray.h>
#include <vision_msgs/Detection2D.h>

#include <image_transport/image_transport.h>


// For Yolov4-tiny
/*static const std::vector<std::string> labelMap = {
    "person",        "bicycle",      "car",           "motorbike",     "aeroplane",   "bus",         "train",       "truck",        "boat",
    "traffic light", "fire hydrant", "stop sign",     "parking meter", "bench",       "bird",        "cat",         "dog",          "horse",
    "sheep",         "cow",          "elephant",      "bear",          "zebra",       "giraffe",     "backpack",    "umbrella",     "handbag",
    "tie",           "suitcase",     "frisbee",       "skis",          "snowboard",   "sports ball", "kite",        "baseball bat", "baseball glove",
    "skateboard",    "surfboard",    "tennis racket", "bottle",        "wine glass",  "cup",         "fork",        "knife",        "spoon",
    "bowl",          "banana",       "apple",         "sandwich",      "orange",      "broccoli",    "carrot",      "hot dog",      "pizza",
    "donut",         "cake",         "chair",         "sofa",          "pottedplant", "bed",         "diningtable", "toilet",       "tvmonitor",
    "laptop",        "mouse",        "remote",        "keyboard",      "cell phone",  "microwave",   "oven",        "toaster",      "sink",
    "refrigerator",  "book",         "clock",         "vase",          "scissors",    "teddy bear",  "hair drier",  "toothbrush"};

*/

// For Yolov4-tiny_ball
static const std::vector<std::string> labelMap = {
    "ball", "house"};

static const std::vector<std::string> dogMesseage = {
    "Woof woof (Found ball)",
    "Ruff ruff (My home!)",
    "Bow-wow (My Favorite!)",
    "Yip! Yip! (Found owner)"
};

//  iamge topic publisher
image_transport::Publisher image_pub;

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
};

// detected objects
std::vector<DetectionData> datas;

/**
 * Draw frame
 * @param img[cv::Mat]  image source
 * @param data[DetectionData] include information of detected objects
 */
void drawFrame(cv::Mat img, std::vector<DetectionData> _datas)
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

    //  clone image
    cv::Mat _img = img.clone();
    // ROS_INFO("Row:%d Col:%d", _img.rows, _img.cols);
    
    //  for each
    for (DetectionData _data : _datas) {
        if(_data.id != -1){
            //  Note: both Mat size and Detection max positon are 416
            const int MAX_SIZE = 416;
            //  center positon
            cv::Point centerPoint = cv::Point(_data.center_x, _data.center_y);
            //  format object size
            double size_x = _data.size_x;
            double size_y = _data.size_y;
            //  box position
            double position_x_min = centerPoint.x - (size_x / 2);
            double position_x_max = centerPoint.x + (size_x / 2);
            double position_y_min = centerPoint.y - (size_y / 2);
            double position_y_max = centerPoint.y + (size_y / 2);

            //  label name 
            std::string label_name = labelMap[_data.id];

            //  color
            auto greenColor = cv::Scalar(0, 255, 0);
            auto greenOppacityColor = cv::Scalar(0, 255, 0);
            auto whiteColor = cv::Scalar(255, 255, 255);

            //  display text
            cv::putText(
                _img,
                label_name,
                cv::Point(position_x_min, position_y_min - 4),
                cv::FONT_HERSHEY_PLAIN,
                1.5,
                greenColor
            );
        
            //  draw rectangle
            cv::rectangle(
                _img,
                cv::Point(position_x_min, position_y_min),
                cv::Point(position_x_max, position_y_max),
                greenColor,
                2
            );

            /////////////////////////////
            //  display telop
            const bool telop = true;
            if (telop) {
                //  rectangle
                cv::Mat overlay;
                _img.copyTo(overlay);
                double alpha = 0.4;
                cv::rectangle(
                    overlay,
                    cv::Point(0, MAX_SIZE - 50),
                    cv::Point(MAX_SIZE, MAX_SIZE),
                    greenColor,
                    -1
                );
                cv::addWeighted(
                    overlay, 
                    alpha, 
                    _img, 
                    1 - alpha, 
                    0, 
                    _img
                );

                //  text
                cv::putText(
                    _img,
                    dogMesseage[_data.id],
                    cv::Point(0, MAX_SIZE - 25),
                    cv::FONT_HERSHEY_DUPLEX,
                    1,
                    whiteColor
                );
            }
        }
    }

    //  convert
    sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", _img).toImageMsg();
    //  publish image topic
    image_pub.publish(msg);
}

void imageCallback(const sensor_msgs::Image& msg)
{
    
    try{
        subscribed_image = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8)->image;
        //  call drawing function
        drawFrame(subscribed_image, datas);
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
    ros::init(argc, argv, "od_visualizer_node");
    ros::NodeHandle pnh("~");

    ros::Subscriber imageSub = pnh.subscribe("/yolo_publisher/color/image", 10, imageCallback);
    ros::Subscriber detectionSub = pnh.subscribe("/yolo_publisher/color/yolo_detections", 10, detectionCallback);

    //  publisher
    image_transport::ImageTransport it(pnh);
    image_pub = it.advertise("/od_visualizer/color/image/", 10);

    ros::spin();

    return 0;
}