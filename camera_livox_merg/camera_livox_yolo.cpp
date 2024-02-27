#include <iostream>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <Eigen/Dense>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include "livox_ros_driver2/CustomMsg.h"
#include <boost/foreach.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include "yolov8_ros_msgs/BoundingBox.h"
#include "yolov8_ros_msgs/BoundingBoxes.h"
#include <thread>
typedef pcl::PointXYZINormal PointType;
typedef pcl::PointCloud<PointType> PointCloudXYZI;

pcl::PointCloud<PointType> pcl_out;
uint64_t TO_MERGE_CNT = 1; 
ros::Publisher pub_out0, pub_out1,pub_pcl;
// 相机内参和激光雷达到相机的外参
Eigen::Matrix3f camera_intrinsics;
Eigen::Matrix3f R;
Eigen::Vector3f t(0.0444954, -0.0279632, -0.0316928);
ros::Time last_time;
int frame_count=0;
// 创建一个ROS图像消息
yolov8_ros_msgs::BoundingBoxes boxes;
sensor_msgs::ImagePtr ros_image_out0 = boost::make_shared<sensor_msgs::Image>();
sensor_msgs::ImagePtr ros_image_out1 = boost::make_shared<sensor_msgs::Image>();
cv::Mat img_raw;
std::vector<livox_ros_driver2::CustomMsgConstPtr> livox_data;
livox_ros_driver2::CustomMsgPtr livox_cam_data(new livox_ros_driver2::CustomMsg);
livox_ros_driver2::CustomMsgPtr livox_cam_data_buf(new livox_ros_driver2::CustomMsg);
int camera_livox_process_started=0;
int camera_livox_process(const sensor_msgs::ImageConstPtr& image_msg, const livox_ros_driver2::CustomMsgConstPtr& livox_msg) {
    camera_livox_process_started=1;
    livox_data.push_back(livox_msg);
    if (livox_data.size() < TO_MERGE_CNT) return 1;
    // 将ROS图像消息转换为OpenCV格式
    cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::BGR8);
    // 创建一个与图像大小相同的临时图层用于绘制点
    cv::Mat overlay = cv_ptr->image.clone();
    cv::Mat img_out = cv_ptr->image.clone();
    img_raw= cv_ptr->image.clone();

    std::cout<<"MySyncPolicy:"<<livox_data.size()<<std::endl;
    for (size_t i = 0; i < livox_data.size(); ++i) {
        auto& livox_p = livox_data[i];
        int j=0;
        for (unsigned int i = 0; i < livox_p->point_num; ++i){
        // 将点从激光雷达坐标系转换到相机坐标系
        Eigen::Vector3f point(livox_p->points[i].x,livox_p->points[i].y, livox_p->points[i].z);
        Eigen::Vector3f point_camera = R * point + t;

        // 将点云投影到相机成像平面
        Eigen::Vector3f point_image = camera_intrinsics * point_camera;
        point_image /= point_image[2];

        if (point_image[0] >= 0 && point_image[1] >= 0 && point_image[0] < overlay.cols && point_image[1] < overlay.rows) {
            float z = point_camera[2];
            if (z > 0.2) {
                int dvalue = std::min(std::max(z / 8.0f, 0.0f), 1.0f) * 250;
                //dvalue = livox_p->points[i].reflectivity;
                cv::circle(overlay, cv::Point(int(point_image[0]), int(point_image[1])), 5, cv::Scalar(250 - dvalue, 0, dvalue), -1);
                livox_cam_data->points.push_back(livox_p->points[i]);
                j++;livox_cam_data->point_num++;
            }
        }
    }
    }
    float alpha = 0.5; // 设置点的透明度
    // 将原图与临时图层按照alpha值混合
    cv::addWeighted(overlay, alpha, img_out, 1 - alpha, 0, img_out);
    // // 显示最终图片
    cv::imshow("Image with PCD", img_out);
    cv::waitKey(3);
    cv_bridge::CvImage cv_image;
    cv_image.image = img_out;
    cv_image.encoding = "bgr8";
    cv_image.toImageMsg(*ros_image_out1);
    pub_out1.publish(ros_image_out1);
    livox_data.clear();
    if (livox_cam_data->point_num>10000)
    {
    livox_cam_data->points.clear();
    livox_cam_data->point_num=0;
    }
    return 0;
}

float findMedian(std::vector<float>& nums) {
    // 首先，排序数组
    std::sort(nums.begin(), nums.end());

    int n = nums.size();
    // 检查元素数量是奇数还是偶数
    if (n % 2 == 0) {
        // 如果是偶数，返回中间两个数的平均值
        return (nums[n / 2 - 1] + nums[n / 2]) / 2.0;
    } else {
        // 如果是奇数，返回中间的数
        return nums[n / 2];
    }
}
// 绘制边界框的函数
void drawBoundingBox(cv::Mat& image, const yolov8_ros_msgs::BoundingBoxes::ConstPtr& boxes) {
    yolov8_ros_msgs::BoundingBox box;
    
    //cv::Mat overlay = image.clone();

    for (int i=0;i<boxes->bounding_boxes.size();i++) {
        std::vector<float> aera_d{-1};
        PointType pt;
        box.xmin = static_cast<int>(boxes->bounding_boxes[i].xmin);
        box.ymin = static_cast<int>(boxes->bounding_boxes[i].ymin);
        box.xmax = static_cast<int>(boxes->bounding_boxes[i].xmax);
        box.ymax = static_cast<int>(boxes->bounding_boxes[i].ymax);
        int boxcx=( box.xmin+ box.xmax)/2;
        int boxcy=( box.ymin+ box.ymax)/2;
        box.Class = boxes->bounding_boxes[i].Class;  // 假设类别名是字符串
        if (box.Class!="balloon") continue;
        box.probability = boxes->bounding_boxes[i].probability;
        for (unsigned int j = 0; j < livox_cam_data->points.size(); ++j){
            // 将点从激光雷达坐标系转换到相机坐标系
            Eigen::Vector3f point(livox_cam_data->points[j].x,livox_cam_data->points[j].y, livox_cam_data->points[j].z);
            Eigen::Vector3f point_camera = R * point + t;

            // 将点云投影到相机成像平面
            Eigen::Vector3f point_image = camera_intrinsics * point_camera;
            point_image /= point_image[2];
            float min_fd=10000;
            if (point_image[0] >= 0 && point_image[1] >= 0 && point_image[0] < image.cols && point_image[1] < image.rows\
                && point_camera[2]>0.2 && point_image[0] > box.xmin && point_image[1] > box.ymin \
                && point_image[0] < box.xmax && point_image[1] < box.ymax) {
                float d = sqrt(point[0]*point[0]+point[1]*point[1]+point[2]*point[2]);
                if (sqrt((boxcx-point_image[0])*(boxcx-point_image[0])+(boxcy-point_image[1])*(boxcy-point_image[1]))<min_fd){
                    min_fd=sqrt((boxcx-point_image[0])*(boxcx-point_image[0])+(boxcy-point_image[1])*(boxcy-point_image[1]));
                    pt.x=point[0];pt.y=point[1];pt.z=point[2];
                }
               
                int dvalue = std::min(std::max(d / 8.0f, 0.0f), 1.0f) * 250;
                //dvalue = livox_p->points[i].reflectivity;
                cv::circle(image, cv::Point(int(point_image[0]), int(point_image[1])), 5, cv::Scalar(250 - dvalue, 0, dvalue), -1);
                aera_d.push_back(d);
            }
        }
         pcl_out.push_back(pt);
        float mid_d=-1;
        mid_d=findMedian(aera_d);
        aera_d.clear();
        if (mid_d<0)
            std::cout<<"debug:"<<aera_d.size()<<"  "<<livox_cam_data->points.size()<<std::endl;

        
        // 设置颜色和标签
        std::ostringstream stream1,stream0,ss2,ss3,ss4;
        cv::Scalar color = cv::Scalar(0, 255, 0); // 绿色
         // 设置固定的浮点数表示法和精度
        stream0 << std::fixed << std::setprecision(1) << box.probability;
        stream1 << std::fixed << std::setprecision(2) << mid_d;
        ss2 << std::fixed << std::setprecision(2) <<pt.x;
        ss3 << std::fixed << std::setprecision(2) <<pt.y;
        ss4 << std::fixed << std::setprecision(2) <<pt.z;
        std::string label = box.Class + ":" + stream0.str()+" dist:"+stream1.str();
        std::string dxyz=" x:"+ss2.str()+" y:"+ss3.str()+" z:"+ss4.str();
        // 绘制矩形中心
        cv::rectangle(image, cv::Point(box.xmin, box.ymin), cv::Point(box.xmax, box.ymax), color, 4);
        cv::circle(image, cv::Point(boxcx, boxcy), 6, color, -1);
        
        // 绘制标签
        int baseLine;
        cv::Size labelSize = cv::getTextSize(label, cv::FONT_HERSHEY_SIMPLEX,1, 2, &baseLine);
        cv::putText(image, label, cv::Point(box.xmin, box.ymin - labelSize.height), 
                    cv::FONT_HERSHEY_SIMPLEX, 0.9, color, 3);
        cv::putText(image, dxyz, cv::Point(box.xmin, box.ymin - labelSize.height+20), 
                    cv::FONT_HERSHEY_SIMPLEX, 0.9, (255, 255,0), 3);
    }
    float alpha = 0.5; // 设置点的透明度
    // 将原图与临时图层按照alpha值混合
    //cv::addWeighted(overlay, alpha, image, 1 - alpha, 0, image);
    livox_cam_data->points.clear();
    livox_cam_data->point_num=0;
}



void boundingBoxesCallback(const yolov8_ros_msgs::BoundingBoxes::ConstPtr& boxes_msg) {
    
    if(livox_cam_data->points.size()<1)
        return;
    if (camera_livox_process_started==0) return;

    ros::Time current_time = ros::Time::now();
    // 计算自上次回调以来经过的时间
    ros::Duration time_elapsed = current_time - last_time;frame_count++;
    // 将ROS图像消息转换为OpenCV格式

    // 创建一个与图像大小相同的临时图层用于绘制框
    cv::Mat img_out = img_raw.clone();
    std::cout<<"MySyncPolicy:"<<2<<std::endl;

    // 计算 FPS
    static double fps=0 ;
    std::cout<< time_elapsed.toSec()<<std::endl;
    if (time_elapsed.toSec() > 1.0) {
        fps = static_cast<double>(frame_count) / time_elapsed.toSec();
        //ROS_INFO("Current FPS: %.2f", fps);
        frame_count = 0;last_time = current_time;
        }
    std::ostringstream stream;
    stream << std::fixed << std::setprecision(1) << fps;
    cv::putText(img_out, "FPS: " + stream.str(), cv::Point(10, 50), cv::FONT_HERSHEY_PLAIN, 2, cv::Scalar(0, 0, 0), 4);
    // 处理 BoundingBoxes 消息
    drawBoundingBox(img_out, boxes_msg);
    cv::imshow("Image with PCD Yolo", img_out);
    cv::waitKey(3);
    cv_bridge::CvImage cv_image;
    cv_image.header=boxes_msg->header;
    cv_image.image = img_out;
    cv_image.encoding = "bgr8";
    cv_image.toImageMsg(*ros_image_out0);
    pub_out0.publish(ros_image_out0);

    sensor_msgs::PointCloud2 pcl_ros_msg;
    std::cout<<"blloon_num:"<<pcl_out.points.size()<<std::endl;
    for (size_t i = 0; i < pcl_out.points.size(); ++i) {
        std::cout<<"x:"<<pcl_out.points[i].x<<" y:"<<pcl_out.points[i].y<<" z:"<<pcl_out.points[i].z<<" "<<std::endl;
    }
    
    pcl::toROSMsg(pcl_out, pcl_ros_msg);
    pcl_ros_msg.header=boxes_msg->header;
    pub_pcl.publish(pcl_ros_msg);
    pcl_out.clear();
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "livox_camera_yolo_node");
    ros::NodeHandle nh;
    last_time= ros::Time::now();
    ROS_INFO("start livox_camera_yolo_node");
     camera_intrinsics << 905.885498046875, 0, 645.70458984375,
                    0, 905.973388671875, 363.8595886230469,
                    0, 0, 1;
     R << 4.23096e-05, -0.999867, -0.0163236,
        -0.0253391, 0.0163173, -0.999546,
      0.999679, 0.000455914, -0.025335;
    // 定义消息过滤器订阅器
    message_filters::Subscriber<livox_ros_driver2::CustomMsg> livox_sub(nh, "/livox/lidar", 1);
    message_filters::Subscriber<sensor_msgs::Image> image_sub(nh, "/camera/color/image_raw", 1);
    // 时间同步策略，这里使用的是ApproximateTime，它为不完全同步的消息提供了一种处理方式
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image,livox_ros_driver2::CustomMsg> MySyncPolicy;
    message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), image_sub,livox_sub);
    sync.registerCallback(boost::bind(&camera_livox_process, _1, _2));
    ros::Subscriber sub = nh.subscribe("/yolov8/BoundingBoxes", 5, boundingBoxesCallback);
    pub_out1 = nh.advertise<sensor_msgs::Image>("/camera/livox_processed", 1);
    pub_out0 = nh.advertise<sensor_msgs::Image>("/camera/yolo/livox_processed", 2);
    pub_pcl = nh.advertise<sensor_msgs::PointCloud2>("/balloons/xyz", 2);
    ros::spin();
}
