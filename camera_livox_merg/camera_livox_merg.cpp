#include <iostream>
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

uint64_t TO_MERGE_CNT = 1; 
ros::Publisher pub_pcl_out0, pub_out1;
// 相机内参和激光雷达到相机的外参
Eigen::Matrix3f camera_intrinsics;
Eigen::Matrix3f R;
Eigen::Vector3f t(0.0444954, -0.0279632, -0.0316928);

// 创建一个ROS图像消息
sensor_msgs::ImagePtr ros_image_out = boost::make_shared<sensor_msgs::Image>();
std::vector<livox_ros_driver2::CustomMsgConstPtr> livox_data;
int camera_livox_process(const sensor_msgs::ImageConstPtr& image_msg, const livox_ros_driver2::CustomMsgConstPtr& livox_msg) {

    livox_data.push_back(livox_msg);
    if (livox_data.size() < TO_MERGE_CNT) return 1;
    // 将ROS图像消息转换为OpenCV格式
    cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::BGR8);
    // 创建一个与图像大小相同的临时图层用于绘制点
    cv::Mat overlay = cv_ptr->image.clone();
    cv::Mat img_out = cv_ptr->image.clone();
    

    std::cout<<"MySyncPolicy:"<<livox_data.size()<<std::endl;
    for (size_t i = 0; i < livox_data.size(); ++i) {
        auto& livox_p = livox_data[i];
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
                cv::circle(overlay, cv::Point(int(point_image[0]), int(point_image[1])), 5, cv::Scalar(250 - dvalue, 0, dvalue), -1);
            }
        }
    }
    }
    float alpha = 0.5; // 设置点的透明度
    // 将原图与临时图层按照alpha值混合
    cv::addWeighted(overlay, alpha, img_out, 1 - alpha, 0, img_out);
    // // 显示最终图片
    // cv::imshow("Image with PCD", img_out);
    // cv::waitKey(0);
    cv_bridge::CvImage cv_image;
    cv_image.image = img_out;
    cv_image.encoding = "bgr8";
    cv_image.toImageMsg(*ros_image_out);
    pub_out1.publish(ros_image_out);
    livox_data.clear();
    return 0;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "livox_camera_node");
    ros::NodeHandle nh;

    ROS_INFO("start livox_camera_merg_node");
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
    pub_out1 = nh.advertise<sensor_msgs::Image>("/camera/livox_processed", 100);

    ros::spin();
}
