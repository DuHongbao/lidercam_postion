#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include "livox_ros_driver2/CustomMsg.h"
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <boost/foreach.hpp>
using std::to_string;
typedef pcl::PointXYZINormal PointType;
typedef pcl::PointCloud<PointType> PointCloudXYZI;

ros::Publisher pub_pcl_out0, pub_pcl_out1;
uint64_t TO_MERGE_CNT = 1; 
constexpr bool b_dbg_line = false;
std::vector<livox_ros_driver2::CustomMsgConstPtr> livox_data;
sensor_msgs::PointCloud2 process(const livox_ros_driver2::CustomMsgConstPtr& livox_msg_in) {
  livox_data.push_back(livox_msg_in);
  //if (livox_data.size() < TO_MERGE_CNT) return;

  pcl::PointCloud<PointType> pcl_in;

  for (size_t j = 0; j < livox_data.size(); j++) {
    auto& livox_msg = livox_data[j];
    auto time_end = livox_msg->points.back().offset_time;
    for (unsigned int i = 0; i < livox_msg->point_num; ++i) {
      PointType pt;
      pt.x = livox_msg->points[i].x;
      pt.y = livox_msg->points[i].y;
      pt.z = livox_msg->points[i].z;
      float s = livox_msg->points[i].offset_time / (float)time_end;

      pt.intensity = livox_msg->points[i].line +livox_msg->points[i].reflectivity /10000.0 ; // The integer part is line number and the decimal part is timestamp
      pt.curvature = s*0.1;
      pcl_in.push_back(pt);
    }
  }

  unsigned long timebase_ns = livox_data[0]->timebase;
  ros::Time timestamp;
  timestamp.fromNSec(timebase_ns);

  sensor_msgs::PointCloud2 pcl_ros_msg;
  pcl::toROSMsg(pcl_in, pcl_ros_msg);
  pcl_ros_msg.header.stamp.fromNSec(timebase_ns);
  pcl_ros_msg.header.frame_id = "/livox";
  livox_data.clear();
  return pcl_ros_msg;
}

int main(int argc, char** argv) {
  for(int i=0;i<=15;i++)
  {
  rosbag::Bag bag_read;
  bag_read.open("/home/robot-2/data/lidar/"+to_string(i)+".bag", rosbag::bagmode::Read);
  rosbag::Bag bag_write;
  bag_write.open("/home/robot-2/data/lidar/m_"+to_string(i)+".bag", rosbag::bagmode::Write);
  rosbag::View view(bag_read);
  for(auto it = view.begin(); it != view.end(); ++it) {
  		const rosbag::MessageInstance& m = *it;
            if (m.getTopic() == "/livox/lidar" || ("/" + m.getTopic() == "/livox/lidar")) {
                // 假设原始消息类型和目标消息类型都是sensor_msgs/PointCloud2
                auto input_msg = m.instantiate<livox_ros_driver2::CustomMsg>();
                if (input_msg != nullptr) {
                    // 调用process函数处理消息
                    auto output_msg=process(input_msg);
                    // 使用新的话题名称写入修改后的消息
                    bag_write.write("/livox_pcl", m.getTime(), output_msg);
                }
            } else {
                // 对于不需要修改的消息，直接写入新的bag文件
                bag_write.write(m.getTopic(), m.getTime(), m, m.getConnectionHeader());
            }
        }

        bag_read.close();
        bag_write.close();
  }
 
}


