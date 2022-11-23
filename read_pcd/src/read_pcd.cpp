#include<ros/ros.h>
#include<pcl/point_cloud.h>
#include<pcl_conversions/pcl_conversions.h>
#include<sensor_msgs/PointCloud2.h>
#include<sensor_msgs/LaserScan.h>
#include<pcl/io/pcd_io.h>//which contains the required definitions to load and store point clouds to PCD and other file formats.
#include <sensor_msgs/point_cloud2_iterator.h>

int main (int argc, char **argv)
{
  ros::init (argc, argv, "UandBdetect");
  ros::NodeHandle nh;
  ros::Publisher pcl_pub = nh.advertise<sensor_msgs::PointCloud2> ("pcl_output", 1);
  pcl::PointCloud<pcl::PointXYZ> cloud;
  sensor_msgs::LaserScan cloud_msg;
  sensor_msgs::PointCloud2 output;

  //PCD read
  pcl::io::loadPCDFile ("/home/zhanglei/Cart/lm/cartographer_detailed_comments_ws/scandata/1405076409.295238.pcd", cloud); //修改自己pcd文件所在路径
  //Convert the cloud to ROS message
  //PCL -> PointCLoud2
  pcl::toROSMsg(cloud, output);


  output.header.frame_id = "horizontal_laser_link";
  //output.header.frame_id = "map";
  output.header.seq = 0;
/*
    output.header = cloud_msg.header;

    output.angle_min = cloud_msg. //angle_min;
    output.angle_max = angle_max_;
    output.angle_increment = angle_increment_;
    output.time_increment = 0.0;
    output.scan_time = scan_time_;
    output.range_min = range_min_;
    output.range_max = range_max_;
*/
/*

    float angle_min_ = -2.3518311977286475;
    float angle_max_ =  2.3518311977286475;
    float angle_increment_ = 0.004363323096185923;
    float scan_time_ = 0.02500000044703484;
    float time_increment_ = 1.736111516947858e-05;
    float range_min_ = 0.02300000000044703484;
    float range_max_ = 60.0;


        //这些参数也是从param中读取
    output.angle_min = angle_min_;
    output.angle_max = angle_max_;
    output.angle_increment = angle_increment_;
    output.time_increment = 0.0;
    output.scan_time = scan_time_;
    output.range_min = range_min_;
    output.range_max = range_max_;

    //确定存储空间
    uint32_t ranges_size = std::ceil((output.angle_max - output.angle_min) / output.angle_increment);
    //output.ranges.resize(ranges_size);
    //determine if laserscan rays with no obstacle data will evaluate to infinity or max_range
    sensor_msgs::PointCloud2ConstPtr cloud_out;

      //cloud_out = &cloud_msg;


    // Iterate through pointcloud
    for (sensor_msgs::PointCloud2ConstIterator<float>
              iter_x(cloud_msg, "x"), iter_y(cloud_msg, "y"), iter_z(cloud_msg, "z");
              iter_x != iter_x.end();
              ++iter_x, ++iter_y, ++iter_z)
    {

      double range = hypot(*iter_x, *iter_y);

      double angle = atan2(*iter_y, *iter_x);


      //overwrite range at laserscan ray if new range is smaller
      int index = (angle - output.angle_min) / output.angle_increment;
      //output.ranges[index] = range;
      output.ranges.push_back(range);
    }
*/

  ros::Rate loop_rate(40);
  while (ros::ok())
  {
      output.header.seq ++;
      pcl_pub.publish(output);
      output.header.stamp = ros::Time::now();
      ros::spinOnce();
      loop_rate.sleep();
  }
  return 0;
}

