#ifndef ELEVATOR_MANAGER_DEPTH_H
#define ELEVATOR_MANAGER_DEPTH_H

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <Eigen/Dense>

class ElevatorManagerDepth {
  public:
    ElevatorManagerDepth();
    ~ElevatorManagerDepth();
    void process();

  private:
    void cloudCallback(const sensor_msgs::PointCloud2::ConstPtr& cloud);
    
    int hz_;
    float max_height_;

    sensor_msgs::PointCloud2ConstPtr realsense_cloud_;
    tf::TransformListener tf_listener_;

    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;
    ros::Subscriber cloud_sub_;
    ros::Publisher cloud_pub_;
};

#endif // ELEVATOR_MANAGER_DEPTH_H