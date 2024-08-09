#ifndef ELEVATOR_MANAGER_LIDAR_H
#define ELEVATOR_MANAGER_LIDAR_H

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Twist.h>
#include <math.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>

class ElevatorManagerLidar
{
  public:
    ElevatorManagerLidar();
    ~ElevatorManagerLidar();
    void process();

  private:
    void FrontlidarCallback(const sensor_msgs::LaserScan::ConstPtr& msg);
    void RearlidarCallback(const sensor_msgs::LaserScan::ConstPtr& msg);
    void imuCallback(const sensor_msgs::Imu::ConstPtr& msg);
    void plannerCallback(const std_msgs::String::ConstPtr& msg);
    void stop();
    void straight();
    void back();
    bool isBehindObs();
    bool isMovingElevator();

    int hz_;
    int up_count_;
    int down_count_;
    int straight_count_ = 0;
    int back_count_ = 0;
    bool is_stop_front_ = false;
    bool is_stop_rear_ = false;
    bool upfloor = false;
    bool downfloor = false;
    bool up_once_ = false;
    bool down_once_ = false;
    bool up_twice_ = false;
    bool down_twice_ = false;
    bool once_ = false;
    bool twice_ = false;
    bool elevator_planner_ = false;
    float distance_threshold_;
    float width_threshold_;

    sensor_msgs::LaserScan front_laser_;
    sensor_msgs::LaserScan rear_laser_;
    sensor_msgs::Imu imu_;
    geometry_msgs::Twist cmd_vel_;
    std_msgs::Bool upfloor_;
    std_msgs::Bool downfloor_;
    std_msgs::String planner_;

    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;

    ros::Subscriber front_laser_sub_;
    ros::Subscriber rear_laser_sub_;
    ros::Subscriber imu_sub_;
    ros::Subscriber planner_sub_;
    ros::Publisher cmd_vel_pub_;
    ros::Publisher upfloor_pub_;
    ros::Publisher downfloor_pub_;
};

#endif // ELEVATOR_MANAGER_LIDAR_H
