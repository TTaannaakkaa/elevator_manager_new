# include "elevator_manager/elevator_manager_lidar.h"

ElevatorManagerLidar::ElevatorManagerLidar() : private_nh_("~") {
  private_nh_.param<int>("hz", hz_, 10);
  private_nh_.param<float>("distance_threshold", distance_threshold_, 0.5);
  private_nh_.param<float>("width_threshold", width_threshold_, 0.5);

  front_laser_sub_ = nh_.subscribe("/front_laser/scan", 1, &ElevatorManagerLidar::FrontlidarCallback, this);
  rear_laser_sub_ = nh_.subscribe("/rear_laser/scan", 1, &ElevatorManagerLidar::RearlidarCallback, this);
  imu_sub_ = nh_.subscribe("/imu/data", 1, &ElevatorManagerLidar::imuCallback, this);
  planner_sub_ = nh_.subscribe("/local_planner/cmd_vel/selected", 1, &ElevatorManagerLidar::plannerCallback, this);

  cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/local_planner/elevator_planner/cmd_vel", 1);
  upfloor_pub_ = nh_.advertise<std_msgs::Bool>("/up_flag", 1);
  downfloor_pub_ = nh_.advertise<std_msgs::Bool>("/down_flag", 1);

  up_count_ = 0;
  down_count_ = 0;
}

ElevatorManagerLidar::~ElevatorManagerLidar() {
}

void ElevatorManagerLidar::FrontlidarCallback(const sensor_msgs::LaserScan::ConstPtr& msg) {
  front_laser_ = *msg;
  for(int i = 0; i < front_laser_.ranges.size(); i += 5) {
    const double angle = front_laser_.angle_min + i * front_laser_.angle_increment;
    const double range = front_laser_.ranges[i];
    if((-width_threshold_ < range * sin(angle) && range * sin(angle) < width_threshold_) && range * cos(angle) < distance_threshold_) {
      is_stop_front_ = true;
    }
  }
}

void ElevatorManagerLidar::RearlidarCallback(const sensor_msgs::LaserScan::ConstPtr& msg) {
  rear_laser_ = *msg;
  for(int i = 0; i < rear_laser_.ranges.size(); i += 5) {
    const double angle = rear_laser_.angle_min + i * rear_laser_.angle_increment;
    const double range = rear_laser_.ranges[i];
    if((-width_threshold_ < range * sin(angle) && range * sin(angle) < width_threshold_) && range * cos(angle) < distance_threshold_) {
      is_stop_rear_ = true;
    }
  }
}

void ElevatorManagerLidar::plannerCallback(const std_msgs::String::ConstPtr& msg) {
  planner_ = *msg;
  if(planner_.data == "/local_planner/elevator_planner/cmd_vel") {
    elevator_planner_ = true;
  } else {
    elevator_planner_ = false;
  }
}

void ElevatorManagerLidar::imuCallback(const sensor_msgs::Imu::ConstPtr& msg) {
  imu_ = *msg;
  if(imu_.linear_acceleration.z > 9.85) up_count_++;
  else
    up_count_ = 0;
  if(imu_.linear_acceleration.z < 9.75) down_count_++;
  else
    down_count_ = 0;

  ROS_WARN_STREAM("up_count_ : " << up_count_ << ", down_count_ : " << down_count_);
  ROS_ERROR_STREAM("z : " << imu_.linear_acceleration.z);

  if(up_count_ > 10 && up_once_ == false) {
    up_once_ = true;
  } else if(up_count_ > 10 && once_ == true) {
    up_twice_ = true;
  }

  ROS_INFO_STREAM("up_once_ : " << up_once_ << ", down_once_ : " << down_once_);

  if(down_count_ > 10 && down_once_ == false) {
    down_once_ = true;
  } else if(down_count_ > 10 && once_ == true) {
    down_twice_ = true;
  }
}

void ElevatorManagerLidar::stop() {
  cmd_vel_.linear.x = 0.0;
  cmd_vel_.angular.z = 0.0;
  cmd_vel_pub_.publish(cmd_vel_);
}

void ElevatorManagerLidar::straight() {
  if(!is_stop_front_) {
    cmd_vel_.linear.x = 0.1;
    cmd_vel_.angular.z = 0.0;
    if(elevator_planner_)  straight_count_++;
    cmd_vel_pub_.publish(cmd_vel_);
  } else stop();
}

void ElevatorManagerLidar::back() {
  if(!is_stop_rear_) {
    cmd_vel_.linear.x = -0.1;
    cmd_vel_.angular.z = 0.0;
    if(elevator_planner_)  back_count_++;
    cmd_vel_pub_.publish(cmd_vel_);
  } else stop();
}

void ElevatorManagerLidar::process()
{
  ros::Rate rate(hz_);
  while (ros::ok())
  {
    if(up_once_ == false && down_once_ == false && once_ == false) {
      if(is_stop_front_) {
        stop();
        is_stop_front_ = false;
      } else {
        straight();
      }
    } else if(up_once_ == true && down_once_ == true && once_ == false) {
      if(back_count_ < straight_count_) {
        if(is_stop_rear_) {
          stop();
          is_stop_rear_ = false;
        } else {
          back();
        }
      } else if(straight_count_ == back_count_ && straight_count_ != 0) {
        stop();
        upfloor_.data = true;
        once_ = true;
        straight_count_ = 0;
        back_count_ = 0;
        upfloor_pub_.publish(upfloor_);
      }
    }

    if(up_twice_ == false && down_twice_ == false && once_ == true) {
      if(is_stop_front_) {
        stop();
        is_stop_front_ = false;
      } else {
        straight();
      }
    } else if(up_twice_ == true && down_twice_ == true && once_ == true) {
      if(back_count_ < straight_count_) {
        if(is_stop_rear_) {
          stop();
          is_stop_rear_ = false;
        } else {
          back();
        }
      } else if(straight_count_ == back_count_ && straight_count_ != 0) {
        stop();
        downfloor_.data = true;
        twice_ = true;
        straight_count_ = 0;
        back_count_ = 0;
        downfloor_pub_.publish(downfloor_);
      }
    }
    ros::spinOnce();
    rate.sleep();
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "elevator_manager_lidar");
  ElevatorManagerLidar eml;
  eml.process();
  return 0;
}
