# include "elevator_manager/elevator_manager_depth.h"

ElevatorManagerDepth::ElevatorManagerDepth():private_nh_("~") {
  private_nh_.param<int>("hz", hz_, 10);
  private_nh_.param<float>("max_height", max_height_, 0.5);

  cloud_sub_ = nh_.subscribe<sensor_msgs::PointCloud2>("/realsense/depth/color/points", 1, &ElevatorManagerDepth::cloudCallback, this);
  cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/elevator_manager/filtered_cloud", 1);
}

ElevatorManagerDepth::~ElevatorManagerDepth() {
}

void ElevatorManagerDepth::cloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg) {
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg(*msg, *cloud);

  // tf::StampedTransform transform;
  // try {
  //   tf_listener_.lookupTransform("base_link", "realsense_link", ros::Time(0), transform);
  // } catch (tf::TransformException ex) {
  //   ROS_ERROR("%s", ex.what());
  //   return;
  // }

  // Eigen::Matrix4f transform_matrix;
  // pcl_ros::transformAsMatrix(transform, transform_matrix);
  // pcl::PointCloud<pcl::PointXYZ>::Ptr detected_points(new pcl::PointCloud<pcl::PointXYZ>);

  // bool pointAboveThreshold = false;

  for (const auto& point : cloud->points)
  {
    // Eigen::Vector4f point_vector(point.x, point.y, point.z, 1.0);
    // Eigen::Vector4f transformed_point = transform_matrix * point_vector;

    // if (transformed_point[0] > max_height_)
    if (point.z > max_height_)
    {
      // pointAboveThreshold = true;
      // break;
      cloud->push_back(point);
    }
  }

  // if (!detected_points->empty()) {
  if (!cloud->empty()) {
    ROS_INFO("Point(s) above height threshold found.");
    sensor_msgs::PointCloud2 filtered_cloud;
    // pcl::toROSMsg(*detected_points, filtered_cloud);
    pcl::toROSMsg(*cloud, filtered_cloud);
    filtered_cloud.header = msg->header;
    cloud_pub_.publish(filtered_cloud);
  } 
  else {
    ROS_INFO("No points above height threshold.");
  }
}

void ElevatorManagerDepth::process() {
  ros::Rate rate(hz_);

  while (ros::ok()) {
    ros::spinOnce();
    rate.sleep();
  }
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "elevator_manager");
  ElevatorManagerDepth elevator_manager;
  elevator_manager.process();
  return 0;
}