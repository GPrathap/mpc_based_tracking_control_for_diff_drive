#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>

class PathSampling
{

private:
  ros::NodeHandle nh_;
  ros::Subscriber robot_odom_;
  ros::Publisher traversed_path_pub_;
  nav_msgs::Path traversed_path_;
  ros::Time last_time_;

public:
  PathSampling(ros::NodeHandle &nh): nh_(nh)
  {
    robot_odom_ = nh_.subscribe<nav_msgs::Odometry>("/odometry/filtered"
                                , 5, &PathSampling::odomCallback, this);
    traversed_path_pub_ = nh_.advertise<nav_msgs::Path>("/husky/traversed_path", 20);
    last_time_ = ros::Time::now();
  }

  void odomCallback(const nav_msgs::OdometryConstPtr& odom){
      ros::Time current_time = ros::Time::now();
      ros::Duration diff = current_time - last_time_;
      double dis = 2.0;
      if(traversed_path_.poses.size()>0){
        geometry_msgs::PoseStamped last_index = traversed_path_.poses.back();
        dis = std::sqrt(std::pow(last_index.pose.position.x-odom->pose.pose.position.x, 2) 
          + std::pow(last_index.pose.position.y-odom->pose.pose.position.y, 2));
      }
      if(diff.toSec()>1.0 && dis > 1.0){
        last_time_ = current_time;
        geometry_msgs::PoseStamped current_pose;
        // current_pose.header.stamp = odom->header.stamp;
        current_pose.header.frame_id = odom->header.frame_id;
        current_pose.pose.position.x = odom->pose.pose.position.x;
        current_pose.pose.position.y = odom->pose.pose.position.y;
        current_pose.pose.position.z = odom->pose.pose.position.z;
        traversed_path_.poses.push_back(current_pose);
        traversed_path_.header.stamp = odom->header.stamp;
        traversed_path_.header.frame_id = odom->header.frame_id;
        traversed_path_pub_.publish(traversed_path_);
      }
  }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "path_sampling");
  ros::NodeHandle nh_;
  PathSampling path_sampler(nh_);
  ros::spin();
}