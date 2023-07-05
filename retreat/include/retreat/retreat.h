#ifndef RETREAT_H
#define RETREAT_H

#include <ros/ros.h>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <std_msgs/Bool.h>

#include <project_msgs/Threats.h>

#include <cmath>
#include <math.h>
#include <cstdlib>

#include <Eigen/Geometry>

class Retreat
{
  private:
    //
    moveit::planning_interface::MoveGroupInterface arm_group_;

  public:
    Retreat(): arm_group_("panda_arm") {};
  
  public:
    bool init();
    void update(ros::Time& curr_time);
  
  private:
    void retreat();

    void preProcess();
    void retreatMotion();
    void homing();
  
  // For ROS node
  private:
    ros::NodeHandle nh_;
    ros::Subscriber threats_sub_;        // Get the direction of staight motion, save to "derection_"
    ros::Publisher at_home_pub_;     // Publish to tell the retreatment is done
    ros::Publisher ee_pose_pub_;

    typedef geometry_msgs::Vector3 subscribe_type_;   // Type might be modified

    // Topic names
    std::string treats_topic_;
    std::string retreat_done_pub_topic_;
    std::string ee_pose_pub_topic_;
    
    // Necessary variables
    bool retreat_command_;
    std_msgs::Bool at_home_;

    // Callback
    void threatsCallback(const project_msgs::Threats::ConstPtr &msg);
    std::vector<Eigen::Vector3d> point_position_;
    std::vector<Eigen::Vector3d> point_vel_;
    int point_num_;
  // For computation
  private:
    std::string ref_frame_;
    std::string ee_link_;
    // Save the current pose of end effector
    geometry_msgs::PoseStamped current_ee_pose_st_;
    geometry_msgs::Pose current_ee_pose_;
    Eigen::Vector3d current_ee_position_;

  private:
    double vel_scaling_;
    geometry_msgs::Pose retreat_goal_;     // Goal for straight line motion
    double safty_distance_;
    
};

#endif