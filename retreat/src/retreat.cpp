#include <retreat/retreat.h>

bool Retreat::init()
{
  treats_topic_ =  "/threat_points";
  retreat_done_pub_topic_ = "/retreat_done";
  ee_pose_pub_topic_ = "/ee_pose";

  ref_frame_ = "panda_link0";
  ee_link_ = "panda_link8";
  
  threats_sub_ = nh_.subscribe<project_msgs::Threats>(treats_topic_, 1, &Retreat::threatsCallback, this);
  at_home_pub_ = nh_.advertise<std_msgs::Bool>(retreat_done_pub_topic_, 1);
  ee_pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>(ee_pose_pub_topic_, 1);

  retreat_command_ = false;
  at_home_.data = true;

  arm_group_.setPoseReferenceFrame(ref_frame_);
  arm_group_.setEndEffectorLink(ee_link_);

  arm_group_.setMaxAccelerationScalingFactor(1);
  arm_group_.setMaxVelocityScalingFactor(1);

  vel_scaling_ = 1.0;
  safty_distance_ = 0.5;

  return true;
}

void Retreat::update(ros::Time& curr_time)
{
  current_ee_pose_st_ = arm_group_.getCurrentPose();
  current_ee_pose_st_.header.stamp = curr_time;
  
  current_ee_pose_ = current_ee_pose_st_.pose; 

  Eigen::Vector3d current_ee_position(current_ee_pose_.position.x, current_ee_pose_.position.y, current_ee_pose_.position.z);
  current_ee_position_ = current_ee_position;
  if (retreat_command_)
  {
    retreat();
    at_home_pub_.publish(at_home_);
    retreat_command_ = false;
  }

  ee_pose_pub_.publish(current_ee_pose_st_);
}

void Retreat::retreat()
{
  preProcess();
  retreatMotion();
  // homing();
}

void Retreat::preProcess()
{
  // Algorithm is demostrated in matalb, see /test/vector_test.mlx
  std::vector<Eigen::Vector3d> vec_point_ee, uvec_point_ee, weighted_uvec;
  std::vector<double> dist_point_ee, scaler_point_vel, angle_vel_position , effective_vel, w_dist, w_vel;

  vec_point_ee.resize(point_num_);
  dist_point_ee.resize(point_num_);

  scaler_point_vel.resize(point_num_);
  angle_vel_position.resize(point_num_);
  effective_vel.resize(point_num_);

  w_dist.resize(point_num_);
  w_vel.resize(point_num_);

  uvec_point_ee.resize(point_num_);
  weighted_uvec.resize(point_num_);

  Eigen::Vector3d retreat_vec = Eigen::Vector3d::Zero();

  // Get essential factors
  for (int i = 0; i < point_num_; ++i)
  {
    vec_point_ee[i] = current_ee_position_ - point_position_[i];
    dist_point_ee[i] = vec_point_ee[i].norm();

    scaler_point_vel[i] = point_vel_[i].norm();
    angle_vel_position[i] = acos(point_vel_[i].dot(vec_point_ee[i]) / (point_vel_[i].norm() * vec_point_ee[i].norm()));
    effective_vel[i] = cos(angle_vel_position[i]) * scaler_point_vel[i];

    w_dist[i] = 15 * exp(-dist_point_ee[i]);

    int sign = 0;
    if (effective_vel[i]>0)
    {
      sign = 1;
    }
    else
    {
      sign = -1;
    }
    w_vel[i] = sign * 5.0017 * log(abs(effective_vel[i]));

    uvec_point_ee[i] = vec_point_ee[i] / dist_point_ee[i];
    weighted_uvec[i] = uvec_point_ee[i] * (w_dist[i] + w_vel[i]);

    retreat_vec += weighted_uvec[i];
  }

  Eigen::Vector3d retreat_dist = safty_distance_ * retreat_vec;

  retreat_goal_ = current_ee_pose_;
  retreat_goal_.position.x += retreat_dist[0]; 
  retreat_goal_.position.y += retreat_dist[1]; 
  retreat_goal_.position.z += retreat_dist[2];  
}

void Retreat::retreatMotion()
{
  std::vector<geometry_msgs::Pose> waypoints;
  waypoints.push_back(retreat_goal_);

  moveit_msgs::RobotTrajectory trajectory;

  double eef_step = 0.01;  // Resolution of the Cartesian path
  double jump_threshold = 0.0;  // No jump threshold

  double fraction = arm_group_.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);

  arm_group_.execute(trajectory);

  ROS_INFO_STREAM("Retreated");
}

void Retreat::homing()
{
  arm_group_.setStartStateToCurrentState();
  arm_group_.setNamedTarget("ready");

  moveit::planning_interface::MoveGroupInterface::Plan home_plan;
  arm_group_.plan(home_plan);
  
  arm_group_.move();
}

void Retreat::threatsCallback(const project_msgs::Threats::ConstPtr &msg)
{
  point_num_ = msg->num;
  point_position_.resize(point_num_);
  point_vel_.resize(point_num_);

  for (int i = 0; i < point_num_; ++i)
  {
    Eigen::Vector3d curr_point_position(msg->position[i].x, msg->position[i].y, msg->position[i].z);
    Eigen::Vector3d curr_point_vel(msg->vel[i].x, msg->vel[i].y, msg->vel[i].z);

    point_position_[i] = curr_point_position;
    point_vel_[i] = curr_point_vel;

    ROS_INFO_STREAM("point velocity: " << curr_point_vel.transpose());
  }

  retreat_command_ = true;
}