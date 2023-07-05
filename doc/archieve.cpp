std::vector<Eigen::Vector3d> point_vector;
  std::vector<double> distance_ee_point;

  point_vector.resize(msg->num);
  distance_ee_point.resize(msg->num);

  std::vector<Eigen::Vector3d> point_vel;       // The velocity of each point
  std::vector<double> scaler_vel;               // The scaler value of velocities
  std::vector<double> vel_angle;                // The angle between the "point_vector" and velocity
  std::vector<double> effctive_vel;             // The velocity direct to the ee
  
  point_vel.resize(msg->num);
  scaler_val.resize(msg->num);
  vel_angle.resize(msg->num);

  // Get the essential factors
  for (int i = 0; i < msg->num; ++i)
  {
    Eigen::Vector3d curr_point(msg->position[i].x, msg->position[i].y, 0);
    point_vector[i] = current_ee_position_ - curr_point;
    distance_ee_point[i] = point_vector[i].norm();
  }

  for (int i = 0; i < msg->num; ++i)
  {
    Eigen::Vector3d curr_vel(msg->vel[i].x, msg->vel[i].y, msg->vel[i].z);
    point_vel[i] = curr_vel;
    scaler_vel[i] = point_vel.norm();
    vel_angle[i] = acos(point_vel[i].transpose()*point_vec[i]/(point_vel[i].norm()*point_vec[i].norm()));
    effective_vel[i] = cos(vel_angle[i])*scaler_vel[i];
  }

  // Get retreat parameters
  std::vector<double> w_dist, w_vel;          // Weight of distance and velocity
  w_dist.resize(msg->num);
  w_vel.resize(msg->num);

  for (int i = 0; i < msg->num; ++i)
  {
  }