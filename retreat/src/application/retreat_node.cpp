#include <retreat/retreat.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "retreat_node");

  ros::AsyncSpinner spinner(1);
  spinner.start();

  Retreat retreat;

  retreat.init();

  ros::Rate loop_rate(10);
  loop_rate.sleep();

  while (ros::ok())
  { 
    ros::Time curr_time = ros::Time::now();
    retreat.update(curr_time);
    loop_rate.sleep();
  }

  ros::shutdown;
  return 0;
}