#include "ros/ros.h"

#include "Airport.cpp"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "atcsim_node");
  ros::NodeHandle n;

  ros::Rate loop_rate(10);

  Airport airport;

  ros::ServiceServer ss = n.advertiseService("commander_service", &Airport::command, &airport);

  while (ros::ok())
  {
    airport.doWork();

    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
