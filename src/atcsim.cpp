#include "ros/ros.h"

#include "Airport.cpp"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "atcsim_node");
  ros::NodeHandle n;

  ros::Rate loop_rate(10);

  Airport airport;

  ros::ServiceServer ss_add_flight = n.advertiseService("add_flight", &Airport::addFlight, &airport);
  ros::ServiceServer ss_add_wp = n.advertiseService("add_wp", &Airport::addWp, &airport);
  ros::ServiceServer ss_clear_wp = n.advertiseService("clear_wp", &Airport::clearWp, &airport);

  while (ros::ok())
  {
    airport.doWork();

    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
