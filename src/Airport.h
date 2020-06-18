
#include "ros/ros.h"
#include <iostream>
#include <list>
#include <memory>

#include "Flight.cpp"

class Airport {
public:
	Airport(): n_() {

    obj_ts_ = ros::Time::now();
		arrayflight_info_pub_ = n_.advertise<atcsim::ArrayFlight>("arrayflight_info", 1000);
  };
  void doWork();

	bool command(atcsim::CommanderService::Request& req,
					atcsim::CommanderService::Response& res);

private:

  //std::list<Flight*> flights_;
	std::list<boost::shared_ptr<Flight> > flights_;

	ros::Time obj_ts_;

  ros::NodeHandle n_;
	ros::Publisher arrayflight_info_pub_;

	tf2_ros::TransformBroadcaster br;
	geometry_msgs::TransformStamped flight_obj_msg;



};
