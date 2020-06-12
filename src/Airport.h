
#include "ros/ros.h"
#include <list>

#include "atcsim/ArrayFlight.h"

#include "tf2/transform_datatypes.h"
#include "tf2/LinearMath/Transform.h"
#include "geometry_msgs/TransformStamped.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2/convert.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2/LinearMath/Quaternion.h"

#include "Flight.cpp"

class Airport {
public:
	Airport(): n_() {
    arrayflight_info_pub_ = n_.advertise<atcsim::ArrayFlight>("arrayflight_info", 1000);
    obj_ts = ros::Time::now();

  };
  void doWork();
  bool addFlight(atcsim::AddFlight::Request& req,
           atcsim::AddFlight::Response& res);

  bool addWp(atcsim::AddWp::Request& req,
           atcsim::AddWp::Response& res);

  bool clearWp(atcsim::ClearWp::Request& req,
           atcsim::ClearWp::Response& res);

private:

  std::list<Flight*> flights;
  std::list<Flight*>::iterator it;
  std::list<Route>::iterator it_wps;
  std::list<atcsim::Waypoint*>::iterator it_wps_req;


  tf2_ros::TransformBroadcaster br;
  geometry_msgs::TransformStamped flight_obj_msg;

  ros::Time obj_ts;

  atcsim::ArrayFlight arrayflight_pub;
  atcsim::Flight flight_pub;
  atcsim::Waypoint wp_pub;

  ros::NodeHandle n_;
  ros::Publisher arrayflight_info_pub_;



};
