#include "ros/ros.h"
#include "atcsim/CommanderService.h"

#include <sys/time.h>
#include <math.h>
#include "Common.h"
#include "Position.cpp"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "generator");

  ros::NodeHandle n;

  ros::ServiceClient client = n.serviceClient<atcsim::CommanderService>("commander_service");

  atcsim::CommanderService srv;

  atcsim::Waypoint wp_req;

  ros::Time obj_ts = ros::Time::now();

  float time_generation = 10;
  int num = 0;
  float reduc = 0.0002; //[TODO]

  while (ros::ok())
  {

    if ((ros::Time::now() - obj_ts).toSec() > time_generation)
    {
      obj_ts = ros::Time::now();


      std::cerr<<"Generate new flight";

      float angle, x, y, z;
    	float bear, inc;
    	char id[6];

    	angle = toRadians((float)(rand() % 360 -0));

    	x = /*fabs*/(AIRPORT_DISTANCE_MAX * cos(angle) * reduc) ; //Only positive, for GyV3D!!!!!!!!!!!
    	y = AIRPORT_DISTANCE_MAX * sin(angle) * reduc;
    	z = (FLIGHT_HEIGHT + (float)(rand() % 2000 )) * reduc;

    	Position ipos(x, y, z);
    	Position pos0(0.0, 0.0, 0.0);

    	pos0.angles(ipos, bear, inc);

    	sprintf(id, "IB%4.4d", num++);

      srv.request.code = 1;
      srv.request.id = id;
      srv.request.posx =  x;
      srv.request.posy =  y;
      srv.request.posz =  z;
      srv.request.speed =  100;
      srv.request.bearing =  bear;
      srv.request.inclination =  0;
      ROS_INFO_STREAM("AVIOOOON: "<< id << bear);

      if (client.call(srv))
        {
          if(!(srv.response.achieved))
            ROS_INFO_STREAM("FAIL: "<<srv.response.expl);


        }
      else
        {
          ROS_ERROR("Failed to call service");
          return 1;
        }

    }
  }
return 0;
}
