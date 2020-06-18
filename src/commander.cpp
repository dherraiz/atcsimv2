#include "ros/ros.h"
#include "atcsim/AddFlight.h"
#include "atcsim/AddWp.h"
#include "atcsim/ClearWp.h"
#include "atcsim/CommanderService.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "commander");

  ros::NodeHandle n;

  ros::ServiceClient client = n.serviceClient<atcsim::CommanderService>("commander_service");

  atcsim::CommanderService srv;

  atcsim::Waypoint wp_req;

  if (argc<1){
    ROS_INFO("invalid command");
    return 1;
  }

  if (std::string(argv[1])== "add_flight"){

      int num_wps = (argc-9)/4;

      if ((argc-9)%4 != 0)
      {
        ROS_INFO("usage:add_flight id (pos) x y z speed(~200) bearing inclination (wps1) x y z speed (wps2) ...");
        //rosrun atcsim commander add_flight IB001 -4 0 1 200 0 0 -1 0 0.1 100 0 0 0 100
        return 1;
      }

      srv.request.code = 1;
      srv.request.id = (argv[2]);
      srv.request.posx =  atof(argv[3]);
      srv.request.posy =  atof(argv[4]);
      srv.request.posz =  atof(argv[5]);
      srv.request.speed =  atof(argv[6]);
      srv.request.bearing =  atof(argv[7]);
      srv.request.inclination =  atof(argv[8]);

      for(int i= 0; i<num_wps; ++i)
      {
        wp_req.x = atof(argv[9+i*4]);
        wp_req.y = atof(argv[10+i*4]);
        wp_req.z = atof(argv[11+i*4]);

        wp_req.speed = atof(argv[12+i*4]);
        srv.request.wps.push_back(wp_req);
      }

  }else if (std::string(argv[1])== "add_wp"){

    if (argc != 7)
      {
        ROS_INFO("usage: add_wps id (pos) x y z speed");
        //rosrun atcsim commander add_wp IB001 0 0 0.5 100
        return 1;
      }
    srv.request.code = 2;
    srv.request.id = (argv[2]);
    wp_req.x = atof(argv[3]);
    wp_req.y = atof(argv[4]);
    wp_req.z = atof(argv[5]);
    wp_req.speed = atof(argv[6]);
    srv.request.wps.push_back(wp_req);

  }else if (std::string(argv[1])== "clear_wp"){

    if (argc != 3)
      {
        ROS_INFO("usage: clear_wps id");
        //rosrun atcsim commander clear_wp IB001
        return 1;
      }
    srv.request.code = 3;
    srv.request.id = (argv[2]);

  }else{
    ROS_ERROR("Invalid Command. Use: add_flight, add_wp, clear_wp");
        return 1;
  }

  if (client.call(srv))
    {
      if(!(srv.response.achieved))
        ROS_INFO_STREAM("FAIL: "<<srv.response.expl);

      return 0;
    }
  else
    {
      ROS_ERROR("Failed to call service");
      return 1;
    }



}
