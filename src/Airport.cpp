
#include "std_msgs/String.h"
#include "atcsim/AddFlight.h"
#include "atcsim/AddWp.h"
#include "atcsim/ClearWp.h"
#include "atcsim/CommanderService.h"
#include <cstdlib>


#include <sstream>
#include <string>

#include "atcsim/ArrayFlight.h"

#include "tf2/transform_datatypes.h"
#include "tf2/LinearMath/Transform.h"
#include "geometry_msgs/TransformStamped.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2/convert.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2/LinearMath/Quaternion.h"

#include "Airport.h"

geometry_msgs::TransformStamped generate_flight_obj(Position pos, float bearing, float inclination, std::string name_flight)
{
  tf2::Stamped<tf2::Transform> object;
  object.frame_id_ = "world";
  object.stamp_ = ros::Time::now();

  object.setOrigin(tf2::Vector3(pos.get_x(), pos.get_y(), pos.get_z()));

  tf2::Quaternion q;
  q.setRPY(0, -inclination, bearing);
  object.setRotation(q);

  geometry_msgs::TransformStamped object_msg = tf2::toMsg(object);
  object_msg.child_frame_id = name_flight;

  return object_msg;
}





void
Airport::doWork()
{
  std::list<boost::shared_ptr<Flight> >::iterator it;
  //std::list<Flight*>::iterator it;
  std::list<Route>::iterator it_wps;
  std::list<boost::shared_ptr<atcsim::Waypoint> >::iterator it_wps_req;
  //std::list<atcsim::Waypoint*>::iterator it_wps_req;


  atcsim::ArrayFlight arrayflight_pub;
  atcsim::Flight flight_pub;
  atcsim::Waypoint wp_pub;

  ros::Publisher arrayflight_info_pub_;

  arrayflight_info_pub_ = n_.advertise<atcsim::ArrayFlight>("arrayflight_info", 1000);

  ros::Rate loop_rate(10);


  if ((ros::Time::now() - obj_ts_).toSec() > INC_SIMTIME/50) /// ACTUALIZAR LA POSICIÓN DE LOS AVIONES Y PUBLICACIÓN DE ArrayFlight
  {
    obj_ts_ = ros::Time::now();
    arrayflight_pub.array_flight.clear();
    for(it = flights_.begin(); it!=flights_.end(); ++it)
    {
      (*it)->update(INC_SIMTIME/50);

      flight_pub.id = (*it)->getId();
      flight_pub.posx = (*it)->getPosition().get_x();
      flight_pub.posy = (*it)->getPosition().get_y();
      flight_pub.posz = (*it)->getPosition().get_z();
      flight_pub.speed = (*it)->getSpeed();
      flight_pub.bearing = (*it)->getBearing();
      flight_pub.inclination = (*it)->getInclination();

      flight_pub.wps.clear();
      for(it_wps = (((*it)->getRoute())->begin()); it_wps!=((*it)->getRoute())->end(); ++it_wps){

        wp_pub.x = it_wps->pos.get_x();
        wp_pub.y = it_wps->pos.get_y();
        wp_pub.z = it_wps->pos.get_z();
        wp_pub.speed = it_wps->speed;

        flight_pub.wps.push_back(wp_pub);
      }

      arrayflight_pub.array_flight.push_back(flight_pub);
    }
    //ROS_INFO("AADD");
    arrayflight_info_pub_.publish(arrayflight_pub);
  }///-------------------------------------------------------------------------------------------


  for(it = flights_.begin(); it!=flights_.end(); ++it)///-- Publicación de transformadas
  {

    flight_obj_msg = generate_flight_obj((*it)->getPosition(), (*it)->getBearing(), (*it)->getInclination(), (*it)->getId());
    try
    {
      flight_obj_msg.header.stamp = ros::Time::now();
      br.sendTransform(flight_obj_msg);
    }
    catch(tf2::TransformException &exception)
    {
      ROS_ERROR("04");
      ROS_ERROR("%s", exception.what());
    }
  }//---------------------------------------------------------------------------

}



bool
Airport::command(atcsim::CommanderService::Request& req,
         atcsim::CommanderService::Response& res)
{
  bool ach = false;
  std::list<boost::shared_ptr<Flight> >::iterator it;
  //std::list<Flight*>::iterator it;
  std::list<Route>::iterator it_wps;
  std::list<boost::shared_ptr<atcsim::Waypoint> >::iterator it_wps_req;
  //std::list<atcsim::Waypoint*>::iterator it_wps_req;

  ROS_INFO("RECIBIDOO");
  if (req.code == 1) //AddFlight
  {
    ach = true;
    for(it = flights_.begin(); it!=flights_.end(); ++it) //Coomprobar que no hay otro avion con el mismo nombre
    {
      if (((*it)->getId()) == req.id){
        ach = false;
      }
    }

    if(ach)
    {
      Position ipos(req.posx, req.posy, req.posz);
      //Flight *aux;
      boost::shared_ptr<Flight> aux(new Flight(req.id, ipos, req.bearing, req.inclination, req.speed));
      //aux = new Flight(req.id, ipos, req.bearing, req.inclination, req.speed);

      for(int i=0; i< req.wps.size(); ++i)
      {
        Position pos0(req.wps[i].x, req.wps[i].y, req.wps[i].z);
        Route r0;

        r0.pos = pos0;
        r0.speed = req.wps[i].speed;
        aux->getRoute()->push_back(r0);
      }
      flights_.push_back(aux);
    }

    res.achieved = ach;
     if(ach){
       ROS_INFO_STREAM(req.id <<" added to the list");
     }else{
       res.expl = "Flight " + req.id + " already exists";
       return 1;
     }


    return true;
  }else if(req.code == 2) //AddWp
  {
      for(it = flights_.begin(); it!=flights_.end(); ++it)
      {
        if (((*it)->getId()) == req.id){
          Position pos0(req.wps[1].x, req.wps[1].y, req.wps[1].z);
          Route r0;
          r0.pos = pos0;
          r0.speed = req.wps[1].speed;
          (*it)->getRoute()->push_back(r0);
          ach = true;
        }
      }
      res.achieved = ach;
       if(ach){
         ROS_INFO_STREAM("Waypoint: ["<< req.wps[1].x<<", "<<
                         req.wps[1].y<< " , "<<req.wps[1].z<<
                         "] speed: "<<req.wps[1].speed <<" added to the list of "<<req.id);
       }else{
         res.expl = "Flight " + req.id + " not found";
         return 1;
       }

      return true;
  }else if(req.code == 3) //ClearWps
  {
    for(it = flights_.begin(); it!=flights_.end(); ++it)
    {
      if (((*it)->getId()) == req.id){
        (*it)->getRoute()->clear();
        ach = true;
      }
    }
    res.achieved = ach;
    if(ach){
      ROS_INFO_STREAM("Erased all the wps of "<<req.id);
    }else{
       res.expl = "Flight " + req.id + " not found";
       return 1;
    }
    return true;
  }else{
    return 0;
  }

}
