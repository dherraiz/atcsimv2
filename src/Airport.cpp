
#include "std_msgs/String.h"
#include "atcsim/AddFlight.h"
#include "atcsim/AddWp.h"
#include "atcsim/ClearWp.h"
#include <cstdlib>


#include <sstream>
#include <string>



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

  ros::Rate loop_rate(10);

  if ((ros::Time::now() - obj_ts).toSec() > INC_SIMTIME/50) /// ACTUALIZAR LA POSICIÓN DE LOS AVIONES Y PUBLICACIÓN DE ArrayFlight
  {
    obj_ts = ros::Time::now();
    arrayflight_pub.array_flight.clear();
    for(it = flights.begin(); it!=flights.end(); ++it)
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
    arrayflight_info_pub_.publish(arrayflight_pub);
  }///-------------------------------------------------------------------------------------------


  for(it = flights.begin(); it!=flights.end(); ++it)///-- Publicación de transformadas
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
Airport::addFlight(atcsim::AddFlight::Request& req,
         atcsim::AddFlight::Response& res)
 {
   ROS_INFO_STREAM(req.id <<" added to the list");

   Position ipos(req.posx, req.posy, req.posz);
   Flight *aux;
   aux = new Flight(req.id, ipos, req.bearing, req.inclination, req.speed);

   for(int i=0; i< req.wps.size(); ++i)
   {
     Position pos0(req.wps[i].x, req.wps[i].y, req.wps[i].z);
     Route r0;

     r0.pos = pos0;
     r0.speed = req.wps[i].speed;
     aux->getRoute()->push_back(r0);
   }


   flights.push_back(aux);

   return true;

 }

bool
Airport::addWp(atcsim::AddWp::Request& req,
          atcsim::AddWp::Response& res)
{

  bool added = false;

  for(it = flights.begin(); it!=flights.end(); ++it)
  {
    if (((*it)->getId()) == req.id){
      Position pos0(req.wps.x, req.wps.y, req.wps.z);
      Route r0;
      r0.pos = pos0;
      r0.speed = req.wps.speed;
      (*it)->getRoute()->push_back(r0);
      added = true;
    }
  }
   if(added){
     ROS_INFO_STREAM("Waypoint: ["<< req.wps.x<<", "<<
                     req.wps.y<< " , "<<req.wps.z<<
                     "] speed: "<<req.wps.speed <<" added to the list of "<<req.id);
   }else{
     ROS_INFO_STREAM("Flight "<<req.id<<" not found");
   }

  return true;
}

bool
Airport::clearWp(atcsim::ClearWp::Request& req,
         atcsim::ClearWp::Response& res)
{

  bool erased = false;

  for(it = flights.begin(); it!=flights.end(); ++it)
  {
    if (((*it)->getId()) == req.id){

      (*it)->getRoute()->clear();
      erased = true;
    }
  }
   if(erased){
    ROS_INFO_STREAM("Erased all the wps of "<<req.id);

   }else{
     ROS_INFO_STREAM("Flight "<<req.id<<" not found");
   }

  return true;


  return true;
}
