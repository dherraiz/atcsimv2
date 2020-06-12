#include "ros/ros.h"
#include "std_msgs/String.h"
#include "atcsim/ArrayFlight.h"
#include <cstdlib>

#include "tf2/transform_datatypes.h"
#include "tf2/LinearMath/Transform.h"
#include "geometry_msgs/TransformStamped.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2/convert.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2/LinearMath/Quaternion.h"

#include <sstream>
#include <string>

#include <list>
#include "Flight.cpp"

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


int main(int argc, char **argv)
{

  ros::init(argc, argv, "airport");

  ros::NodeHandle n;

  ros::Publisher arrayflight_info_pub = n.advertise<atcsim::ArrayFlight>("arrayflight_info", 1000); //Declarar publicación de info
  atcsim::ArrayFlight arrayflight_pub;
  atcsim::Flight flight_pub;
  atcsim::Waypoint wp_pub;

  ros::Rate loop_rate(10);

  std::list<Flight*> flights;
  std::list<Flight*>::iterator it;
  std::list<Route>::iterator it_wps;

  tf2_ros::TransformBroadcaster br; // Declarar tf
  geometry_msgs::TransformStamped flight_obj_msg;

  ros::Time obj_ts = ros::Time::now();

  Position pos0(0.0, 2.0, 0.3); //------------------------------------------------------------Creando Wps
  Position pos1(0.0, 1.0, 0.15);
  Position pos2(0.0, 0.5, 0.05);
  Position pos3(0.0, 0.0, 0.0);
  Route r0, r1, r2, r3;

  r0.pos = pos0;
  r0.speed = 100;
  r1.pos = pos1;
  r1.speed = 75;
  r2.pos = pos2;
  r2.speed = 50;
  r3.pos = pos3;
  r3.speed = 50;

  for (int i = 0; i < 3; i++) {
    float angle;
	  float bear, inc;
	  char id[6];
    Position ipos(-1+i, 3+i, 1);
  	Position pos0(0.0, 0.0, 0.0);

  	pos0.angles(ipos, bear, inc);
    sprintf(id, "IB%d", i);
  	Flight *aux;
  	aux = new Flight(id, ipos, bear, 0.0, 200.0);

    aux->getRoute()->push_back(r3);
		aux->getRoute()->push_front(r2);
		aux->getRoute()->push_front(r1);
		aux->getRoute()->push_front(r0);

  	flights.push_back(aux);
  }
//----------------------------------------------------------------------------


  while (ros::ok())
  {

    if ((ros::Time::now() - obj_ts).toSec() > INC_SIMTIME/50) /// ACTUALIZAR LA POSICIÓN DE LOS AVIONES Y PUBLICACIÓN DE ArrayFlight
    {
      obj_ts = ros::Time::now();
      arrayflight_pub.array_flight.clear();
      for(it = flights.begin(); it!=flights.end(); ++it)
      {
        (*it)->update(INC_SIMTIME/50);

        flight_pub.id = (*it)->getId();/// TODO: Publicar TODA la información
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
      arrayflight_info_pub.publish(arrayflight_pub);
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


    ros::spinOnce();
    loop_rate.sleep();
  }



  return 0;
}
