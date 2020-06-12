/*
 * Flight.cpp
 *
 *  Created on: 15/07/2014
 *      Author: paco
 *
 *  Copyright 2014 Francisco Martín
 *
 *  This file is part of ATCSim.
 *
 *  ATCSim is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  ATCSim is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with ATCSim.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "std_msgs/String.h"

#include "Flight.h"

#ifdef __APPLE__
#include <OpenGL/gl.h>
#include <OpenGL/glu.h>
#else
#include <GL/glu.h>
#include <GL/gl.h>
#endif

#include "Common.h"

#include <iostream>
#include <string>
#include <math.h>





Flight::~Flight() {
	// TODO Auto-generated destructor stub
}

Flight::Flight(std::string _id, Position _pos, float _bearing, float _inclination, float _speed)
{
	id = _id;
	pos = _pos;
	bearing = _bearing;
	init_bearing = _bearing;
	inclination = _inclination;
	//speed = _speed;
	setSpeed(_speed);	// Through set in order to limit speeds

	route.clear();
	inStorm = false;

	focused = false;
	points = INIT_FLIGHT_POINTS;

	w_speed = 0.0f;
}

void
Flight::update(float delta_t)
{
	float trans;
	Position CPpos;
	float distXY_CP = 1.0f; //[TODO]

	float reduc = 0.02; //REDUCTOR DE VELOCIDAD [ TODO ]

	if(routed())
	{
		float goal_bearing, diff_bearing, new_w;

		CPpos = route.front().pos;

		if(CPpos.get_z() <= MAINTAIN_ALT){ // Maintain altitude
			float current_alt = (this->getPosition()).get_z();
			CPpos.set_z(current_alt);
			route.front().pos.set_z(current_alt);
		}

		pos.angles(CPpos, goal_bearing, inclination);

		goal_bearing = normalizePi(goal_bearing + M_PI);
		diff_bearing = normalizePi(goal_bearing - bearing);
		new_w = diff_bearing;

		if(fabs(new_w)>MAX_FLIFGT_W) new_w = (fabs(new_w)/new_w) * MAX_FLIFGT_W;


		bearing = bearing + new_w*delta_t*50; //TODO Factor de aumento para que gire normal"

		float goal_speed, diff_speed, acc;

		//mantener la velocidad durante el giro
		if(fabs(new_w) < MAX_FLIFGT_W*0.9){
			goal_speed = checkSpeedLimits(route.front().speed);
		}else{
			goal_speed = speed;
		}
		acc = (goal_speed - speed);

		if(fabs(acc)>MAX_ACELERATION) acc = (acc/fabs(acc))*MAX_ACELERATION;

		speed = speed + acc*delta_t*50;

		//gire antes de llegar al CP
		/*if(route.size()>=2){
			Position CPpos_next;
			std::list<Route>::iterator it = route.begin();
			CPpos_next = (++it)->pos;

			float alpha;
			if(0.0 <= pos.distX_EjeBody(CPpos, CPpos_next)){ //saber si el CPpos_next esta adelante o atras del CPpos
				alpha = M_PI - pos.get_angle(CPpos, CPpos_next);
			}else{
				alpha = pos.get_angle(CPpos, CPpos_next);
			}

			distXY_CP = fabs(speed / (MAX_FLIFGT_W * tan(alpha/2)));
		}*/



		if(pos.distance(CPpos)<DIST_POINT*reduc*0.01 /*|| pos.distanceXY(CPpos)<=distXY_CP*/){
			route.pop_front();
			ROS_INFO_STREAM("POPPING "<< id);
		}


	}else{
		inclination = 0.0;
	}

	last_pos = pos;

	trans = speed*reduc * delta_t;


	pos.set_x(pos.get_x() + trans * cos(bearing) * cos(inclination));
	pos.set_y(pos.get_y() + trans * sin(bearing) * cos(inclination));
	pos.set_z(pos.get_z() + ( trans * sin(inclination)));


}


float Flight::checkSpeedLimits(float tgt_speed){
	return (tgt_speed > CRASH_SPEED_MAX ? CRASH_SPEED_MAX : tgt_speed);
}
