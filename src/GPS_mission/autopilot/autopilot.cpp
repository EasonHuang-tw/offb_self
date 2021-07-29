#include <iostream>
#include<thread>
#include<vector>
#include "autopilot.h"

#define error 0.1



autopilot::autopilot(gps_transform gps){
	this->gps = gps;
	double *pose_now = new double[3];
	double *target_now = new double[3];
	mission = 0;
	land_ok = false;
	

}

void autopilot::update(double *recent_pose){
	pose_now[0] = recent_pose[0];
	pose_now[1] = recent_pose[1];
	pose_now[2] = recent_pose[2];
	
	if (get_mission() == 0){
		takeoff();
		if(is_arrived_z() == true){
			mission = 1;
			this->gps.update(47.3977545,8.5457408,535.5597166987343);
			this->gps.get_ENU(target_now);
			target_now[2] = 1;
		}
	}

	if(get_mission() == 1){
		
		if(is_arrived_xy() == true){
			mission = 2;
			this->gps.update(47.3978816,8.5459172,535.8185302211843);
			this->gps.get_ENU(target_now);
			target_now[2] = 1;
		}
	}
	if(get_mission() == 2){
		if(is_arrived_xy() == true){
			mission = 3;
		}
	}
	if(get_mission() == 3){
		if(is_arrived_xy() == true){
			mission = 4;
		}
	}
	if (get_mission() == 4){
		land();
		if(is_arrived_z() == true){
			land_ok = true;
		}
	}
}

void autopilot::set_position(double *msg){
	
}

void  autopilot::takeoff(){
	target_now[0] = 0;
	target_now[1] = 0;
	target_now[2] = 1;
}

void autopilot::land(){
	target_now[0] = pose_now[0];
	target_now[1] = pose_now[1];
	target_now[2] = 0;
}

bool autopilot::is_arrived_xy(){
	if(abs(target_now[0]-pose_now[0])<error && abs(target_now[1]-pose_now[1])<error){
		return true;
	}else{
		return false;
	}
}
bool autopilot::is_arrived_z(){
	if(abs(target_now[2]-pose_now[2])<error){
		return true;
	}else{
		return false;
	}
}
bool autopilot::get_land_ok(){
	return land_ok;
}

int autopilot::get_mission(){
	return mission;
}

double* autopilot::get_target_now(){
	return target_now;
}
