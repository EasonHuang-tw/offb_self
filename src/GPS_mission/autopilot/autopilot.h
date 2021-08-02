#ifndef AUTOPILOT
#define AUTOPILOT
#include <math.h>
#include "gps_transform.h"


enum autopilot_state{not_flying,takeoff,pose,waypoint,land};

class autopilot{
	public:
		autopilot(gps_transform gps);
		
		void update(double *recent_pose);
		
		//add waypoint by different datatype
		void add_waypoint(sensor_msgs::NavSatFix);
		void add_waypoint(double*);
		void add_waypoint(double latitude,double longitude ,double altitude);
		
		void show_waypoints();
		
		void takeoff();
		void land();
		void mission_start();
		void mission_stop();

		bool is_arrived_xy();
		bool is_arrived_z();
		bool get_land_ok();
		
		autopilot_state get_state();
		
		double* get_target_now();
	private:
		std::vector<sensor_msgs::NavSatFix> waypoints;
		autopilot_state state;
		int waypoint_num = 0;	
		
		double target_now[3];
		double pose_now[3];
		double pose_start[3];
		
		bool land_ok;
		bool in_mission = false;
		gps_transform gps;
};
#endif
