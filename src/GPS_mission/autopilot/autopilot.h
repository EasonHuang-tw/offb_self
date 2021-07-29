#ifndef AUTOPILOT
#define AUTOPILOT
#include <math.h>
#include "gps_transform.h"

class autopilot{
	public:
		autopilot(gps_transform gps);
		void update(double *recent_pose);
		void set_position(double *msg);
		void takeoff();
		void land();
		bool is_arrived_xy();
		bool is_arrived_z();
		bool get_land_ok();
		int get_mission();
		double* get_target_now();
	private:
		struct pose{ int pose[2];};
		std::vector<pose> waypoints;
		double target_now[3];
		double pose_now[3];
		double way_point[3];
		int mission;
		bool land_ok;
		gps_transform gps;
};
#endif
