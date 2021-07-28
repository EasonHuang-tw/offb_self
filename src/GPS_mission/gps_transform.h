#ifndef GPS_TRANSFORM
#define GPS_TRANSFORM

#include <iostream>
#include <math.h>
#define pi M_PI
#define a 6377397.155 
#define b 6356078.965
class gps_transform{
	public:
		void set_home_longitude_latitude(double lon, double lat, double hei);
		void ECEF_calculate(double lon, double lat, double hei);
		void ECEF_2_ENU();
		void get_ENU(double*);
		void get_ECEF(double*);	
		void get_home_ECEF(double*);	
	private:
		double sin_lambda;
		double cos_lambda;
		double sin_phi;
		double cos_phi;
		
		double latitude, longitude, altitude;
    		double X,Y,Z;
		double x_enu,y_enu,z_enu;
		double home_longitude,home_latitude,home_height_msl; 
		double home_ecef_x,home_ecef_y,home_ecef_z;
		double r11,r12,r13,r21,r22,r23,r31,r32,r33;
		float N(float phi);


};



#endif
