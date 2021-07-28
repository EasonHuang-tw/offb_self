#include <iostream>
#include <gps_transform.h>

using namespace std;
float gps_transform::N(float phi){  //degree to rad
	float e;
	e = 1 - b*b/(a*a); 
	return a/sqrt(1 - (sin(phi/180*pi)*e)*(sin(phi/180*pi)*e));
}

void gps_transform::set_home_longitude_latitude(double lon, double lat, double hei)
{

	latitude = lat;
	longitude = lon;
	altitude = hei;
	
	sin_lambda = sin((longitude/180)*pi);
	cos_lambda = cos((longitude/180)*pi);
	sin_phi = sin((latitude/180)*pi);
	cos_phi = cos((latitude/180)*pi);

	home_longitude = longitude;
	home_latitude = latitude;
	home_height_msl = altitude;
	
	r11 = -sin_lambda;
	r12 = cos_lambda;
	r13 = 0;
	r21 = -cos_lambda * sin_phi;
	r22 = -sin_lambda * sin_phi;
	r23 = cos_phi;
	r31 = cos_phi*cos_lambda;
	r32 = cos_phi*sin_lambda;
	r33 = sin_phi;
	
	home_ecef_x = (N(latitude)+altitude)*cos_phi*cos_lambda;
	home_ecef_y = (N(latitude)+altitude)*cos_phi*sin_lambda;
	home_ecef_z = ((b*b/(a*a))*N(latitude)+altitude)*sin_phi;
}
void gps_transform::ECEF_calculate(double lat ,double lon,double alt){	
	latitude = lat;
	longitude = lon;
	altitude = alt;
	sin_lambda = sin((longitude/180)*pi);
	cos_lambda = cos((longitude/180)*pi);
	sin_phi = sin((latitude/180)*pi);
	cos_phi = cos((latitude/180)*pi);
    	X = (N(latitude)+altitude)*cos_phi*cos_lambda;
    	Y = (N(latitude)+altitude)*cos_phi*sin_lambda;
    	Z = (b*b/(a*a)*N(latitude)+altitude)*sin_phi;
}

void gps_transform::ECEF_2_ENU(){
	double dx = X - home_ecef_x;
	double dy = Y - home_ecef_y;
	double dz = altitude - home_height_msl;
//	cout << "delta(new) \tx:"<<dx<<"\ty:"<<dy<<"\tz:"<<dz<<endl;
	//ENU
	x_enu = (r11 * dx) + (r12 * dy) + (r13 * dz);
	y_enu = (r21 * dx) + (r22 * dy) + (r23 * dz);
	z_enu = (r31 * dx) + (r32 * dy) + (r33 * dz); ; //barometer of height sensor
}

void gps_transform::get_ENU(double *msg){
	msg[0] = x_enu;
	msg[1] = y_enu;
	msg[2] = z_enu;
}

void gps_transform::get_home_ECEF(double *msg){
	msg[0] = home_ecef_x;
	msg[1] = home_ecef_y;
	msg[2] = home_ecef_z;
//	cout << "ECEF home(new) \tx:" << home_ecef_x <<",\ty: " << home_ecef_y <<",\tz: " << home_ecef_z << endl;

}
void gps_transform::get_ECEF(double *msg){
	msg[0] = X;
	msg[1] = Y;
	msg[2] = Z;
//	cout << "ECEF(new) \tx:" << X <<",\ty: " << Y <<",\tz: " << Z << endl;

}
