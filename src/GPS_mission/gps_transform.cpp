#include <iostream>
#include <gps_transform.h>

using namespace std;
float gps_transform::N(float phi){  //degree to rad
	float e;
	e = 1 - b*b/(a*a); 
	return a/sqrt(1 - (sin(phi/180*pi)*e)*(sin(phi/180*pi)*e));
}

void gps_transform::update(double latitude, double longitude, double altitude){
	this->latitude = latitude;	
	this->longitude = longitude;	
	this->altitude = altitude;
	double ECEF_pose[3];	
	this->ECEF_update(latitude ,longitude,altitude, ECEF_pose);
	double ENU_pose[3];	
	this->ECEF_2_ENU_update(ECEF_pose[0],ECEF_pose[1],ECEF_pose[2],ENU_pose);
}

bool gps_transform::is_init(){
	return init_flag;
}

void gps_transform::set_home_longitude_latitude(double latitude ,double longitude,double altitude)
{

	this->home_longitude = longitude;
	this->home_latitude = latitude;
	this->home_height_msl = altitude;
	
	double ECEF_pose[3];	
	ECEF_update(latitude , longitude, altitude,ECEF_pose);
	
	this->home_ecef_x = this->X;
	this->home_ecef_y = this->Y;
	this->home_ecef_z = this->Z;
	
	this->r11 = -this->sin_lambda;
	this->r12 = this->cos_lambda;
	this->r13 = 0;
	this->r21 = -this->cos_lambda * this->sin_phi;
	this->r22 = -this->sin_lambda * this->sin_phi;
	this->r23 = this->cos_phi;
	this->r31 = this->cos_phi*this->cos_lambda;
	this->r32 = this->cos_phi*this->sin_lambda;
	this->r33 = this->sin_phi;
	this->init_flag = true;
	
}
void gps_transform::ECEF_update(double latitude ,double longitude,double altitude,double* ECEF_pose){
	this->sin_lambda = sin((longitude/180)*pi);
	this->cos_lambda = cos((longitude/180)*pi);
	this->sin_phi = sin((latitude/180)*pi);
	this->cos_phi = cos((latitude/180)*pi);
    	this->X = (this->N(latitude)+altitude)*this->cos_phi*this->cos_lambda;
    	this->Y = (this->N(latitude)+altitude)*this->cos_phi*this->sin_lambda;
    	this->Z = (b*b/(a*a)*this->N(latitude)+altitude)*this->sin_phi;
	ECEF_pose[0] = this->X;
	ECEF_pose[1] = this->Y;
	ECEF_pose[2] = this->Z;

}

void gps_transform::ECEF_2_ENU_update(double X,double Y,double Z,double *ENU_pose){
	double dx =this-> X - home_ecef_x;
	double dy =this-> Y - home_ecef_y;
	double dz =this-> Z - home_ecef_z;
	cout << "delta(new) \tx:"<<dx<<"\ty:"<<dy<<"\tz:"<<dz<<endl;
	//ENU
	this->x_enu = (this->r11 * dx) + (this->r12 * dy) + (this->r13 * dz);
	this->y_enu = (this->r21 * dx) + (this->r22 * dy) + (this->r23 * dz);
	//this->z_enu = (this->r31 * dx) + (this->r32 * dy) + (this->r33 * dz) ; //barometer of height sensor
	this->z_enu = this->altitude - this->home_height_msl; 
	ENU_pose[0]=x_enu;
	ENU_pose[1]=y_enu;
	ENU_pose[2]=z_enu;
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
