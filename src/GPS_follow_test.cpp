#include <ros/ros.h>
#include <iostream>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <Eigen/Geometry>
#include <Eigen/Dense>
#include <cstdio>
#include <unistd.h>
#include <termios.h>
#include <fcntl.h>
#include <math.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <sensor_msgs/NavSatFix.h>
#include <tf/transform_datatypes.h>
#define pi 3.1415926
#define XY_VEL_MAX 0.5
//gain
#define KPx 5.0f//1   3
#define KPy 5.0f//1   3
#define KPz 1.0f//
#define KProll 1.0f//1  2

//earth radius
#define a 6377397.155
#define b 6356078.965

bool init=0;

//
float GPS_init = 0;
double home_longitude,home_latitude,home_height_msl;
double home_ecef_x,home_ecef_y,home_ecef_z;
double x_enu,y_enu,z_enu;
double r11,r12,r13,r21,r22,r23,r31,r32,r33;

using namespace std;
struct vir
{
    float roll;
    float x;
    float y;
    float z;
};

mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg) {
    current_state = *msg;
}
geometry_msgs::PoseStamped offset;
geometry_msgs::PoseStamped host_mocap;

double latitude, longitude, altitude;

//geodetic to ECEF
float N(float phi){  //degree to rad
	float e;
	e = 1 - b*b/(a*a); 
	return a/sqrt(1 - (sin(phi/180*pi)*e)*(sin(phi/180*pi)*e));
}


void set_home_longitude_latitude(double longitude, double latitude, double height_msl)
{
	double sin_lambda = sin((longitude/180)*pi);
	double cos_lambda = cos((longitude/180)*pi);
	double sin_phi = sin((latitude/180)*pi);
	double cos_phi = cos((latitude/180)*pi);

	home_longitude = longitude;
	home_latitude = latitude;
	home_height_msl = height_msl;
	
	r11 = -sin_lambda;
	r12 = cos_lambda;
	r13 = 0;
	r21 = -cos_lambda * sin_phi;
	r22 = -sin_lambda * sin_phi;
	r23 = cos_phi;
	r31 = cos_phi*cos_lambda;
	r32 = cos_phi*sin_lambda;
	r33 = sin_phi;
	
	home_ecef_x = (N(latitude)+height_msl)*cos_phi*cos_lambda;
	home_ecef_y = (N(latitude)+height_msl)*cos_phi*sin_lambda;
	home_ecef_z = ((b*b/(a*a))*N(height_msl)+height_msl)*sin_phi;
}

void gps_pos_cb(const sensor_msgs::NavSatFix::ConstPtr& msg) {

	latitude = msg->latitude;
	longitude = msg->longitude;
	altitude = msg->altitude;
	//gps_received = true;
	
	//set home point
	if(GPS_init < 5){ 
		set_home_longitude_latitude(longitude,latitude,altitude);
		GPS_init ++;
	}
	
    	//ROS_INFO_ONCE("Got global position: [%.2f, %.2f, %.2f]", msg->latitude, msg->longitude, msg->altitude);
    	
	double sin_lambda = sin((longitude/180)*pi);
	double cos_lambda = cos((longitude/180)*pi);
	double sin_phi = sin((latitude/180)*pi);
	double cos_phi = cos((latitude/180)*pi);
	
	//ECEF
    	double X,Y,Z;
    	X = (N(latitude)+altitude)*cos_phi*cos_lambda;
    	Y = (N(latitude)+altitude)*cos_phi*sin_lambda;
    	Z = (b*b/(a*a)*N(latitude)+altitude)*sin_phi;

	double dx = X - home_ecef_x;
	double dy = Y - home_ecef_y;
	double dz = altitude - home_height_msl;
	//ENU
	x_enu = (r11 * dx) + (r12 * dy) + (r13 * dz);
	y_enu = (r21 * dx) + (r22 * dy) + (r23 * dz);
	z_enu = (r31 * dx) + (r32 * dy) + (r33 * dz); ; //barometer of height sensor
	
	cout << "ENU" << endl;
	cout << x_enu << endl;
	cout << y_enu << endl;
	cout << z_enu << endl;
	
	//local 
	
    	
}


void host_pos(const geometry_msgs::PoseStamped::ConstPtr& msg)
{

	host_mocap = *msg;
	if(init == 0){
	offset = host_mocap;
	init = true;
	}

}
float qua2eul(geometry_msgs::PoseStamped& host_mocap)
{
    float pitch,yaw,roll,qx2,qy2,qz2,qw2;
    qx2=(host_mocap.pose.orientation.x)*(host_mocap.pose.orientation.x);
    qy2=(host_mocap.pose.orientation.y)*(host_mocap.pose.orientation.y);
    qz2=(host_mocap.pose.orientation.z)*(host_mocap.pose.orientation.z);
    qw2=(host_mocap.pose.orientation.w)*(host_mocap.pose.orientation.w);
    roll = atan2(2*host_mocap.pose.orientation.z*host_mocap.pose.orientation.w+2*host_mocap.pose.orientation.x*host_mocap.pose.orientation.y , 1 - 2*qy2 - 2*qz2);
    //roll = asin(2*host_mocap.pose.orientation.x*host_mocap.pose.orientation.y + 2*host_mocap.pose.orientation.z*host_mocap.pose.orientation.w);
    //pitch = atan2(2*host_mocap.pose.orientation.x*host_mocap.pose.orientation.w-2*host_mocap.pose.orientation.y*host_mocap.pose.orientation.z , 1 - 2*qx2 - 2*qz2);
	//ROS_INFO("eul: %.3f, %.3f, %.3f", pitch/pi*180, yaw/pi*180, roll/pi*180);
	
    return roll;
}
void follow(vir& vir, geometry_msgs::PoseStamped& host_mocap, geometry_msgs::TwistStamped* vs, float dis_x, float dis_y)
{
	float err_x, err_y, err_z, err_roll;
	float u_x, u_y, u_z, u_roll;
	//float dis_x = 0, dis_y = -0.5;
	float local_x, local_y;

	local_x = cos(vir.roll)*dis_x+sin(vir.roll)*dis_y;
	local_y = -sin(vir.roll)*dis_x+cos(vir.roll)*dis_y;

	err_x = vir.x - host_mocap.pose.position.x - local_x;
	err_y = vir.y - host_mocap.pose.position.y - local_y;
	err_z = vir.z - host_mocap.pose.position.z - 0;
	err_roll = vir.roll - qua2eul(host_mocap);
	
	if(err_roll>pi)
		err_roll = err_roll - 2*pi;
	else if(err_roll<-pi)
		err_roll = err_roll + 2*pi;

	ROS_INFO("err_roll: %.3f",err_roll);

	u_x = KPx*err_x;
	u_y = KPy*err_y;
	u_z = KPz*err_z;
	u_roll = KProll*err_roll;

//	set upper bound	
	u_x = u_x > XY_VEL_MAX ? XY_VEL_MAX : u_x;
	u_y = u_y > XY_VEL_MAX ? XY_VEL_MAX : u_y;
	u_z = u_z > XY_VEL_MAX ? XY_VEL_MAX : u_z;
	u_x = u_x < -XY_VEL_MAX ? -XY_VEL_MAX : u_x;
	u_y = u_y < -XY_VEL_MAX ? -XY_VEL_MAX : u_y;
	u_z = u_z < -XY_VEL_MAX ? -XY_VEL_MAX : u_z;

	vs->twist.linear.x = u_x;
	vs->twist.linear.y = u_y;
	vs->twist.linear.z = u_z;
	vs->twist.angular.z = u_roll;

}
/*
 * Taken from
 * http://stackoverflow.com/questions/421860/capture-characters-from-standard-input-without-waiting-for-enter-to-be-pressed
 *
 * @return the character pressed.
 */
char getch()
{
    int flags = fcntl(0, F_GETFL, 0);
    fcntl(0, F_SETFL, flags | O_NONBLOCK);

    char buf = 0;
    struct termios old = {0};
    if (tcgetattr(0, &old) < 0) {
        perror("tcsetattr()");
    }
    old.c_lflag &= ~ICANON;
    old.c_lflag &= ~ECHO;
    old.c_cc[VMIN] = 1;
    old.c_cc[VTIME] = 0;
    if (tcsetattr(0, TCSANOW, &old) < 0) {
        perror("tcsetattr ICANON");
    }
    if (read(0, &buf, 1) < 0) {
        //perror ("read()");
    }
    old.c_lflag |= ICANON;
    old.c_lflag |= ECHO;
    if (tcsetattr(0, TCSADRAIN, &old) < 0) {
        perror ("tcsetattr ~ICANON");
    }
    return (buf);
}

/*
 * Call main using `rosrun offb offb_main`.
 */
int main(int argc, char **argv)
{
    ros::init(argc, argv, "GPS_follow_test");
    ros::NodeHandle nh;

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
                                ("/mavros/state", 10, state_cb);
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
                                   ("/mavros/setpoint_position/local", 10);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
                                       ("/mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
                                         ("/mavros/set_mode");
    ros::Subscriber host_sub = nh.subscribe<geometry_msgs::PoseStamped>("mavros/local_position/pose", 10, host_pos);
	ros::Subscriber gps_sub = nh.subscribe<sensor_msgs::NavSatFix>("/mavros/global_position/global", 10, gps_pos_cb);	//gps position
    ros::Publisher local_vel_pub = nh.advertise<geometry_msgs::TwistStamped>("/mavros/setpoint_velocity/cmd_vel", 10);
 
    // The setpoint publishing rate MUST be faster than 2Hz.
    ros::Rate rate(100);

    // Wait for FCU connection.
    while (ros::ok() && current_state.connected) {
//	mocap_pos_pub.publish(host_mocap);
        ros::spinOnce();
        rate.sleep();
    }

    
    geometry_msgs::PoseStamped pose;
    geometry_msgs::TwistStamped vs;
	vir vir1;

    vs.twist.linear.x = 0;
    vs.twist.linear.y = 0;
    vs.twist.linear.z = 0;
    vs.twist.angular.x = 0;
    vs.twist.angular.y = 0;
    vs.twist.angular.z = 0;

	vir1.x = 0;
	vir1.y = 0;
	vir1.z = 0.3;
	vir1.roll = 0;

    //send a few setpoints before starting
   for(int i = 100; ros::ok() && i > 0; --i){
        local_vel_pub.publish(vs);
//		mocap_pos_pub.publish(host_mocap);
	vir1.x=offset.pose.position.x;
	vir1.y=offset.pose.position.y;
	vir1.z=offset.pose.position.z + 0.5;	
	ros::spinOnce();
        rate.sleep();
    }

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    ros::Time last_request = ros::Time::now();
	//ros::Time last_request(0);

        if (current_state.mode != "OFFBOARD"){
            if( set_mode_client.call(offb_set_mode) &&
                    offb_set_mode.response.mode_sent) {
                ROS_INFO("Offboard enabled");
            }
            last_request = ros::Time::now();
        } 
	

        if (!current_state.armed){
                if( arming_client.call(arm_cmd) &&
                        arm_cmd.response.success) {
                    ROS_INFO("Vehicle armed");
                }
                last_request = ros::Time::now();
        }
        
    while (ros::ok()) {
//	mocap_pos_pub.publish(host_mocap);

        int c = getch();
	//ROS_INFO("C: %d",c);
        if (c != EOF) {
            switch (c) {
            case 65:    // key up
                vir1.z += 0.05;
                break;
            case 66:    // key down
                vir1.z += -0.05;
                break;
            case 67:    // key CW(->)
                vir1.roll -= 0.03;
                break;
            case 68:    // key CCW(<-)
                vir1.roll += 0.03;
                break;
			case 119:    // key foward
                vir1.x += 0.05;
                break;
            case 120:    // key back
                vir1.x += -0.05;
                break;
            case 97:    // key left
                vir1.y += 0.05;
                break;
            case 100:    // key right
                vir1.y -= 0.05;
                break;
	    	case 115:    // key right
		{
		vir1.x = offset.pose.position.x;
      	vir1.y = offset.pose.position.y;
		vir1.z = offset.pose.position.z+0.2;
		vir1.roll = 0;
                break;
		}
		case 108:    // close arming
			{
			offb_set_mode.request.custom_mode = "MANUAL";
			set_mode_client.call(offb_set_mode);
			arm_cmd.request.value = false;
            break;
			}
            case 63:
                return 0;
                break;
            }
        }
		if(vir1.roll>pi)
		vir1.roll = vir1.roll - 2*pi;
		else if(vir1.roll<-pi)
		vir1.roll = vir1.roll + 2*pi;

        ROS_INFO("setpoint: %.2f, %.2f, %.2f, %.2f", vir1.x, vir1.y, vir1.z, vir1.roll/pi*180);
		follow(vir1,host_mocap,&vs,0,0);
//        mocap_pos_pub.publish(host_mocap);
        local_vel_pub.publish(vs);

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}

