#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <sensor_msgs/NavSatFix.h>

#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/GlobalPositionTarget.h>

#include <cstdio>
#include <unistd.h>
#include <termios.h>
#include <fcntl.h>

#define pi 3.1415926

using namespace std;
typedef struct vir
{
    float roll;
    float x;
    float y;
    float z;
};

/*	global variable 	*/

int flag=0;
float KPx = 5;		//1   3
float KPy = 5;		//1   3
float KPz = 1;
float KProll = 0;	//1  2
bool local_offset_init_flag =0;		//flag of local position offset init
bool gps_offset_init_flag =0;		//flag of local position offset init

mavros_msgs::State current_state;	//state

geometry_msgs::PoseStamped offset;
geometry_msgs::PoseStamped host_mocap;

sensor_msgs::NavSatFix gps_position;
sensor_msgs::NavSatFix gps_position_offset;

/*	 call back function	*/

void state_cb(const mavros_msgs::State::ConstPtr& msg) {	//get state
    current_state = *msg;
}

void host_pos_cb(const geometry_msgs::PoseStamped::ConstPtr& msg)	// mavros/local_position/pose
{

	host_mocap = *msg;
	if( local_offset_init_flag  == 0){
	offset = host_mocap;
	local_offset_init_flag = true;
	}

}

void gps_pos_cb(const sensor_msgs::NavSatFix::ConstPtr& msg) {
	gps_position = *msg;
	if( gps_offset_init_flag  == 0){
		gps_position_offset = gps_position;
		gps_offset_init_flag = true;
	}
    	//ROS_INFO_ONCE("Got global position: [%.2f, %.2f, %.2f]", msg->latitude, msg->longitude, msg->altitude);
}

/*	functions	*/

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
	float errx, erry, errz, err_roll;
	float ux, uy, uz, uroll;
	//float dis_x = 0, dis_y = -0.5;
	float local_x, local_y;

	local_x = cos(vir.roll)*dis_x+sin(vir.roll)*dis_y;
	local_y = -sin(vir.roll)*dis_x+cos(vir.roll)*dis_y;

	errx = vir.x - host_mocap.pose.position.x - local_x;
	erry = vir.y - host_mocap.pose.position.y - local_y;
	errz = vir.z - host_mocap.pose.position.z - 0;
	err_roll = vir.roll - qua2eul(host_mocap);
	if(err_roll>pi)
	err_roll = err_roll - 2*pi;
	else if(err_roll<-pi)
	err_roll = err_roll + 2*pi;

	ROS_INFO("err_roll: %.3f",err_roll);

	ux = KPx*errx;
	uy = KPy*erry;
	uz = KPz*errz;
	uroll = KProll*err_roll;

	vs->twist.linear.x = ux;
	vs->twist.linear.y = uy;
	vs->twist.linear.z = uz;
	vs->twist.angular.z = uroll;

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

	/*	publisth	*/
	ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
                                   ("/mavros/setpoint_position/local", 10);
	ros::Publisher goal_pos_pub = nh.advertise < mavros_msgs::GlobalPositionTarget > ("mavros/setpoint_position/global", 10);

	/*	subscriber	*/
	ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("/mavros/state", 10, state_cb);				//state
	ros::Subscriber host_sub = nh.subscribe<geometry_msgs::PoseStamped>("mavros/local_position/pose", 10, host_pos_cb);		//local position
	ros::Subscriber gps_sub = nh.subscribe<sensor_msgs::NavSatFix>("/mavros/global_position/global", 10, gps_pos_cb);	//gps position
	ros::Publisher local_vel_pub = nh.advertise<geometry_msgs::TwistStamped>("/mavros/setpoint_velocity/cmd_vel", 10);

	 /*	service		*/
	ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
				       ("/mavros/cmd/arming");
	ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
                                         ("/mavros/set_mode");

    	// The setpoint publishing rate MUST be faster than 2Hz.
	ros::Rate rate(10);

	// Wait for FCU connection.
	while (ros::ok() && current_state.connected) {
		ros::spinOnce();
		rate.sleep();
	}
	ROS_INFO("FCU connected");

    	while (ros::ok() && !gps_offset_init_flag) {
        	ROS_INFO_ONCE("Waiting for GPS signal...");
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
	/*
	for(int i = 100; ros::ok() && i > 0; --i){
		Vlocal_vel_pub.publish(vs);
		//		mocap_pos_pub.publish(host_mocap);
		vir1.x=offset.pose.position.x;
		vir1.y=offset.pose.position.y;
		vir1.z=offset.pose.position.z + 0.5;	
		ros::spinOnce();
		rate.sleep();
	}
	*/
	
	// set target position
	mavros_msgs::GlobalPositionTarget goal_position;
	goal_position.latitude = gps_position.latitude;
	goal_position.longitude = gps_position.longitude;
	goal_position.altitude = gps_position.altitude;

	for (int i=0; i<20; ++i) {
		goal_position.header.stamp = ros::Time::now();
		goal_pos_pub.publish(goal_position);
		ros::spinOnce();
		rate.sleep();
	}


	ros::Time last_request = ros::Time::now();
	
	//set mode
	mavros_msgs::SetMode offb_set_mode;
	offb_set_mode.request.custom_mode = "OFFBOARD";
        
	if (current_state.mode != "OFFBOARD"){
            if( set_mode_client.call(offb_set_mode) &&
                    offb_set_mode.response.mode_sent) {
                ROS_INFO("Offboard enabled");
            }
            last_request = ros::Time::now();
        }
       //arm	
	mavros_msgs::CommandBool arm_cmd;
	arm_cmd.request.value = true;
        if (!current_state.armed){
                if( arming_client.call(arm_cmd) &&
                        arm_cmd.response.success) {
                    ROS_INFO("Vehicle armed");
                }
                last_request = ros::Time::now();
        }
	goal_position.altitude = goal_position.altitude + 5.0;
		
	while (ros::ok()) {
//	mocap_pos_pub.publish(host_mocap);
/*
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
			arming_client.call(arm_cmd);
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
*/
	
	goal_position.header.stamp = ros::Time::now();
        goal_pos_pub.publish(goal_position);
	ROS_INFO_THROTTLE(1, "At altitude %.2f", gps_position.altitude);

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}

