/**
 * @file offb_node.cpp
 * @brief Offboard control example node, written with MAVROS version 0.19.x, PX4 Pro Flight
 * Stack and tested in Gazebo SITL
 */

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>

#include <mavros_msgs/ManualControl.h>
#include <mavros_msgs/AttitudeTarget.h>
#include <geometry_msgs/Vector3Stamped.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <cstdio>
#include <unistd.h>
#include <termios.h>
#include <fcntl.h>

#include "tf/transform_datatypes.h"
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
mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "offb_node");
    ros::NodeHandle nh;

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("/mavros/state", 10, state_cb);
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("/mavros/setpoint_position/local", 10);

    ros::Publisher local_attitude_pub = nh.advertise<mavros_msgs::AttitudeTarget>
            ("/mavros/setpoint_raw/attitude", 10);
    ros::Publisher local_acc_pub = nh.advertise<geometry_msgs::Vector3Stamped>
            ("/mavros/setpoint_accel/accel", 10);
    ros::Publisher manual_pub = nh.advertise<mavros_msgs::ManualControl>
            ("/mavros/mavros_msgs/ManualControl/send", 10);

    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("/mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("/mavros/set_mode");

    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);

    // wait for FCU connection
    while(ros::ok() && !current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }
    /*
    mavros_msgs::AttitudeTarget attitude;
    tf::Quaternion quat;
    double r, p, y;
    r = 0;
    p = 0;
    y = 0;

    //tf::createQuaternionMsgFromRollPitchYaw(double r, double p, double y);
    */
    /*
    geometry_msgs::Vector3Stamped acc;
    acc.vector.x = 0;
    acc.vector.y = 0.2;
    acc.vector.z = 0.6;


    mavros_msgs::ManualControl manual;
    manual.x = 0.0;
    manual.y = 0.0;
    manual.z = 0.0;
    manual.r = 0.0;

    */
    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = 0.75;
    pose.pose.position.y = 0;
    pose.pose.position.z = 0.5;
    /**/
    int flag = 0;
    float r = float(pose.pose.position.x);
    float dt = 0;

    //send a few setpoints before starting
    for(int i = 100; ros::ok() && i > 0; --i){
        local_pos_pub.publish(pose);
        //local_attitude_pub.publish(attitude);
        //local_acc_pub.publish(acc);
        //manual_pub.publish(manual);
        ros::spinOnce();
        rate.sleep();
    }

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";
    //offb_set_mode.request.custom_mode = "MANUAL";
    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    ros::Time last_request = ros::Time::now();

    while(ros::ok()){
        if( current_state.mode != "OFFBOARD" &&
            (ros::Time::now() - last_request > ros::Duration(5.0))){
            if( set_mode_client.call(offb_set_mode) &&
                offb_set_mode.response.mode_sent){
                ROS_INFO("Offboard enabled");
            }
            last_request = ros::Time::now();
        } else {
            if( !current_state.armed &&
                (ros::Time::now() - last_request > ros::Duration(5.0))){
                if( arming_client.call(arm_cmd) &&
                    arm_cmd.response.success){
                    ROS_INFO("Vehicle armed");
                }
                last_request = ros::Time::now();
            }
        }
        int c = getch();
        //ROS_INFO("C: %d",c);

        /*
        quat.setRPY(r,p,y);
        pose.pose.orientation.x = quat[0];
        pose.pose.orientation.y = quat[1];
        pose.pose.orientation.z = quat[2];
        pose.pose.orientation.w = quat[3];
*/
        if(flag == 1){
            dt += 0.1;
            pose.pose.position.x = r * cos(dt);
            pose.pose.position.y = r * sin(dt);
        }

        if (c != EOF) {
            switch (c) {
            case 65:    // key up
                pose.pose.position.z += 0.05;
                //r += 5;
                //acc.vector.z += 0.05;

                break;
            case 66:    // key down
                pose.pose.position.z += -0.05;
                //r -= 5;
                //acc.vector.z -= 0.05;

                break;
            case 119:    // key foward
                pose.pose.position.x += 0.05;
                //p += 5;
                //acc.vector.x += 0.05;
                //manual.x -= 0.05;
                break;
            case 120:    // key back
                pose.pose.position.x += -0.05;
                //p -= 5;
                //acc.vector.x -= 0.05;
                //manual.x += 0.05;
                break;
            case 97:    // key left
                pose.pose.position.y += 0.05;
                //y += 5;
                //acc.vector.y += 0.05;
                //manual.z += 0.05;
                break;
            case 100:    // key right
                pose.pose.position.y -= 0.05;
                //y -= 5;
                //acc.vector.y -= 0.05;
                //manual.z -= 0.05;
                flag = 1;
                break;
            case 115:    // key right
		{
		pose.pose.position.x = 0;
      		pose.pose.position.y = 0;
		pose.pose.position.z = 0.2;
		flag = 0;
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
        local_pos_pub.publish(pose);
        //local_attitude_pub.publish(attitude);
        //local_acc_pub.publish(acc);
        //manual_pub.publish(manual);
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
