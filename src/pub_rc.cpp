#include <ros/ros.h>
#include <mavros_msgs/ActuatorControl.h>
#include <mavros_msgs/OverrideRCIn.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/RCOut.h>
#include <sensor_msgs/BatteryState.h>

mavros_msgs::State current_state;
mavros_msgs::RCOut rc_out;
sensor_msgs::BatteryState battery;
void state_cb(const mavros_msgs::State::ConstPtr& msg){
  current_state = *msg;
}
void rc_cb(const mavros_msgs::RCOut::ConstPtr& msg){
  rc_out = *msg;
}
void battery_cb(const sensor_msgs::BatteryState::ConstPtr &msg){
  battery = *msg;
}
int main(int argc, char **argv)
{
  ros::init(argc, argv, "control_rc");
  ros::NodeHandle nh;

  ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("/mavros/state", 2, state_cb);
  ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
                                       ("/mavros/set_mode");
  ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
                                     ("/mavros/cmd/arming");

  ros::Subscriber rc_sub = nh.subscribe<mavros_msgs::RCOut>("/mavros/rc/out", 2, rc_cb);
  ros::Subscriber battery_sub = nh.subscribe<sensor_msgs::BatteryState>("/mavros/battery", 2, battery_cb);

  ros::Rate loop_rate(50);

  double sum;
  int count = 1;
  double average;

  double pwm1, pwm2, pwm3, pwm4;
  double F1, F2, F3, F4;
  int  battery_flag1 =0 ,battery_flag2 = 0,battery_flag3 = 0;
  
  while(ros::ok()){
/*
    double rc_value_0, rc_value_1, rc_value_2, rc_value_3;

    if(rc_out.channels.size()!=0 && rc_out.channels[0] != 0){
      rc_value_0 =rc_out.channels[0];
      sum = sum + rc_value_0;

      average = sum / count;
      count += 1;
    }
    if(rc_out.channels.size()!=0 && rc_out.channels[1] != 0){
      rc_value_1 =rc_out.channels[1];
    }
    if(rc_out.channels.size()!=0 && rc_out.channels[2] != 0){
      rc_value_2 =rc_out.channels[2];
    }
    if(rc_out.channels.size()!=0 && rc_out.channels[3] != 0){
      rc_value_3 =rc_out.channels[3];
    }

    ROS_INFO("channel 0 = %f", rc_value_0);
    ROS_INFO("channel 1 = %f", rc_value_1);
    ROS_INFO("channel 2 = %f", rc_value_2);
    ROS_INFO("channel 3 = %f", rc_value_3);
    ROS_INFO("average = %f", average);
*/
if(rc_out.channels.size()!=0 && rc_out.channels[0] != 0){

    pwm1 = rc_out.channels[0];
    pwm2 = rc_out.channels[1];
    pwm3 = rc_out.channels[2];
    pwm4 = rc_out.channels[3];

}

if(battery.voltage !=0 && (battery.voltage < 11)){

battery_flag1 = 1;
battery_flag2 = 0;
battery_flag3 = 0;
 }

if(battery.voltage !=0 && (battery.voltage < 11.3)&& (battery.voltage > 11)){

battery_flag1 = 0;
battery_flag2 = 1;
battery_flag3 = 0;
    }

if(battery.voltage !=0 &&(battery.voltage > 11.3) ){
battery_flag1 = 0;
battery_flag2 = 0;
battery_flag3 = 1;
}

if(battery_flag1 == 1){

F1 = (0.0003 *(pwm1*pwm1) +0.1845 *pwm1 -609.4464)*9.8/1000; 
F2 = (0.0004 *(pwm2*pwm2) -0.3171 *pwm2 -212.6339)*9.8/1000; 
F3 = (0.0003 *(pwm3*pwm3) +0.0350 *pwm3 -483.5982)*9.8/1000; 
F4 = (0.0004 *(pwm4*pwm4) -0.1531 *pwm4 -338.2946)*9.8/1000;
}

if(battery_flag2 == 1){

F1 = (0.0004 *(pwm1*pwm1) -0.2214 *pwm1 -302.8036)*9.8/1000; 
F2 = (0.0003 *(pwm2*pwm2) +0.0133 *pwm2 -491.8750)*9.8/1000; 
F3 = (0.0002 *(pwm3*pwm3) +0.4545 *pwm3 -828.6161)*9.8/1000; 
F4 = (0.0004 *(pwm4*pwm4) -0.2405 *pwm4 -303.1964)*9.8/1000;
}

if(battery_flag3 == 1){

F1 = (0.0004 *(pwm1*pwm1) -0.1933 *pwm1 -336.5446)*9.8/1000; 
F2 = (0.0005 *(pwm2*pwm2) -0.3390 *pwm2 -231.0982)*9.8/1000; 
F3 = (0.0003 *(pwm3*pwm3) +0.1555 *pwm3 -597.5536)*9.8/1000; 
F4 = (0.0003 *(pwm4*pwm4) +0.0367 *pwm4 -527.0089)*9.8/1000;
}
ROS_INFO("weight = %f", F1+F2+F3+F4);
    ros::spinOnce();
    loop_rate.sleep();


  }

  return 0;
}
