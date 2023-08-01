#include "ros/init.h"
#include "ros/publisher.h"
#include "ros/rate.h"
#include "ros/ros.h"
#include "ros/subscriber.h"
#include "user_package/control_chassis.h"
#include <VCOMCOMM.h>
#include <cstdint>
#include "std_msgs/String.h"

#include <memory>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/TwistStamped.h>

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>

#include <boost/smart_ptr.hpp>
#include <boost/make_shared.hpp>

bool Navigation_Mode_now = true;

// std::shared_ptr<control_chassis_ns::control_chassis> vcom_trinsmit_speed(new control_chassis_ns::control_chassis());
void OdometryAndCmd_VelHandler(const geometry_msgs::TwistStamped::ConstPtr& cmd_vel)
{
    Robot_Chassis_Exp_ Robot_Chassis_Exp;
    VCOMCOMM vcom_user;
    QByteArray serial_data_user;
    
    Robot_Chassis_Exp.Expect_Speed_X = cmd_vel->twist.linear.x;
    Robot_Chassis_Exp.Expect_Speed_Y = cmd_vel->twist.linear.y;
    Robot_Chassis_Exp.Expect_Speed_Yaw = cmd_vel->twist.angular.z;
    // robot_expect = (Robot_Chassis_Exp_ *)serial_data_user.data();
    serial_data_user.append((char *)&Robot_Chassis_Exp, sizeof(Robot_Chassis_Exp_));
    ROS_INFO("X = %f,Y = %f,YAW = %f\n", Robot_Chassis_Exp.Expect_Speed_X, Robot_Chassis_Exp.Expect_Speed_Y, Robot_Chassis_Exp.Expect_Speed_Yaw);

    ROS_INFO("Robot is in navigation mode");

    vcom_user.Transmit(1, 1, serial_data_user);
}

void cmd_vel_user_callback(const geometry_msgs::TwistStamped::ConstPtr& cmd_vel)
{
    Robot_Chassis_Exp_ Robot_Chassis_Exp;
    VCOMCOMM vcom_user;
    QByteArray serial_data_user;
    
    Robot_Chassis_Exp.Expect_Speed_X = cmd_vel->twist.linear.x;
    Robot_Chassis_Exp.Expect_Speed_Y = cmd_vel->twist.linear.y;
    Robot_Chassis_Exp.Expect_Speed_Yaw = cmd_vel->twist.angular.z;

    serial_data_user.append((char *)&Robot_Chassis_Exp, sizeof(Robot_Chassis_Exp_));

    ROS_INFO("X = %f,Y = %f,YAW = %f\n", Robot_Chassis_Exp.Expect_Speed_X, Robot_Chassis_Exp.Expect_Speed_Y, Robot_Chassis_Exp.Expect_Speed_Yaw);

    ROS_INFO("Robot is not in navigation mode");

    vcom_user.Transmit(1, 1, serial_data_user);
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "control_chassis");
    ros::NodeHandle nh;

    nh.getParam("Navigation_Mode_now", Navigation_Mode_now);

    ros::Subscriber subCmd_Vel = Navigation_Mode_now ? \
     nh.subscribe<geometry_msgs::TwistStamped>("/cmd_vel", 10, OdometryAndCmd_VelHandler) : \
     nh.subscribe<geometry_msgs::TwistStamped>("/cmd_vel_user", 10, cmd_vel_user_callback);

    ROS_INFO("%d", Navigation_Mode_now);

    ros::spin();

    return 0;
}