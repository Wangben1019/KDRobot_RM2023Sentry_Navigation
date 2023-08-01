#include "ros/duration.h"
#include "ros/init.h"
#include "ros/node_handle.h"
#include "ros/publisher.h"
#include "ros/ros.h"
#include "ros/subscriber.h"
#include <boost/bind/bind.hpp>
#include <boost/bind/placeholders.hpp>
#include <cstdint>
#include <geometry_msgs/TwistStamped.h>
#include "demo_reconfigure_user/demo_configConfig.h"
#include "dynamic_reconfigure/server.h"
#include "ros/timer.h"

bool Navigation_Mode;
bool Patrol_Mode;
bool Rotate_Mode;
bool Counterattack_Mode;

geometry_msgs::TwistStamped cmd_vel_user;

ros::Publisher pubCmd_Vel_User;

namespace demo_reconfigure_user{
    void callback(demo_configConfig &config, uint32_t level)
    {
        if (!config.Navigation_Mode) 
        {
            if (config.Rotate_Mode && !config.Patrol_Mode && !config.Counterattack_Mode) 
            {
                cmd_vel_user.twist.linear.x = 0;
                cmd_vel_user.twist.linear.y = 0;
                cmd_vel_user.twist.angular.z = 0.75;    // 0.75

                Navigation_Mode = config.Navigation_Mode;
                Patrol_Mode = config.Patrol_Mode;
                Rotate_Mode = config.Rotate_Mode;
                Counterattack_Mode = config.Counterattack_Mode;

                ROS_INFO("Robot is in patrol mode\n");
            }
            else if (!config.Rotate_Mode && config.Patrol_Mode && !config.Counterattack_Mode) 
            {
                cmd_vel_user.twist.linear.x = 0;
                cmd_vel_user.twist.linear.y = 1;  // 1
                cmd_vel_user.twist.angular.z = 0;

                Navigation_Mode = config.Navigation_Mode;
                Patrol_Mode = config.Patrol_Mode;
                Rotate_Mode = config.Rotate_Mode;
                Counterattack_Mode = config.Counterattack_Mode;

                ROS_INFO("Robot is in navigation mode\n");
            }
            else if (!config.Rotate_Mode && !config.Patrol_Mode && config.Counterattack_Mode) {
                cmd_vel_user.twist.linear.x = 0;
                cmd_vel_user.twist.linear.y = 0;
                cmd_vel_user.twist.angular.z = 1.25;  // 1.25

                Navigation_Mode = config.Navigation_Mode;
                Patrol_Mode = config.Patrol_Mode;
                Rotate_Mode = config.Rotate_Mode;
                Counterattack_Mode = config.Counterattack_Mode;

                ROS_INFO("Robot is in navigation mode\n");
            }
            else 
            {
                cmd_vel_user.twist.linear.x = 0;
                cmd_vel_user.twist.linear.y = 0;
                cmd_vel_user.twist.angular.z = 0;

                Navigation_Mode = config.Navigation_Mode;
                Patrol_Mode = config.Patrol_Mode;
                Rotate_Mode = config.Rotate_Mode;
                Counterattack_Mode = config.Counterattack_Mode;
                
                ROS_INFO("If robot is not in navigation mode. Please ensure that the robot has only one valid state\n");
            }
        }
        else 
        {
            ROS_INFO("Robot is in navigation mode\n");
            return;
        }

    }
};

void timer_callback_pub(const ros::TimerEvent& event)
{
    if (!Navigation_Mode) 
    {
        pubCmd_Vel_User.publish(cmd_vel_user);
    }
}

void timer_callback_back(const ros::TimerEvent& event)
{
    if (!Navigation_Mode && !Rotate_Mode && Patrol_Mode && !Counterattack_Mode) 
    {
        cmd_vel_user.twist.linear.x = cmd_vel_user.twist.linear.x;
        cmd_vel_user.twist.linear.y = -cmd_vel_user.twist.linear.y;
        cmd_vel_user.twist.angular.z = cmd_vel_user.twist.angular.z;
    }
}


int main(int argc, char *argv[])
{
    ros::init(argc, argv, "temp_solve_user");
    ros::NodeHandle nh;

    nh.param<bool>("Navigation_Mode", Navigation_Mode, true);
    nh.param<bool>("Patrol_Mode", Patrol_Mode, false);
    nh.param<bool>("Rotate_Mode", Rotate_Mode, false);
    nh.param<bool>("Counterattack_Mode", Counterattack_Mode, false);

    dynamic_reconfigure::Server<demo_reconfigure_user::demo_configConfig> srv;
    dynamic_reconfigure::Server<demo_reconfigure_user::demo_configConfig>::CallbackType ca = boost::bind(&demo_reconfigure_user::callback, _1, _2);

    srv.setCallback(ca);

    pubCmd_Vel_User = nh.advertise<geometry_msgs::TwistStamped>("/cmd_vel_user", 5);

    ros::Timer timer_pub = nh.createTimer(ros::Duration(0.01), timer_callback_pub);

    ros::Timer timer_back = nh.createTimer(ros::Duration(2), timer_callback_back);

    // ros::Rate rate(100);

    // while (ros::ok()) {
    //     ros::spinOnce();

    //     nh.setParam("Navigation_Mode_now", Navigation_Mode);

    //     rate.sleep();
    // }

    ros::spin();

    return 0;
}
