#ifndef _CONTROL_CHASSIS_H_
#define _CONTROL_CHASSIS_H_

#include "qbytearray.h"
#include "VCOMCOMM.h"
#include "memory"
#include <memory>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/TwistStamped.h>


typedef struct{
    double Expect_Speed_X;
    double Expect_Speed_Y;
    double Expect_Speed_Yaw;

    // double Current_Speed_X;
    // double Current_Speed_Y;
    // double Current_Speed_Yaw;
} Robot_Chassis_Exp_;

// namespace control_chassis_ns {

// class control_chassis
// {
// private:
//     QByteArray serial_data;
//     Robot_Chassis_Exp_ *Robot_Chassis_Exp_Ptr = NULL;
//     VCOMCOMM *vcom = NULL;
// public:
//     control_chassis(Robot_Chassis_Exp_ *Expect, VCOMCOMM *vcomcomm);
//     ~control_chassis();
//     void set_Need_Speed(const geometry_msgs::TwistStamped::ConstPtr& cmd);
//     void Trinsmit_Speed();
// };

// }

#endif