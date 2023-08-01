#include "user_package/sentry_master.h"
#include "nav_msgs/Odometry.h"
#include "ros/duration.h"
#include "ros/init.h"
#include "VCOMCOMM.h"
#include "qbytearray.h"
#include "std_msgs/Int8.h"
#include <cmath>
Map_Point Control_Area(6.03125, -1, 0);
Map_Point Patrol_Area(0, 0, 0);
Map_Point Outpose_Out_Area(6.03125, -4, 0);
Map_Point Outpose_In_Area(3.1485, -4, 0);

struct CmpByValue {
    bool operator()(const std::pair<std::string, float>& lhs, const std::pair<std::string, float>& rhs) {
        return lhs.second > rhs.second;
    }
};

void Sentry_Master::odomHandler(const nav_msgs::Odometry::ConstPtr& odom)
{
    Sentry_Odometry.pose.pose.position.x = odom->pose.pose.position.x;
    Sentry_Odometry.pose.pose.position.y = odom->pose.pose.position.y;
    Sentry_Odometry.pose.pose.position.z = odom->pose.pose.position.z;
}

void Sentry_Master::Timer_Event_cb(const ros::TimerEvent& event)
{
    Can_Make_Decisions = true;
}

/* 从头到尾裁判系统断连 */
void Sentry_Master::Timer_Start_Referee_Error_Event_cb(const ros::TimerEvent& event)
{
    Sentry_Robot_Status.Referee_Status = Referee_Error; 
}

/* 比赛途中裁判系统断连 */
void Sentry_Master::Timer_Competition_Referee_Error_Event_cb(const ros::TimerEvent& event)
{
    if (Sentry_Robot_Status.Referee_Status != Unconnected_Competition) {
        Sentry_Robot_Status.Referee_Status = Referee_Error;
    }
}

void Sentry_Master::Master_Init()
{
    sentry_goal_pub = nh.advertise<geometry_msgs::PointStamped>("/goal_point", 5);
    Lock_Stop_pub = nh.advertise<std_msgs::Int8>("/stop", 5);
    Odom_msg_Sub = nh.subscribe<nav_msgs::Odometry>("/state_estimation", 5, &Sentry_Master::odomHandler, this);
    Pub_Goal_Interval_Timer = nh.createTimer(ros::Duration(20), &Sentry_Master::Timer_Event_cb, this);
    Start_Referee_Error_Event_Timer = nh.createTimer(ros::Duration(240), &Sentry_Master::Timer_Start_Referee_Error_Event_cb, this);
    Competition_Referee_Error_Event_Timer = nh.createTimer(ros::Duration(15), &Sentry_Master::Timer_Competition_Referee_Error_Event_cb, this);
}

void Sentry_Master::receive_data()
{
    VCOMCOMM vcom_receive;
    QByteArray QReceive_data1;
    QByteArray QReceive_data2;
    Transmit_Data_1_t *Transmit_Data_1;
    Transmit_Data_2_t *Transmit_Data_2;

    Robot_Last_Data = this->Referee_system_Data;

    vcom_receive.receiveData(0x01, 0x01, QReceive_data1);
    vcom_receive.receiveData(0x02, 0x02, QReceive_data2);

    // Referee_system_Data_t *data_ptr = (Referee_system_Data_t *)QReceive_data.data();
    Transmit_Data_1 = (Transmit_Data_1_t *)QReceive_data1.data();
    Transmit_Data_2 = (Transmit_Data_2_t *)QReceive_data2.data();

    Referee_system_Data.Transmit_Data_1 = *Transmit_Data_1;
    Referee_system_Data.Transmit_Data_2 = *Transmit_Data_2;

    if (Referee_system_Data.Transmit_Data_1.Aim_Status == 0) {
        Sentry_Robot_Status.Aim_Status = UnLock_Robot;
    }
    else if (Referee_system_Data.Transmit_Data_1.Aim_Status == 1) {
        Sentry_Robot_Status.Aim_Status = Lock_Robot;
    }
    else if (Referee_system_Data.Transmit_Data_1.Aim_Status == 2) {
        Sentry_Robot_Status.Aim_Status = Shoot;
    }

    Competition_Referee_Error_Event_Timer.setPeriod(ros::Duration(15));
    Start_Referee_Error_Event_Timer.setPeriod(ros::Duration(240));
    Finish_Receive_Referee_Flag = true;
}

void Sentry_Master::Analysis_Referee_Data()
{
    
    if (Finish_Receive_Referee_Flag) 
    {
        this->Analysis_Competition_Stage();
        if (Sentry_Robot_Status.Referee_Status == Start_Competition) {
            /* 经历状态变化则执行决策，此属战况告急，可以忽略视觉锁定情况 */
            if (Sentry_Robot_Status.Competition_Stage == Defend1 && Last_Sentry_Robot_Status.Competition_Stage != Defend1) {
                this->UnStop();
                this->pub_goal_point(Outpose_Out_Area);
                Ji_Ji_Ji_Ji_Ji = true;
                return;
            }
            if (Sentry_Robot_Status.Competition_Stage == Defend2 && Last_Sentry_Robot_Status.Competition_Stage != Defend2) {
                this->UnStop();
                this->pub_goal_point(Outpose_In_Area);
                Ji_Ji_Ji_Ji_Ji = true;
                return;
            }
            if (Sentry_Robot_Status.Competition_Stage == Return_Patrol_Area && Last_Sentry_Robot_Status.Competition_Stage != Return_Patrol_Area) {
                this->UnStop();
                this->pub_goal_point(Patrol_Area);
                Ji_Ji_Ji_Ji_Ji = true;
                return;
            }
            if (Sentry_Robot_Status.Aim_Status == UnLock_Robot && Sentry_Robot_Status.Competition_Stage >= 4) {
                this->UnStop();
                this->pose_world_to_sentry(); 
                this->Update_HP_Information();
                this->Compute_Event_Score();
                this->Sort_Event_Score();
                if (Max_Score.second > Event_Score_Thre) {
                    if (Max_Score.first == "Hero_Event_Score") {
                        if (!Near_Hero_Flag) {
                        
                        }
                    }
                    else if (Max_Score.first == "Infantry3_Event_Score") {
                        if (!Near_Infantry3_Flag) {
                        
                        }
                    
                    }
                    else if (Max_Score.first == "Infantry4_Event_Score") {
                        if (!Near_Infantry4_Flag) {
                            
                        }
                    }
                }
            }
            else {
                /* 跑路很急，不能停 */
                if (!Ji_Ji_Ji_Ji_Ji) {
                    this->Stop();
                }
            }
        }
        else if (Sentry_Robot_Status.Referee_Status == Referee_Error) {
            Referee_system_Data = Empty_Referee_system_Data;
            /* 裁判系统错误但是不在巡逻区 */
            if (!Error_But_In_Patrol_Area) {
                if (std::fabs(Sentry_Odometry.pose.pose.position.x) >= 0.5 || std::fabs(Sentry_Odometry.pose.pose.position.y) >= 0.5) {
                    this->pub_goal_point(Patrol_Area);
                }
                else {
                    Error_But_In_Patrol_Area = true;
                }
            }
            else {
                this->Stop();
            }
        }
        Finish_Receive_Referee_Flag = false;
    }
}

void Sentry_Master::Analysis_Competition_Stage()
{
    /* 首次接收裁判系统数据或者断线重连之后且哨兵仍存活更新裁判系统状态 */
    if ((Sentry_Robot_Status.Referee_Status == Unconnected_Competition || Sentry_Robot_Status.Referee_Status == Referee_Error) &&  Referee_system_Data.Transmit_Data_1.Robot_HP_Data.Sentry_HP != 0) {
        if (Referee_system_Data.Transmit_Data_1.Game_Status.game_progress == 1 || Referee_system_Data.Transmit_Data_1.Game_Status.game_progress == 2 || Referee_system_Data.Transmit_Data_1.Game_Status.game_progress == 3) {
            Sentry_Robot_Status.Referee_Status = Preparation_match;
            Start_Competition_Flag = false;
        }
        else if (Referee_system_Data.Transmit_Data_1.Game_Status.game_progress == 4) {
            Sentry_Robot_Status.Referee_Status = Start_Competition;
            Start_Competition_Flag = false;
        }
    }
    if (Sentry_Robot_Status.Referee_Status == Preparation_match && Referee_system_Data.Transmit_Data_1.Game_Status.game_progress == 4) {
        Sentry_Robot_Status.Referee_Status = Start_Competition;
    }
    /* 比赛开始且哨兵存活 */
    if (Sentry_Robot_Status.Referee_Status == Start_Competition && Referee_system_Data.Transmit_Data_1.Robot_HP_Data.Sentry_HP != 0 ) {
        Last_Sentry_Robot_Status.Competition_Stage = Sentry_Robot_Status.Competition_Stage;
        if (Referee_system_Data.Transmit_Data_1.Robot_HP_Data.Outpost_HP >= 800) {
            // 进攻个屁
        }
        else if (Referee_system_Data.Transmit_Data_1.Robot_HP_Data.Outpost_HP < 800) {
            Sentry_Robot_Status.Competition_Stage = Defend1;
        }
        else if (Referee_system_Data.Transmit_Data_1.Robot_HP_Data.Outpost_HP < 500) {
            Sentry_Robot_Status.Competition_Stage = Defend2;
        }
        else if (Referee_system_Data.Transmit_Data_1.Robot_HP_Data.Outpost_HP < 250) {
            Sentry_Robot_Status.Competition_Stage = Return_Patrol_Area;
        }

        /* 达到就近位置“别急” */
        if (Ji_Ji_Ji_Ji_Ji) {
            if (Sentry_Robot_Status.Competition_Stage == Defend1) {
                Ji_Ji_Ji_Ji_Ji = this->Odom_Dis_Goal(Outpose_Out_Area); // 别急
            }
            else if (Sentry_Robot_Status.Competition_Stage == Defend2) {
                Ji_Ji_Ji_Ji_Ji = this->Odom_Dis_Goal(Outpose_In_Area);  // 别急
            }
            else if (Sentry_Robot_Status.Competition_Stage == Return_Patrol_Area) {
                Ji_Ji_Ji_Ji_Ji = this->Odom_Dis_Goal(Patrol_Area);      // 别急
            }
        }

        /* 开局前往控制区，只发送一次 */
        if (!Start_Competition_Flag) {
            this->pub_goal_point(Control_Area);
            Sentry_Robot_Status.Competition_Stage = Start_Control_Area;
            Start_Competition_Flag = true;
        }

        /* 开局识别阵容，无法识别则放弃某些高危决策的执行，识别后不再执行 */
        if (Sentry_Robot_Status.Robot_Lineup == Unknown) {
            if (Referee_system_Data.Transmit_Data_1.Robot_Lineup == 1 && Referee_system_Data.Transmit_Data_2.Robot_Pose.Sentry_X < 14) {
                Sentry_Robot_Status.Robot_Lineup = Red;
            }
            else if (Referee_system_Data.Transmit_Data_1.Robot_Lineup == 2 && Referee_system_Data.Transmit_Data_2.Robot_Pose.Sentry_X > 14) {
                Sentry_Robot_Status.Robot_Lineup = Blue;
            }
        }
    }
}

/**
 * 3号步兵：血量优先
 * 4号步兵：功率优先
 * 1号英雄：血量优先
 */

void Sentry_Master::Compute_Event_Score()
{
    // 非撞击伤害
    if (dHero_HP_Last >= 10) {    
        float Hero_Event_Score = dHero_HP_Start * Hero_Robot_Weight;
        if (Hero_Event_Score >= Event_Score_Thre) {
            Event_Score.at("Hero_Event_Score") = Hero_Event_Score;
        }
    }
    if (dInfantry3_HP_Last >= 10) {
        float Infantry3_Event_Score = dInfantry3_HP_Start * Infantry3_Robot_Weight;
        if (Infantry3_Event_Score >= Event_Score_Thre) {
            Event_Score.at("Infantry3_Event_Score") = Infantry3_Event_Score;
        }
    }
    if (dInfantry4_HP_Last >= 10) {
        float Infantry4_Event_Score = dInfantry4_HP_Start * Infantry4_Robot_Weight;
        if (Infantry4_Event_Score >= Event_Score_Thre) {
            Event_Score.at("Infantry4_Event_Score") = Infantry4_Event_Score;
        }
    }
}

void Sentry_Master::Sort_Event_Score()
{
    std::vector<std::pair<std::string, float>> Event_Score_vec(Event_Score.begin(), Event_Score.end());
    std::sort(Event_Score_vec.begin(), Event_Score_vec.end(), CmpByValue());

    this->Max_Score = Event_Score_vec[0];
}

void Sentry_Master::pose_world_to_sentry()
{
    hero_x_sentry = Referee_system_Data.Transmit_Data_2.Robot_Pose.Hero_X - Referee_system_Data.Transmit_Data_2.Robot_Pose.Sentry_X;
    hero_y_sentry = Referee_system_Data.Transmit_Data_2.Robot_Pose.Hero_Y - Referee_system_Data.Transmit_Data_2.Robot_Pose.Sentry_Y;
    engineer_x_sentry = Referee_system_Data.Transmit_Data_2.Robot_Pose.Engineer_X - Referee_system_Data.Transmit_Data_2.Robot_Pose.Sentry_X;
    engineer_y_sentry = Referee_system_Data.Transmit_Data_2.Robot_Pose.Engineer_Y - Referee_system_Data.Transmit_Data_2.Robot_Pose.Sentry_Y;
    standard_3_x_sentry = Referee_system_Data.Transmit_Data_2.Robot_Pose.Infantry_3_X - Referee_system_Data.Transmit_Data_2.Robot_Pose.Sentry_X;
    standard_3_y_sentry = Referee_system_Data.Transmit_Data_2.Robot_Pose.Infantry_3_Y - Referee_system_Data.Transmit_Data_2.Robot_Pose.Sentry_Y;
    standard_4_x_sentry = Referee_system_Data.Transmit_Data_2.Robot_Pose.Infantry_4_X - Referee_system_Data.Transmit_Data_2.Robot_Pose.Sentry_X;
    standard_4_y_sentry = Referee_system_Data.Transmit_Data_2.Robot_Pose.Infantry_4_Y - Referee_system_Data.Transmit_Data_2.Robot_Pose.Sentry_Y;
}

void Sentry_Master::pub_goal_point(double x, double y, double z)
{
    sentry_goal_point.point.x = x;
    sentry_goal_point.point.y = y;
    sentry_goal_point.point.z = z;

    sentry_goal_pub.publish(sentry_goal_point);
}

void Sentry_Master::pub_goal_point(Map_Point &point)
{
    sentry_goal_point.point.x = point.x;
    sentry_goal_point.point.y = point.y;
    sentry_goal_point.point.z = point.z;

    sentry_goal_pub.publish(sentry_goal_point);
}
void Sentry_Master::Update_HP_Information()
{
    float Last_dHero_HP_Start = dHero_HP_Start;
    float Last_dInfantry3_HP_Start = dInfantry3_HP_Start;
    float Last_dInfantry4_HP_Start = dInfantry4_HP_Start;
    dHero_HP_Start = Referee_system_Data.Transmit_Data_1.Robot_HP_Data.Hero_HP - Robot_Start_Data.Transmit_Data_1.Robot_HP_Data.Hero_HP;
    if (dHero_HP_Start > 0) {
        Hero_Level = (Hero_Level == 1) ? 2 : 3;
        if (Hero_Level == 2) {
            Robot_Start_Data.Transmit_Data_1.Robot_HP_Data.Hero_HP = 350;   // 英雄：血量优先
        }
        else if (Hero_Level == 3) {
            Robot_Start_Data.Transmit_Data_1.Robot_HP_Data.Hero_HP = 450;
        }
    }
    dEngineer_HP_Start = Referee_system_Data.Transmit_Data_1.Robot_HP_Data.Engineer_HP - Robot_Start_Data.Transmit_Data_1.Robot_HP_Data.Engineer_HP;
    dInfantry3_HP_Start = Referee_system_Data.Transmit_Data_1.Robot_HP_Data.Infantry_3_HP - Robot_Start_Data.Transmit_Data_1.Robot_HP_Data.Infantry_3_HP;
    if (dInfantry3_HP_Start > 0) {
        Infantry3_Level = (Infantry3_Level == 1) ? 2 : 3;
        if (Infantry3_Level == 2) {
            Robot_Start_Data.Transmit_Data_1.Robot_HP_Data.Infantry_3_HP = 300; // 三号步兵：血量优先
        }
        else if (Infantry3_Level == 3) {
            Robot_Start_Data.Transmit_Data_1.Robot_HP_Data.Infantry_3_HP = 400;
        }
    }
    dInfantry4_HP_Start = Referee_system_Data.Transmit_Data_1.Robot_HP_Data.Infantry_4_HP - Robot_Start_Data.Transmit_Data_1.Robot_HP_Data.Infantry_4_HP;
    if (dInfantry4_HP_Start > 0) {
        Infantry4_Level = (Infantry4_Level == 1) ? 2 : 3;
        if (Infantry4_Level == 2) {
            Robot_Start_Data.Transmit_Data_1.Robot_HP_Data.Infantry_4_HP = 200; // 四号步兵：功率优先
        }
        else if (Infantry4_Level == 3) {
            Robot_Start_Data.Transmit_Data_1.Robot_HP_Data.Infantry_4_HP = 250;
        }
    }

    dHero_HP_Last = Referee_system_Data.Transmit_Data_1.Robot_HP_Data.Hero_HP - Robot_Last_Data.Transmit_Data_1.Robot_HP_Data.Hero_HP;
    dEngineer_HP_Last = Referee_system_Data.Transmit_Data_1.Robot_HP_Data.Engineer_HP - Robot_Last_Data.Transmit_Data_1.Robot_HP_Data.Engineer_HP;
    dInfantry3_HP_Last = Referee_system_Data.Transmit_Data_1.Robot_HP_Data.Infantry_3_HP - Robot_Last_Data.Transmit_Data_1.Robot_HP_Data.Infantry_3_HP;
    dInfantry4_HP_Last = Referee_system_Data.Transmit_Data_1.Robot_HP_Data.Infantry_4_HP - Robot_Last_Data.Transmit_Data_1.Robot_HP_Data.Infantry_4_HP;
}

void Sentry_Master::Stop()
{
    if (!Publish_Stop_Flag) {
        std_msgs::Int8 data;
        data.data = 2;
        Lock_Stop_pub.publish(data);
        Publish_Stop_Flag = true;
    }
}

void Sentry_Master::UnStop()
{
    if (Publish_Stop_Flag) {
        std_msgs::Int8 data;
        data.data = 0;
        Lock_Stop_pub.publish(data);
        Publish_Stop_Flag = false;
    }
}

bool Sentry_Master::Odom_Dis_Goal(Map_Point &goal)
{
    return (std::fabs((Sentry_Odometry.pose.pose.position.x) - goal.x) >= 0.15 && std::fabs((Sentry_Odometry.pose.pose.position.y - goal.y) >= 0.15)); 
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "Sentry_Master");
    Sentry_Master sentry_master;
    while (ros::ok()) {
    	sentry_master.Analysis_Referee_Data();
    	ros::spinOnce();
    }

    return 0;
}

