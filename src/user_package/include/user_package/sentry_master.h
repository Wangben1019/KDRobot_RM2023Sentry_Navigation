#ifndef _REFEREE_SYSTEM_H_
#define _REFEREE_SYSTEM_H_
#include "ros/node_handle.h"
#include "ros/publisher.h"
#include "ros/ros.h"
#include "geometry_msgs/PointStamped.h"
#include "ros/subscriber.h"
#include "nav_msgs/Odometry.h"
#include <cstdint>
#include "ros/timer.h"
#include "string"
#include "vector"
#include "map"
#include <algorithm>
typedef enum {
    Unconnected_Competition = 0,    //!<@brief 未接收裁判系统信息
    Referee_Error,                  //!<@brief 裁判系统通信错误
    Preparation_match,              //!<@brief 赛前准备
    Start_Competition,              //!<@brief 开始比赛
} Referee_Status_;

typedef enum {
    None = 0,
    Return_Patrol_Area = 1,             //!<@brief 返回巡逻区
    Defend1 = 2,                        //!<@brief 一级防守
    Defend2 = 3,                        //!<@brief 二级防守
    Start_Control_Area = 4,             //!<@brief 开始应处于控制区
    Attack1 = 5,                        //!<@brief 一级进攻
    Attack2 = 6,                        //!<@brief 二级进攻
} Competition_Stage_;

/**
 * @brief 机器人阵容枚举体
 */
typedef enum {
    Unknown = 0,    //!<@brief 未知
    Red,            //!<@brief 红方
    Blue            //!<@brief 蓝方
} Robot_Lineup_;

typedef enum {
    UnLock_Robot = 0,   //!<@brief 未锁定敌方机器人
    Lock_Robot,         //!<@brief 已锁定敌方机器人
    Shoot               //!<@brief 发射
} Aim_Status_;

typedef struct {
    Referee_Status_ Referee_Status;
    Competition_Stage_ Competition_Stage;
    Robot_Lineup_ Robot_Lineup;
    Aim_Status_ Aim_Status;
} Sentry_Robot_Status_Typedef;

/**
 * @brief 比赛状态信息结构体
 */
typedef struct {
    uint8_t game_progress;      //!<@brief 比赛进行阶段
    uint16_t stage_remain_time; //!<@brief 比赛剩余时间
} Game_Status_t;

/**
 * @brief 比赛结果信息结构体
 */
typedef struct {
    uint8_t winner;     //!<@brief 比赛结果
} Game_Result_t;

/**
 * @brief 机器人血量信息结构体
 */
typedef struct {
    uint16_t Hero_HP;       //!<@brief 英雄机器人血量
    uint16_t Engineer_HP;   //!<@brief 工程机器人血量
    uint16_t Infantry_3_HP; //!<@brief 三号步兵机器人血量
    uint16_t Infantry_4_HP; //!<@brief 四号步兵机器人血量
    uint16_t Sentry_HP;     //!<@brief 哨兵机器人血量
    uint16_t Outpost_HP;    //!<@brief 前哨站血量
    uint16_t Base_HP;       //!<@brief 基地血量
} Robot_HP_Data_t;

/**
 * @brief 发送缓冲区1
 */
typedef struct {
    Game_Status_t Game_Status;
    Robot_HP_Data_t Robot_HP_Data;
    uint16_t Allow_Shoot_Num;
    uint8_t Aim_Status;
    uint8_t Robot_Lineup;
} Transmit_Data_1_t;

/**
 * @brief 机器人位置信息结构体
 */
typedef struct {
    float Hero_X;       //!<@brief 英雄机器人X轴坐标
    float Hero_Y;       //!<@brief 英雄机器人Y轴坐标
    float Engineer_X;   //!<@brief 工程机器人X轴坐标
    float Engineer_Y;   //!<@brief 工程机器人Y轴坐标
    float Infantry_3_X; //!<@brief 三号步兵机器人X轴坐标
    float Infantry_3_Y; //!<@brief 三号步兵机器人Y轴坐标
    float Infantry_4_X; //!<@brief 四号步兵机器人X轴坐标
    float Infantry_4_Y; //!<@brief 四号步兵机器人Y轴坐标
    float Sentry_X;     //!<@brief 哨兵机器人X轴坐标
    float Sentry_Y;     //!<@brief 哨兵机器人Y轴坐标
} Robot_Pose_t;

/**
 * @brief 发送缓冲区2
 */
typedef struct {
    Robot_Pose_t Robot_Pose;    
} Transmit_Data_2_t;

typedef struct{
    Transmit_Data_1_t Transmit_Data_1;
    Transmit_Data_2_t Transmit_Data_2;
} Referee_system_Data_t;

class Map_Point{
public:
    float x;
    float y;
    float z;
    Map_Point(float x, float y, float z)
    {
        this->x = x;
        this->y = y;
        this->z = z;
    }
};

class Sentry_Master{
private:
    ros::NodeHandle nh;
    ros::Publisher sentry_goal_pub;
    ros::Publisher Lock_Stop_pub;
    ros::Subscriber Odom_msg_Sub;
    ros::Timer Pub_Goal_Interval_Timer;
    ros::Timer Start_Referee_Error_Event_Timer;
    ros::Timer Competition_Referee_Error_Event_Timer;
    ros::Timer UnLock_Robot_Timer;

    geometry_msgs::PointStamped sentry_goal_point;
    nav_msgs::Odometry Sentry_Odometry;

    Referee_system_Data_t Referee_system_Data;
    Referee_system_Data_t Empty_Referee_system_Data;
    Sentry_Robot_Status_Typedef Sentry_Robot_Status;
    Sentry_Robot_Status_Typedef Last_Sentry_Robot_Status;

    float hero_x_sentry;
    float hero_y_sentry;
    float engineer_x_sentry;
    float engineer_y_sentry;
    float standard_3_x_sentry;
    float standard_3_y_sentry;
    float standard_4_x_sentry;
    float standard_4_y_sentry;
    float standard_5_x_sentry;
    float standard_5_y_sentry;
    bool Finish_Receive_Referee_Flag = false;
    bool Publish_Stop_Flag = false;
    bool Error_But_In_Patrol_Area = false;
    bool Start_Competition_Flag = false;
    bool Ji_Ji_Ji_Ji_Ji = false; // false别急，true丢盔弃甲跑路
    bool Can_Make_Decisions = false;
    bool Referee_Error_Integral = false;

    Referee_system_Data_t Robot_Start_Data;
    Referee_system_Data_t Robot_Last_Data;

    // float Event_Score;
    float Event_Score_Thre = 100;

    float Hero_Robot_Weight = 0.5;
    float Infantry3_Robot_Weight = 0.6667;
    float Infantry4_Robot_Weight = 1;

    uint8_t Hero_Level = 1;
    uint8_t Infantry3_Level = 1;
    uint8_t Infantry4_Level = 1;

    float dHero_HP_Start = 0;
    float dEngineer_HP_Start = 0;
    float dInfantry3_HP_Start = 0;
    float dInfantry4_HP_Start = 0;

    float dHero_HP_Last = 0;
    float dEngineer_HP_Last = 0;
    float dInfantry3_HP_Last = 0;
    float dInfantry4_HP_Last = 0;

    std::map<std::string, float> Event_Score{{"Hero_Event_Score", 0},
                                             {"Infantry3_Event_Score", 0},
                                             {"Infantry4_Event_Score", 0}};

    std::pair<std::string, float> Max_Score;
    bool Near_Hero_Flag = false;
    bool Near_Infantry3_Flag = false;
    bool Near_Infantry4_Flag = false;
    bool Near_Robot_Flag = false;

public:
    Sentry_Master() = default;
    ~Sentry_Master() = default;
    void Master_Init();
    void Master_Loop();
    void pub_goal_point(double x, double y, double z);
    void pub_goal_point(Map_Point &point);
    void pose_world_to_sentry();
    void receive_data();
    void Analysis_Referee_Data();
    void Analysis_Competition_Stage();
    void Update_HP_Information();
    void Compute_Event_Score();
    void Sort_Event_Score();
    void odomHandler(const nav_msgs::Odometry::ConstPtr& odom);
    void Timer_Event_cb(const ros::TimerEvent& event);
    void Timer_Start_Referee_Error_Event_cb(const ros::TimerEvent& event);
    void Timer_Competition_Referee_Error_Event_cb(const ros::TimerEvent& event);
    void Stop();
    void UnStop();
    bool Odom_Dis_Goal(Map_Point &goal);
};


#endif
