//
// Created by Yuhang Xing on 10/6/20.
//

//------------------------------------------------------------
//头文件-------------------------------------------------------
//------------------------------------------------------------
#include <iostream>
#include <vector>
#include <cmath>
#include "position_controll.h"

using namespace std;


Gait_para gait_para_state[] =
        {
                //{step_length, lateral_displacement, step_hight, yaw_angle}
                {0,0,0,0}, // STOP
                {60,0,0,0}, // FORWARD
                {0,0,0,0}, // BACK
                {0,0,0,0}, // LEFT
                {0,0,0,0}, // RIGHT
                {0,0,0,0}, // LEFT_TURN
                {0,0,0,0}, // RIGHT_TURN
                {0,0,0,0}, // RESET

        };

//计算-----------------------------------------------------
resolution_ratio = time_step/time_int;

//步态规划 顺序间隔------------------------------------------
void gait_planning(state)
{

    //三角步态
    vector<vector<float>> joint_theta_pack[][];

    gaitPara = gait_para_state[FORWARD];

    joint_theta_pack[0] = lifting_foot_trajectory_gene(gaitPara);
    joint_theta_pack[1] = supporting_foot_trajectory_gene(gaitPara);
    joint_theta_pack[2] = supporting_foot_trajectory_gene(gaitPara);
    joint_theta_pack[3] = supporting_foot_trajectory_gene(gaitPara);

}

vector<float> Gebot_Gait::lifting_foot_trajectory_gene(struct Gait_para para_temp)
{
    //定义变量
    float foot_px[],foot_py[],foot_pz[];
    float diff_px,diff_py,diff_pz;
    float old_px,old_py,old_pz;
    int res,i;
    vector<float> theta;

    //赋初值
    res = resolution_ratio;
    foot_px[0] = 0;
    foot_py[0] = 0;
    foot_pz[0] = 0;

    //设定轨迹曲线，离散存储，调用逆运动学函数
    for (i = 0; i < res+1; i++)
    {
        old_px = foot_px[i];
        old_py = foot_py[i];
        old_pz = foot_pz[i];

        foot_px[i] = i*para_temp.step_length/res;
        foot_py[i] = i*para_temp.lateral_displacement/res;
        foot_pz[i] = -2*para_temp.step_hight/powf(para_temp.step_length,2)*foot_px[i]*(foot_px[i]-para_temp.step_length);

        diff_px = foot_px[i] - old_px;
        diff_py = foot_py[i] - old_py;
        diff_pz = foot_pz[i] - old_pz;

        theta = DH_inv_kinematics(diff_px,diff_py,diff_pz,para_temp.yaw_angle);
        return theta;
    }
}

Gebot_Gait::supporting_foot_trajectory_gene(struct Gait_para para_temp)//para_temp与移动脚一致，具体移动参数在函数中处理
{
    //定义变量
    float foot_px[],foot_py[],foot_pz[];
    float diff_px,diff_py,diff_pz;
    float old_px,old_py,old_pz;
    int res,i;
    vector<float> theta;

    //赋初值
    res = resolution_ratio;
    foot_px[0] = 0;
    foot_py[0] = 0;
    foot_pz[0] = 0;

    //设定轨迹曲线，离散存储，调用逆运动学函数
    for (i = 0; i < res+1; i++)
    {
        old_px = foot_px[i];
        old_py = foot_py[i];
        old_pz = foot_pz[i];

        foot_px[i] = -1/3*i*para_temp.step_length/res;
        foot_py[i] = -1/3*i*para_temp.lateral_displacement/res;

        diff_px = foot_px[i] - old_px;
        diff_py = foot_py[i] - old_py;
        diff_pz = foot_pz[i] - old_pz;

        theta = DH_inv_kinematics(diff_px,diff_py,diff_pz);

        return theta;
    }
}

//逆运动学----------------------------------------------------
Gebot_Gait::DH_inv_kinematics(float px, float py, float pz)
{
    //定义变量
    vector<float> theta;
    float l1,l2,l3;
    float m;

    //theta.push_back(atan2(-px,py) - atan2(sqrt(px*px+py*py-l3*l3),l3));

    theta[0] = atan2(-px,py) - atan2(sqrt(px*px+py*py-l3*l3),l3);
    m = px*cos(theta[0]) + py*sin(theta[0]);
    theta[2] = acos((m*m+pz*pz-l1*l1-l2*l2)/(2*l1*l2));
    theta[1] = atan2(pz*(l1+l2*cos(theta[2]))-l2*sin(theta[2]),m*(l1+l2*cos(theta[2]))+pz*l2*sin(theta[2]));

    return theta;
}


};
//------------------------------------------------------------




/*
 *在每一个状态机的状态中，完成对各个步态参数的配置，并通过函数完成对步态参数的处理分析，规划出最终步态，和传递给电机的参数
 */
switch(state)
{
    case STOP:
    {
        //
    }
    break;

    case FORWARD:
    {
        //
    }
    break;

    case BACK:
    {
        //
    }
    break;

    case LEFT:
    {
        //
    }
    break;

    case RIGHT:
    {
        //
    }
    break;

    case LEFT_TURN:
    {
        //
    }
    break;

    case RIGHT_TURN:
    {
        //
    }
    break;

    case RESET:
    {
        //
    }
    break;

}



