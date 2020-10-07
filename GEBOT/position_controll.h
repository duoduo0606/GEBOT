//
// Created by Yuhang Xing on 10/6/20.
//

#ifndef POSITION_CONTROLL_POSITION_CONTROLL_H
#define POSITION_CONTROLL_POSITION_CONTROLL_H

using namespace std;

enum States
        {
            STOP = 0,
            FORWARD = 1,
            BACK = 2,
            LEFT = 3,
            RIGHT = 4,
            LEFT_TURN = 5,
            RIGHT_TURN = 6,
            RESET = 7,
        };

//------------------------------------------------------------
//类定义-------------------------------------------------------
//------------------------------------------------------------
class Gebot_Gait                    //步态参数设置 函数功能设置
{
public:
    Gebot_Gait();
    //定义结构体-----------------------------------------------
    struct Gait_para
    {
        float step_length;				//步距
        float lateral_displacement;		//侧移
        float step_hight;               //步高
        float yaw_angle;				//偏航角
    };

    //定义参数-------------------------------------------------
    float time_int;                 //时间间隔
    float time_step;                //一次抬腿的时间
    int resolution_ratio;
    Gait_para gaitPara;
    int state;

    //赋初值---------------------------------------------------
    //time_int = 0.1;
    //time_step = 1;

    /*
    struct Gait_para gait_para_state[] =
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
    */

    //计算-----------------------------------------------------
    //resolution_ratio = time_step/time_int;

    //步态规划 顺序间隔------------------------------------------
    void gait_planning();

    // vector<float> lifting_foot_trajectory_gene(struct Gait_para para_temp);
    // vector<float> supporting_foot_trajectory_gene(struct Gait_para para_temp);
    // vector<float> DH_inv_kinematics(float px, float py, float pz);
};

#endif //POSITION_CONTROLL_POSITION_CONTROLL_H
