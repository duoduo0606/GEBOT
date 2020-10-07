//
// Created by Yuhang Xing on 10/6/20.
//

//------------------------------------------------------------
//头文件-------------------------------------------------------
//------------------------------------------------------------
#include <iostream>
#include <vector>
#include <cmath>


//------------------------------------------------------------
//资料---------------------------------------------------------
//------------------------------------------------------------
/*
int len;
cin>>len;
vector<int> array(len);//声明变长数组
for(int i=0;i<len;i++)
{
array[i]=i;
cout<<array[i]<<"\t";
*/


//------------------------------------------------------------
//类定义-------------------------------------------------------
//------------------------------------------------------------
class Gebot_Gait                    //步态参数设置 函数功能设置
{
public:

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
    struct Gait_Para gait_para_state[],gaitPara;

    //赋初值---------------------------------------------------
    time_int = 0.1;
    time_step = 1;

    struct Gait_para gait_para_state[] =
            {
                //{step_length, lateral_displacement, step_hight, yaw_angle}
                {0,0,0,0}, // STOP
                {0,0,0,0}, // FORWARD
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
    void gait_planning()
    {
        //FORWARD
        //三角步态
        RF = lifting_foot_trajectory_gene(gaitPara)





    }

    //足端轨迹--------------------------------------------------
    //说明 para_temp包含了状态关系，对应状态的步态参数
    void lifting_foot_trajectory_gene(struct Gait_para para_temp)
    {
        //定义变量
        float foot_px[],foot_py[],foot_pz[];
        float diff_px,diff_py,diff_pz;
        float old_px,old_py,old_pz;
        int res,i;

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

            DH_inv_kinematics(diff_px,diff_py,diff_pz,para_temp.yaw_angle)
        }
    }

    void supporting_foot_trajectory_gene(struct Gait_para para_temp)//para_temp与移动脚一致，具体移动参数在函数中处理
    {
        //定义变量
        float foot_px[],foot_py[],foot_pz[];
        float diff_px,diff_py,diff_pz;
        float old_px,old_py,old_pz;
        int res,i;

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

            DH_inv_kinematics(diff_px,diff_py,diff_pz,para_temp.yaw_angle)
        }
    }

    //逆运动学----------------------------------------------------
    void DH_inv_kinematics(float px, float py, float pz, float yaw)
    {
        //
    }





    /*void toSTOP()
    void toFORWARD()
    void toLEFT()*/

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



