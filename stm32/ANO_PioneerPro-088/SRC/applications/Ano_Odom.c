/*==========================================================================
 * 描述    ：通过对速度积分生成里程计数据供给SLAM使用
 * 更新时间：2025-06-21 
 * 作者		 ：RZR
============================================================================*/


#include "Ano_Odom.h"
#include "Ano_LocCtrl.h"
#include "Ano_OF.h"
#include "Ano_Comp_Link.h"
#include "Ano_Imu.h"

#define _VEL_X      (ano_of.of2_dx_fix)
#define _VEL_Y      (ano_of.of2_dy_fix)
#define _REF_X      ((float)pos.hx_x / 10000.0f)
#define _REF_Y      ((float)pos.hx_y / 10000.0f)


Position2D displacement;

// 位移计算函数
void CalculateDisplacement(float delta_time_sec) 
{
    // 机体坐标系速度
    float body_vel[VEC_XYZ] = {0};

    // 世界坐标速度
    float world_vel[VEC_XYZ] = {0};

    // 机头方向向量（cos/sin值）
    float ref_ax[VEC_XYZ] = {0};

    ref_ax[X] = _REF_X;
    ref_ax[Y] = _REF_Y;

    body_vel[X] = _VEL_X;
    body_vel[Y] = _VEL_Y;

    h2w_2d_trans(body_vel, ref_ax, world_vel);

    // 计算微小位移增量 (Δd = v * Δt)
    float delta_x = world_vel[X] * delta_time_sec;
    float delta_y = world_vel[Y] * delta_time_sec;
    
    // 累加到总位移
    displacement.X += delta_x;
    displacement.Y += delta_y;

}

void PosCalculation_Task(u8 dT_ms) 
{   
	if(comp_offline)
	{
		displacement.X = 0;
		displacement.Y = 0;
		return;
	};	
    // 步骤1：将毫秒时间转换为秒（用于速度积分）
    float dT_sec = (float)dT_ms / 1000.0f;
    // 步骤2：执行位移计算
    CalculateDisplacement(dT_sec);
}
