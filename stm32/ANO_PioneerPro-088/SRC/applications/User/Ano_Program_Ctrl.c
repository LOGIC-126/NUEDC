/*==========================================================================
 * 描述    ：自动程序飞行接口
 * 更新时间：2025-07-12 
 * 作者		 ：RZR
============================================================================*/
#include "Ano_Program_Ctrl.h"
#include "Ano_Pid.h"
#include "Ano_Parameter.h"
#include "Ano_OF.h"
#include "Ano_ProgramCtrl_User.h"
#include "Ano_Math.h"
#include "Ano_FlightCtrl.h"
#include "Ano_RC.h"
#include "Ano_Imu.h"
#include "Ano_Comp_Link.h"

/* ///////////////////////////////////////////////////
    高度环pid程序控制接口
*/ ///////////////////////////////////////////////////

// 高度PID参数
_PID_arg_st alt_pid_arg;
_PID_val_st alt_pid_val;

// 可能会用的flag变量
// flag.unlock_sta
// s16 CH_N[CH_NUM]

void Alt_pid_init_13(void)
{
	alt_pid_arg.kp = Ano_Parame.set.halt_pid_13[KP];
	alt_pid_arg.ki = Ano_Parame.set.halt_pid_13[KI];
	alt_pid_arg.kd_ex= 0;
	alt_pid_arg.kd_fb = Ano_Parame.set.halt_pid_13[KD];	
	alt_pid_arg.k_ff = 0;
}

_pid_st altPid;
u8 Test_flag = 0;
u8 Alt_Set_flag = 1;

void Alt_Data_Pid_Task(u8 dT_ms,float Target_Alt_val,u8 en)
{
    static float pid_alt_out = 0.00f;
    // static u16 timtmp = 0;			// 定时变量，在某些需要时间t的情况下使用
    if(flag.auto_take_off_land == AUTO_TAKE_OFF_NULL && // 无自动起飞任务
       (flag.taking_off == 0) &&
       (Target_Alt_val > 0.0f) &&           // 目标高度大于0
       (CH_N[CH6]> 400)                     // 开关打开
    )          
    {
        // flag.unlock_cmd = 1;
        one_key_take_off();  // 一键起飞
    }
    
    if (Target_Alt_val == 0.0f && flag.taking_off == 1 && en)
    {
        /* 降落 */
        one_key_land();
    }
    
    if ((en) && (flag.taking_off == 1) && (flag.auto_take_off_land == AUTO_TAKE_OFF_FINISH))
    {
        /* 使能高度pid */
        altPid.exp = Target_Alt_val;   // 期望高度
        altPid.fb  = ano_of.of_alt_cm;                      // 反馈高度
        
        PID_calculate( dT_ms*1e-3f,            //周期（单位：秒）
                                0 ,				//前馈值
                                altPid.exp ,				//期望值（设定值）
                                altPid.fb ,			//反馈值（）
                                &alt_pid_arg,   //PID参数结构体
                                &alt_pid_val,	//PID数据结构体
                                50,//积分误差限幅
                                10 *flag.taking_off			//integration limit，积分限幅
                                    )	;
        pid_alt_out=alt_pid_val.out;
        Program_Ctrl_User_Set_Zcmps(LIMIT(pid_alt_out,-16,15));
    }
    else
    {
		Program_Ctrl_User_Set_Zcmps(0);
    }
    
}

void Alt_ProgramCtrl_Task(u8 dT_ms)
{
    float Alt_Ctrl_val;
    u8 FC_Ctrl_flag;
    u8 en_flag;

    static float last_manual_alt = 0;   // 保存上次手动高度
     /* 判断遥控通道值 */
    if ((CH_N[CH_THR] < -15)||(CH_N[CH_THR] > 15))     //手动控制油门状态
    {
        FC_Ctrl_flag = 1;
        last_manual_alt = ano_of.of_alt_cm;    // 控制高度等于当前高度
    }
    else
    {
        FC_Ctrl_flag = 0;       // 遥控回中
		if(1){
			Alt_Ctrl_val = tar.z;
//			Alt_Ctrl_val = 80;
		}
		else
		{
			Alt_Ctrl_val = last_manual_alt;
		}
    }
    if (
    (FC_Ctrl_flag == 0) && (CH_N[CH6]> 400)
    )      // 如果在中位，通道6开启，启动pid控制
        en_flag = 1;
    else
        en_flag = 0;
    Alt_Data_Pid_Task(dT_ms,Alt_Ctrl_val,en_flag);
}


/* ///////////////////////////////////////////////////
    位移环pid程序控制接口
*/ ///////////////////////////////////////////////////

// 位移PID参数
_PID_arg_st disX_pid_arg;
_PID_val_st disX_pid_val;

_PID_arg_st disY_pid_arg;
_PID_val_st disY_pid_val;

void Dis_pid_init_14(void)
{
	disX_pid_arg.kp = Ano_Parame.set.xy_pid_14[KP];
	disX_pid_arg.ki = Ano_Parame.set.xy_pid_14[KI];
	disX_pid_arg.kd_ex= 0;
	disX_pid_arg.kd_fb = Ano_Parame.set.xy_pid_14[KD];	
	disX_pid_arg.k_ff = 0;

	disY_pid_arg.kp = Ano_Parame.set.xy_pid_14[KP];
	disY_pid_arg.ki = Ano_Parame.set.xy_pid_14[KI];
	disY_pid_arg.kd_ex= 0;
	disY_pid_arg.kd_fb = Ano_Parame.set.xy_pid_14[KD];	
	disY_pid_arg.k_ff = 0;    
}

_pid_st disxPid;
_pid_st disyPid;

void Dis_Data_Pid_Task(u8 dT_ms,float Target_X_val,float Target_Y_val,u8 en)
{
    static float pid_disx_out = 0.00f;
    static float pid_disy_out = 0.00f;

    // // 阻尼系数
    // const float damping_factor = 0.1f;

    // 机体坐标系速度输出
    float body_vel[VEC_XYZ] = {0};

    // 世界坐标系速度输出
    float world_vel[VEC_XYZ] = {0};
    
//    // 世界坐标系下的来自传感器速度输入
//    float world_vel_damping[VEC_XYZ] = {0};

    // 机头方向向量（cos/sin值）
    float ref_ax[VEC_XYZ] = {0};

    if ((en) && (flag.taking_off == 1) && (comp_offline == 0))
    {
        /* 使能pid */
        disxPid.exp = Target_X_val;   // 期望
        disxPid.fb  = pos.x;                      // 反馈
        
        PID_calculate( dT_ms*1e-3f,            //周期（单位：秒）
                                0 ,				//前馈值
                                disxPid.exp ,				//期望值（设定值）
                                disxPid.fb ,			//反馈值（）
                                &disX_pid_arg,   //PID参数结构体
                                &disX_pid_val,	//PID数据结构体
                                50,//积分误差限幅
                                10 *flag.taking_off			//integration limit，积分限幅
                                    )	;
        pid_disx_out = disX_pid_val.out;
         /* 使能pid */
        disyPid.exp = Target_Y_val;   // 期望
        disyPid.fb  = pos.y;                      // 反馈
        
        PID_calculate( dT_ms*1e-3f,            //周期（单位：秒）
                                0 ,				//前馈值
                                disyPid.exp ,				//期望值（设定值）
                                disyPid.fb ,			//反馈值（）
                                &disY_pid_arg,   //PID参数结构体
                                &disY_pid_val,	//PID数据结构体
                                50,//积分误差限幅
                                10 *flag.taking_off			//integration limit，积分限幅
                                    )	;
        pid_disy_out = disY_pid_val.out;
		
        /* 设置机头方向向量（直接使用cos/sin值） */
        ref_ax[X] = ((float)pos.hx_x / 10000.0f);  // cos(yaw)
        ref_ax[Y] = ((float)pos.hx_y / 10000.0f);  // sin(yaw)
        
        // float body_vel_damping[VEC_XYZ] = {ano_of.of2_dx_fix, ano_of.of2_dy_fix, 0};
        // h2w_2d_trans(body_vel_damping, ref_ax, world_vel_damping);

        // /* 在世界坐标系下添加阻尼 */
        // pid_disx_out -= damping_factor * world_vel_damping[X];
        // pid_disy_out -= damping_factor * world_vel_damping[Y];
        
        /* 构建世界坐标系速度向量 */
        world_vel[X] = pid_disx_out;
        world_vel[Y] = pid_disy_out;
        
        /* 使用w2h_2d_trans进行坐标系转换 */
        w2h_2d_trans(world_vel, ref_ax, body_vel);
        Program_Ctrl_User_Set_HXYcmps(
            LIMIT(body_vel[X],-21,20),
            LIMIT(body_vel[Y],-21,20)
        );
    }
    else
    {
		Program_Ctrl_User_Set_HXYcmps(0,0);
    }
    
}

void Dis_ProgramCtrl_Task(u8 dT_ms)
{
    float Dis_Ctrl_val[2];
    u8 FC_Ctrl_flag;
    u8 en_flag;
     /* 判断遥控通道值 */
    if ((CH_N[CH_ROL] < -15)||(CH_N[CH_ROL] > 15)||(CH_N[CH_PIT] < -15)||(CH_N[CH_PIT] > 15))     //手动控制油门状态
    {
        FC_Ctrl_flag = 1;
    }
    else
    {
        FC_Ctrl_flag = 0;       // 遥控回中
		if(1){
			Dis_Ctrl_val[X] = tar.x;
            Dis_Ctrl_val[Y] = tar.y;
		}
    }
    /* 使能控制标志 */
    if (
    (FC_Ctrl_flag == 0) && (CH_N[CH6]> 400)
    )      // 如果在中位，通道6开启，启动pid控制
        en_flag = 1;
    else
        en_flag = 0;
    Dis_Data_Pid_Task(dT_ms,Dis_Ctrl_val[X],Dis_Ctrl_val[Y],en_flag);

}



