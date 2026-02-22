# 25年电赛H题整理

<img src=".\Pic\uav.jpg" style="zoom:33%;" />

文件介绍：

- ground_station：地面站相关代码，采用PyQT与无线串口实现。
- hardware：硬件相关。
- ros2_ws：机载电脑ros2功能包。
- stm32：匿名拓空者飞控相关代码。
- yolov5：机载电脑视觉识别相关。



## 零、总体架构

​	这是一套面向室内静态目标的无人机自主定位与追踪系统。由自制四旋翼平台（330轴距）与香橙派 5 Pro（RK3588）、激光雷达、单目相机、光流传感器和 IMU 组成；软件基于 ROS 2， 集成 Cartographer SLAM 实时建图定位，部署轻量化 YOLOv5 进行目标检测。针对地面静态目标，将单目视觉几何与机体姿态融合的实时位置估计算法，以消除相机倾斜引起的投影误差；同时采用有限状态机实现目标搜索、识别、定位与自主靠近的完整任务流程。

<img src=".\Pic\系统架构图.png" style="zoom:75%;" />

## 一.stm32飞控部分

​	使用匿名拓空者飞控，在此基础上我们进行飞控固件修改，以达到我们的目的。

​	首先明确我们的需求，需要修改飞控程序的哪些地方。

- 与机载电脑之间的数据传输

- 飞机自主飞行，雷达定点定位（PID）

- 视觉识别跟踪

  此时我们再整理整理需要改动和参考的地方。

##### 1.任务调度器 Ano_Scheduler.c

​	飞控的所有程序都基于这个调度器来跑，我们要研究的是如何往里面添加我们自己的任务程序。具体的实现方式很值得探讨，然而，然而。

​	例如：

```c
/* Ano_Scheduler.c */

static void Loop_100Hz(void)	//10ms执行一次
{
			test_rT[0]= GetSysTime_us ();
//////////////////////////////////////////////////////////////////////				
	/*遥控器数据处理*/
	RC_duty_task(10);
	
	/*飞行模式设置任务*/
	Flight_Mode_Set(10);
	
	//
	GPS_Data_Processing_Task(10);
	
	/*高度速度环控制*/
	Alt_1level_Ctrl(10e-3f);
	
	/*高度环控制*/
	Alt_2level_Ctrl(10e-3f);
	
	/*匿名光流状态检测*/	
	AnoOF_Check_State(0.01f);//AnoOF_DataAnl_Task(10);

	/*灯光控制*/	
	LED_Task2(10);
//////////////////////////////////////////////////////////////////////		
			test_rT[1]= GetSysTime_us ();
			test_rT[2] = (u32)(test_rT[1] - test_rT[0]) ;	
				
}
```

​	我们需要做的就是在相应的周期程序里添加我们自己的Task。

​	这里我大体写一个任务模板

```c
void My_Task(u8 dT_ms)
{
    static u16 timtmp = 0;			// 定时变量，在某些需要时间t的情况下使用
    if(Condition){					// 这里填入相应启动执行条件，例如遥控器某通道值处于特定值
    	timtmp += dT_ms;      		// 累加计时
     	Program();					// 执行相应程序
    }
}
```

##### 2.与机载电脑通讯 Ano_Comp_Link.c	

​	2d激光雷达slam返回了位置信息和姿态信息，我们应当确保这些信息正确传输给机载电脑。这里定义了一个简单的通信协议（见文件**Ano_Comp_Link.c**）。

| 帧头 (1字节) | 长度 (1字节) | 类型 (1字节) | 数据 (N字节)     | 校验和 (1字节) |
| :----------- | :----------- | :----------- | :--------------- | :------------- |
| `0xAA`       | `len`        | `type`       | `data[0..len-1]` | `checksum`     |

- **帧头**：固定为 `0xAA`，用于同步。

- **长度**：后续数据域的字节数（不包含帧头、长度、类型和校验和本身）。

- **类型**：标识数据类型，当前定义两种：

  - `POS = 0x01`：位置数据，长度应为8字节。
  - `TAR = 0x02`：目标数据，长度应为6字节。

- **数据**：小端模式存储的多字节数据。根据类型不同，解析为对应的结构体字段。

- **校验和**：从长度字节开始到数据域最后一个字节的累加和（仅取低8位），用于验证帧的完整性。

  将协议解析代码放置到串口2中断中。

```c
void Usart2_IRQ ( void )
{
    u8 com_data;

    if ( USART2->SR & USART_SR_ORE ) //ORE中断
    {
        com_data = USART2->DR;
    }

    //接收中断
    if ( USART_GetITStatus ( USART2, USART_IT_RXNE ) )
    {
        USART_ClearITPendingBit ( USART2, USART_IT_RXNE ); //清除中断标志

        com_data = USART2->DR;
        //AnoDTRxOneByteUart ( com_data );
		Airbone_Comp_Byte_Get(com_data);
    }
    //发送（进入移位）中断
    if ( USART_GetITStatus ( USART2, USART_IT_TXE ) )
    {

        USART2->DR = TxBuffer[TxCounter++]; //写DR清除中断标志
        if ( TxCounter == count )
        {
            USART2->CR1 &= ~USART_CR1_TXEIE;		//关闭TXE（发送中断）中断
        }


        //USART_ClearITPendingBit(USART2,USART_IT_TXE);
    }



}
```

将掉线检测函数放置到任务调度器：

```c
static void Loop_50Hz(void)	//20ms执行一次
{	
	//省略其他代码
    
	/*检测机载电脑是否掉线*/
	Comp_Offline_Check(20);	// 1000ms后判断电脑掉线
}
```

##### 3.程控自主飞行

​	我们需要实现无人机的定位，通过2dslam获取xy平面位置信息，再通过激光定高模块获取z方向高度，以低成本实现室内定位。

​	<img src=".\Pic\PID框图.png" alt="PID框图" style="zoom:25%;" />

代码见**stm32\ANO_PioneerPro-088\SRC\applications\User**。

```c
/*stm32\ANO_PioneerPro-088\SRC\applications\User\Ano_Program_Ctrl.c*/
// 高度PID参数
_PID_arg_st alt_pid_arg;
_PID_val_st alt_pid_val;

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
```



## 二、ros功能包

​	工作空间中有以下几个功能包：

- **lslidar_driver**：n10p雷达驱动

- **lslidar_msgs**：雷达msg

- **ano_cartographer**：SLAM定位算法，我们**定位相关的launch文件**存放在这里

- **ano_data**：数据交换，任务规划，是比赛更改最多的地方

  ​	前三个功能包不介绍的，可以直接翻一翻源码。着重介绍**ano_data**功能包，其余功能包的代码基本上不需要修改。

  ### 1.mission

​	这是执行自主飞行任务的逻辑所在，通过任务点列表控制无人机按顺序执行任务。以下是使用说明：

#### 1. 任务点配置

任务点定义在节点初始化时的 `mission_points` 列表中，每个元素为一个四元组：

```python
(x坐标, y坐标, z坐标, 任务类型字符串)
```

例如：

```python
self.mission_points = [
    (0.0, 0.0, 0.5, "wait"),      
    (0.4, -0.4, 0.5, "scan"),     
    (0.8, -0.8, 0.5, "wait"),     
    (1.2, -1.2, 0.5, "pass"),     
]
```

​	修改此列表即可自定义航线和任务。也可以通过动态改变任务列表实现自主任务添加。

- **insert_mission_point_after_current(self, x, y, z, mission_type)**
- **insert_mission_point_at_end(self, x, y, z, mission_type)**
- **insert_mission_point_at_index(self, index, x, y, z, mission_type)**

#### 2.任务类型

​	节点预定义任务类型，可在 `execute_mission()` 中扩展：

```python
    """ros2_ws\ano_data\ano_data\normal_mode.py"""
    
    def execute_mission(self, mission_type):
        """执行特定任务"""
        if mission_type == "wait":        
            return self.Wait()
        if mission_type == "pass":
            return self.Pass()
        if mission_type == "scan":
            return self.Scan()
        if mission_type == "flow":
            return self.Flow()
        return True
    """
    任务列表
    """
    def Pass(self):
        """逃过"""
        self.get_logger().info("跳过...")
        return True

    def Scan(self):
        """查找"""
        # 省略……
        return False # 如果任务没有完成，可以 return False 使任务列表卡在这里。

    def Wait(self):
        self.get_logger().info("执行等待任务...")
        time.sleep(5)  # 等待5秒
        return True
    
    def Flow(self):
        """跟随视觉识别到的目标"""
        self.get_logger().info("执行跟随任务...")
        # 暂时不开发
        return False  # 继续跟随任务
```

##### 任务状态机：

<img src=".\Pic\状态机框图.png" alt="状态机框图" style="zoom:25%;" />

