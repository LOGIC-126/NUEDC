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

## 二、ros功能包

