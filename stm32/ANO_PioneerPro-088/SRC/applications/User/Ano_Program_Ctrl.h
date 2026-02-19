#ifndef __ANO_PROGRAM_CTRL_H
#define	__ANO_PROGRAM_CTRL_H

#include "stm32f4xx.h"

typedef struct
{
	float exp;
	float fb;
	float out;

}_pid_st;

#define WINDOW_SIZE 50

// 2D位置滤波结构体
typedef struct {
    float x_buffer[WINDOW_SIZE+1];  // X坐标滤波数组
    float y_buffer[WINDOW_SIZE+1];  // Y坐标滤波数组
    u16 x_counter;       // X坐标计数器
    u16 y_counter;       // Y坐标计数器
    float filtered_x;    // 滤波后的X坐标
    float filtered_y;    // 滤波后的Y坐标
} PositionFilter2D;

extern u8 Test_flag;
extern PositionFilter2D pos_fit;

void Alt_pid_init_13(void);
void Alt_ProgramCtrl_Task(u8 dT_ms);
void Dis_pid_init_14(void);
void Dis_ProgramCtrl_Task(u8 dT_ms);


#endif



