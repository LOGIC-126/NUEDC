/*==========================================================================
 * 描述    ：与机载电脑的通讯协议
 * 更新时间：2025-07-15 
 * 作者		 ：RZR
============================================================================*/
#include "Ano_Comp_Link.h"
#include "Ano_Program_Ctrl.h"

enum {
    STATE_HEADER = 0,
    STATE_LENGTH,
    STATE_TYPE,
    STATE_DATA,
    STATE_CHECK
};

typedef enum {
    POS = 0x01,
    TAR = 0x02
} DataType;

PositionFilter2D pos_fit;
PosData pos;
TarData tar;

#define COMP_OFFLINE_TIME_MS  1000
u16 comp_offline_check_time;
u8 comp_offline;

/**********************************************************************************************************
*函 数 名: Airbone_Comp_Byte_Get
*功能说明: 获取解析来自机载电脑的数据
*参    数: 时间（毫秒）
*返 回 值: 无
**********************************************************************************************************/
void Airbone_Comp_Byte_Get(u8 byte)
{
    static u8 state = STATE_HEADER;
    static u8 len = 0;
    static u8 type = 0;
    static u8 data_count = 0;
    static u8 data_buf[8];  // 足够存放最大数据(TAR的6字节)
    static u8 calc_checksum = 0;

    switch (state) {
        case STATE_HEADER:
            if (byte == 0xAA) {
                state = STATE_LENGTH;
                calc_checksum = 0;  // 重置校验和计算
            }
            break;

        case STATE_LENGTH:
            len = byte;
            calc_checksum += byte;  // 累加长度到校验和
            data_count = 0;
            state = STATE_TYPE;
            break;

        case STATE_TYPE:
            type = byte;
            calc_checksum += byte;  // 累加类型到校验和
            state = STATE_DATA;
            break;

        case STATE_DATA:
            if (data_count < sizeof(data_buf)) {
                data_buf[data_count] = byte;
                calc_checksum += byte;  // 累加数据到校验和
                data_count++;
            }
            
            // 检查是否接收完所有数据
            if (data_count >= len) {
                state = STATE_CHECK;
            }
            break;

        case STATE_CHECK:
            // 验证校验和 (len + type + data之和的低8位)
            if ((calc_checksum & 0xFF) == byte) {
                // 手动解析数据（小端模式）
                if (type == POS && len == 8) {
                    pos.x = (s16)((data_buf[1] << 8) | data_buf[0]);
                    pos.y = (s16)((data_buf[3] << 8) | data_buf[2]);
                    pos.hx_x = (s16)((data_buf[5] << 8) | data_buf[4]);  // 新增字段
                    pos.hx_y = (s16)((data_buf[7] << 8) | data_buf[6]);  // 新增字段
                    
                    // 使用位置数据：pos.x, pos.y, pos.hx_x, pos.hx_y
                    
                } 
                else if (type == TAR && len == 6) {
                    tar.x = (s16)((data_buf[1] << 8) | data_buf[0]);
                    tar.y = (s16)((data_buf[3] << 8) | data_buf[2]);
                    tar.z = (s16)((data_buf[5] << 8) | data_buf[4]);
                    
                    // 使用目标数据：tar.x, tar.y, tar.z
                }
                Comp_Check_Reset();
            }
            // 无论校验是否通过都回到初始状态
            state = STATE_HEADER;
            break;
    }
}


/**********************************************************************************************************
*函 数 名: Comp_Offline_Check
*功能说明: Comp掉线检测，用来检测硬件是否在线
*参    数: 时间（毫秒）
*返 回 值: 无
**********************************************************************************************************/
void Comp_Offline_Check(u8 dT_ms)
{
	if(comp_offline_check_time<COMP_OFFLINE_TIME_MS)
	{
		comp_offline_check_time += dT_ms;
	}
	else
	{
		comp_offline = 1;   // 掉线
		pos.x = 0;
		pos.y = 0;
		pos.hx_x = 0;
		pos.hx_y = 0;
		tar.x = 0;
		tar.y = 0;
		tar.z = 0;
	}
	
}

/**********************************************************************************************************
*函 数 名: Comp_Check_Reset
*功能说明: Comp掉线检测复位，证明没有掉线
*参    数: 无
*返 回 值: 无
**********************************************************************************************************/
static void Comp_Check_Reset()
{
	comp_offline_check_time = 0;
	comp_offline = 0;
}



