#ifndef __ANO_COMP_LINK_H
#define	__ANO_COMP_LINK_H

#include "stm32f4xx.h"

typedef struct {
    s16 x;
    s16 y;
    s16 hx_x;
    s16 hx_y;
} PosData;

typedef struct {
    s16 x;
    s16 y;
    s16 z;
} TarData;

extern PosData pos;
extern TarData tar;
extern u8 comp_offline;


void Airbone_Comp_Byte_Get(u8 byte);
void Comp_Offline_Check(u8 dT_ms);
static void Comp_Check_Reset(void);

#endif
