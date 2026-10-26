#ifndef BSP_CAN_H
#define BSP_CAN_H
#include "main.h"

extern volatile uint32_t g_can_cfg_ret;
extern volatile uint32_t g_can_start_ret;
extern volatile uint32_t g_can_notify_ret;

extern volatile uint32_t g_can_rx_cnt;
extern volatile uint32_t g_can_last_id;
extern volatile uint32_t g_can_id_cnt[8];

extern volatile uint32_t g_can_esr;
extern volatile uint32_t g_can_rf0r;
extern volatile uint32_t g_can_fa1r;
extern volatile uint32_t g_can_tec;
extern volatile uint32_t g_can_rec;
extern volatile uint32_t g_can_lec;
extern volatile uint32_t g_can_tx_ret;
extern volatile uint32_t g_can_tx_ok_cnt;
extern volatile uint32_t g_can_tx_err_cnt;

extern void can_filter_init(void);
extern void can_debug_sample(void);

#endif
