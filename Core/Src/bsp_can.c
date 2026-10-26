#include "bsp_can.h"
#include "main.h"


extern CAN_HandleTypeDef hcan1;

volatile uint32_t g_can_cfg_ret = 0xFFFFFFFFU;
volatile uint32_t g_can_start_ret = 0xFFFFFFFFU;
volatile uint32_t g_can_notify_ret = 0xFFFFFFFFU;

volatile uint32_t g_can_rx_cnt = 0;
volatile uint32_t g_can_last_id = 0xFFFFFFFFU;
volatile uint32_t g_can_id_cnt[8] = {0};

volatile uint32_t g_can_esr = 0;
volatile uint32_t g_can_rf0r = 0;
volatile uint32_t g_can_fa1r = 0;
volatile uint32_t g_can_tec = 0;
volatile uint32_t g_can_rec = 0;
volatile uint32_t g_can_lec = 0;
volatile uint32_t g_can_tx_ret = 0xFFFFFFFFU;
volatile uint32_t g_can_tx_ok_cnt = 0;
volatile uint32_t g_can_tx_err_cnt = 0;

void can_filter_init(void)
{

    CAN_FilterTypeDef can_filter_st;
    can_filter_st.FilterActivation = ENABLE;
    can_filter_st.FilterMode = CAN_FILTERMODE_IDMASK;
    can_filter_st.FilterScale = CAN_FILTERSCALE_32BIT;
    can_filter_st.FilterIdHigh = 0x0000;
    can_filter_st.FilterIdLow = 0x0000;
    can_filter_st.FilterMaskIdHigh = 0x0000;
    can_filter_st.FilterMaskIdLow = 0x0000;
    can_filter_st.FilterBank = 0;
    can_filter_st.FilterFIFOAssignment = CAN_RX_FIFO0;
    g_can_cfg_ret = (uint32_t)HAL_CAN_ConfigFilter(&hcan1, &can_filter_st);
    g_can_start_ret = (uint32_t)HAL_CAN_Start(&hcan1);
    g_can_notify_ret = (uint32_t)HAL_CAN_ActivateNotification(
        &hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
}

void can_debug_sample(void)
{
    uint32_t esr = hcan1.Instance->ESR;

    g_can_esr = esr;
    g_can_rf0r = hcan1.Instance->RF0R;
    g_can_fa1r = hcan1.Instance->FA1R;
    g_can_tec = (esr >> 16U) & 0xFFU;
    g_can_rec = (esr >> 24U) & 0xFFU;
    g_can_lec = (esr >> 4U) & 0x7U;
}
