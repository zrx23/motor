#ifndef BSP_PID_H
#define BSP_PID_H

#include "main.h"

#define POSITION_PID 1U
#define DELTA_PID    2U

typedef struct
{
    uint8_t mode;
    float Kp;
    float Ki;
    float Kd;
    float max_out;
    float max_iout;

    float set;
    float fdb;

    float out;
    float Pout;
    float Iout;
    float Dout;
    float Dbuf[3];
    float error[3];
    float I_Separation;
    float gama;
    float lastdout;
} PID_TypeDef;

extern PID_TypeDef PID_M3508;

void PID_Init(PID_TypeDef *pid,
              uint32_t mode,
              float max_out,
              float integral_limit,
              float kp,
              float ki,
              float kd,
              float integral_separation,
              float gamma);
void PID_Reset(PID_TypeDef *pid);
float PID_Calculate(PID_TypeDef *pid, float set, float ref);

#endif /* BSP_PID_H */
