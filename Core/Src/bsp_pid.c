#include "bsp_pid.h"

#include <math.h>

/*
 * Positional PID adapted from the extra-energy-mechanism-code project.
 * The speed loop runs every 5 ms; the C620 still performs its internal
 * current loop.
 */
PID_TypeDef PID_M3508;

static float abs_limit(float value, float limit)
{
    if (value > limit)
    {
        return limit;
    }
    if (value < -limit)
    {
        return -limit;
    }
    return value;
}

void PID_Reset(PID_TypeDef *pid)
{
    if (pid == NULL)
    {
        return;
    }

    pid->set = 0.0f;
    pid->fdb = 0.0f;
    pid->out = 0.0f;
    pid->Pout = 0.0f;
    pid->Iout = 0.0f;
    pid->Dout = 0.0f;
    pid->Dbuf[0] = 0.0f;
    pid->Dbuf[1] = 0.0f;
    pid->Dbuf[2] = 0.0f;
    pid->error[0] = 0.0f;
    pid->error[1] = 0.0f;
    pid->error[2] = 0.0f;
    pid->lastdout = 0.0f;
}

void PID_Init(PID_TypeDef *pid,
              uint32_t mode,
              float max_out,
              float integral_limit,
              float kp,
              float ki,
              float kd,
              float integral_separation,
              float gamma)
{
    if (pid == NULL)
    {
        return;
    }

    pid->mode = (uint8_t)mode;
    pid->max_out = max_out;
    pid->max_iout = integral_limit;
    pid->Kp = kp;
    pid->Ki = ki;
    pid->Kd = kd;
    pid->I_Separation = integral_separation;
    pid->gama = gamma;
    PID_Reset(pid);
}

float PID_Calculate(PID_TypeDef *pid, float set, float ref)
{
    float previous_out;

    if (pid == NULL)
    {
        return 0.0f;
    }

    previous_out = pid->out;
    pid->error[2] = pid->error[1];
    pid->error[1] = pid->error[0];
    pid->set = set;
    pid->fdb = ref;
    pid->error[0] = set - ref;

    if (fabsf(pid->error[0]) > pid->I_Separation)
    {
        pid->Iout = 0.0f;
    }

    if (pid->mode == POSITION_PID)
    {
        pid->Pout = pid->Kp * pid->error[0];
        pid->Iout += pid->Ki * pid->error[0];

        pid->Dbuf[2] = pid->Dbuf[1];
        pid->Dbuf[1] = pid->Dbuf[0];
        pid->Dbuf[0] = pid->error[0] - pid->error[1];

        pid->Dout = pid->Kd *
                    ((1.0f - pid->gama) * pid->Dbuf[0] +
                     pid->gama * pid->lastdout);
        pid->lastdout = pid->Dout;

        pid->Iout = abs_limit(pid->Iout, pid->max_iout);
        pid->out = pid->Pout + pid->Iout + pid->Dout;
        pid->out = abs_limit(pid->out, pid->max_out);
    }
    else if (pid->mode == DELTA_PID)
    {
        pid->Pout = pid->Kp * (pid->error[0] - pid->error[1]);
        pid->Iout = pid->Ki * pid->error[0];

        pid->Dbuf[2] = pid->Dbuf[1];
        pid->Dbuf[1] = pid->Dbuf[0];
        pid->Dbuf[0] = pid->error[0] -
                       2.0f * pid->error[1] +
                       pid->error[2];

        pid->Dout = pid->Kd * pid->Dbuf[0];
        pid->out = previous_out + pid->Pout + pid->Iout + pid->Dout;
        pid->out = abs_limit(pid->out, pid->max_out);
    }

    return pid->out;
}
