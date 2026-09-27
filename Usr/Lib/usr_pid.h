#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

typedef enum{
    PID_POSITION = 0,
    PID_DELTA
}pid_mode_t;

typedef struct{
    pid_mode_t mode;

    float Kp;
    float Ki;
    float Kd;

    float max_out;
    float max_Iout;

    float set;
    float input;
    float out;
    float Pout;
    float Iout;
    float Dout;

    float Dbuf[3];
    float err[3];
}pid_t;

void usr_pid_init(pid_t* pid, uint8_t mode, const float PID[3], float max_out, float max_Iout);
float usr_pid_caculate(pid_t* pid, float input, float set);

#ifdef __cplusplus
}
#endif
