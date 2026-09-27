#include "usr_pid.h"

#include <stdint.h>

#include "usr_algorithm.h"

void usr_pid_init(pid_t* pid, uint8_t mode, const float PID[3], float max_out, float max_Iout){
	if(pid == NULL || PID == NULL) return;
	pid->mode = mode;
	pid->Kp = PID[0];
	pid->Ki = PID[1];
	pid->Kd = PID[2];
	pid->max_out = max_out;
	pid->max_Iout = max_Iout;
	pid->Dbuf[0] = pid->Dbuf[1] = pid->Dbuf[2] = 0.0f;
	pid->err[0] = pid->err[1] = pid->err[2] = pid->Pout = pid->Iout = pid->Dout = pid->out;
}

float usr_pid_caculate(pid_t* pid, float input, float set){
    if(pid == NULL) return 0.0f;

    pid->err[2] = pid->err[1];
    pid->err[1] = pid->err[0];

    pid->set = set;
    pid->input = input;
    pid->err[0] = set - input;

    if(pid->mode == PID_POSITION){
        pid->Pout = pid->Kp * pid->err[0];
        pid->Iout += pid->Ki * pid->err[0];
        fclampf(pid->Iout, -pid->max_Iout, pid->max_Iout);
        pid->Dbuf[2] = pid->Dbuf[1];
        pid->Dbuf[1] = pid->Dbuf[0];
        pid->Dbuf[0] = (pid->err[0] - pid->err[1]);
        pid->Dout = pid->Kd * pid->Dbuf[0];

        pid->out = pid->Pout + pid->Iout + pid->Dout;
        fclampf(pid->out, -pid->max_out, pid->max_out);
    }else if(pid->mode == PID_DELTA){
        pid->Pout = pid->Kp * (pid->err[0] - pid->err[1]);
        pid->Iout = pid->Ki * pid->err[0];
        pid->Dbuf[2] = pid->Dbuf[1];
        pid->Dbuf[1] = pid->Dbuf[0];
        pid->Dbuf[0] = (pid->err[0] - 2.0f * pid->err[1] + pid->err[2]);
        pid->Dout = pid->Kd * pid->Dbuf[0];
        pid->out += pid->Pout + pid->Iout + pid->Dout;
        fclampf(pid->out, -pid->max_out, pid->max_out);
    }
    return pid->out;
}
