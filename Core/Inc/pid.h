#ifndef PID_H
#define PID_H

typedef struct {
    float Kp;
    float Ki;
    float Kd;

    float previous_error;
    float integral;

    float output_max;
    float output_min;
} PID_t;

void PID_Reset(PID_t *pid);
float PID_Update(PID_t *pid, float target, float feedback, float dt);



#endif