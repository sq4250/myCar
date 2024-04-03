#ifndef __MY_FUNC
#define __MY_FUNC

#include "tim.h"
#include <cmath>
typedef enum {
    wait = 0,
    linewalk,
    turnleft,
    stop
} car_s;

class State {
private:
    uint8_t startline = 0, corner = 0, L1, L2, R1, R2, s[50] = {0}, c[50] = {0};
public:
    car_s state = wait;
    void get_state();
    friend class PID;
};

class PID {
private:
    int err[3] = {0};
public:
    int U = 0;
    void get_U(const State &state);
    friend class Motor;
};

class Motor {
    int range(int x);
    int baseSpeed(int preset, const PID &pid);
public:
    void linewalk(int preset, const PID &pid);
    void turnleft();
    void stop();
};

inline void delay_us(uint16_t t) {
    __HAL_TIM_SetCounter(&htim2, 0);
    HAL_TIM_Base_Start(&htim2);
    while (__HAL_TIM_GetCounter(&htim2) < t);
    HAL_TIM_Base_Stop(&htim2);
}

#endif