#include "my_func.h"
#define L_Go() {HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_SET); HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_RESET);}
#define L_Back() {HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_RESET); HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_SET);}
#define R_Go() {HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_SET); HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET);}
#define R_Back() {HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET); HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET);}
#define L_speed(pwmVal) __HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_2, pwmVal);
#define R_speed(pwmVal) __HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_1, pwmVal);
#define KP 1500
#define KI 1
#define KD 4000
#define K1 3
#define K2 1
#define K3 1
#define K4 3
#define K  1


void State::get_state() {
    L1 = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_14);
    L2 = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13);
    R1 = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_15);
    R2 = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_12);
    for (int i = 49; i > 0; i--) s[i] = s[i - 1];
    for (int i = 49; i > 0; i--) c[i] = c[i - 1];
    
    if ((L1 | L2 | R1 | R2) == 0 && state == wait) return;

    if ((L1 | L2 | R1 | R2) == 0 && state == linewalk) s[0] = 1;
    else s[0] = 0;
    if (s[0] - s[48] == 0 && s[0] - s[49] == 1) startline++;

    if ((L1 | L2 | R1) == 0 && R2 == 1 && state == linewalk) c[0] = 1;
    else c[0] = 0;

    if (startline > 2)
        state = stop;
    
    else if (c[0] - c[48] == 0 && c[0] - c[49] == 1)
    {
        if (corner == 5) corner = 1;
        else corner++;
        switch (corner)
        {
        case 1:
        case 2:
        case 5:
            state = turnleft;
            break;
        case 3:
        case 4:
            state = linewalk;
            break;
        }
    }
    else state = linewalk;
}

void PID::get_U(const State &state) {
    err[2] = err[1];
    err[1] = err[0];
    if ((state.L2 & state.R1) == 0)
        err[0] = K2 * state.L2 - K3 * state.R1;
    else
        err[0] = K1 * state.L1 - K4 * state.R2;
    U = U + KP * (err[0] - err[1]) + KI * err[0] + KD * (err[0] - 2 * err[1] + err[2]);
}

int Motor::range(int x) {
    if (x < -7199)
        return -7199;
    else if (x > 7199)
        return 7199;
    else return x;
}

int Motor::baseSpeed(int preset, const PID &pid) {
    int basespeed = preset - K * abs(pid.U);
    if (basespeed > 0)
        return basespeed;
    return 0;
}

void Motor::linewalk(int preset, const PID &pid) {
    int basespeed = baseSpeed(preset, pid);
    int L = range(basespeed + pid.U);
    int R = range(basespeed - pid.U);
    if (L > 0) {
        L_Go();
        L_speed(L);
    }
    else {
        L_Back();
        L_speed(-L);
    }
    if (R > 0) {
        R_Go();
        R_speed(R);
    }
    else {
        R_Back();
        R_speed(-R);
    }
}

void Motor::stop() {
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET);
}

void Motor::turnleft() {
    L_Back();
    L_speed(2500);
    R_Go();
    R_speed(3500);
}
