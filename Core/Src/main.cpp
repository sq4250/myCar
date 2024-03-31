#include "my_func.h"

static State state;
static PID pid;
static Motor motor;


int main() {

    init();

    while(1) {
        state.get_state();
        switch (state.state)
        {
        case wait:
            break;

        case linewalk:
            pid.get_U(state);
            motor.linewalk(1500, pid);
            delay_us(500);
            break;

        case turnleft: 
            HAL_Delay(300);
            motor.turnleft();
            HAL_Delay(270);
            break;

        case stop:
            HAL_Delay(600);
            motor.stop();
            HAL_Delay(HAL_MAX_DELAY);
            break;
        }
    }
}