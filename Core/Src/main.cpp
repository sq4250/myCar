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
            motor.linewalk(2750, pid);
            delay_us(250);
            break;

        case turnleft:
            motor.turnleft();
            HAL_Delay(315);
            break;

        case stop:
            HAL_Delay(150);
            motor.stop();
            HAL_Delay(HAL_MAX_DELAY);
            break;
        }
    }
}