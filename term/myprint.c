#include "myprint.h"
void printMotorSet(const MotorSet* motorSet) {
    if (motorSet == NULL) {
        printf("MotorSet is NULL\n");
        return;
    }

    printf("MotorSet Data:\n");
    printf("Target Positions: ");
    for (int i = 0; i < MOTOR_NUM; i++) {
        printf("%d ", motorSet->target_pos[i]);
    }
    printf("\n");

    printf("Actual Positions: ");
    for (int i = 0; i < MOTOR_NUM; i++) {
        printf("%d ", motorSet->actual_pos[i]);
    }
    printf("\n");

    printf("Planned Positions: ");
    for (int i = 0; i < MOTOR_NUM; i++) {
        printf("%d ", motorSet->plan_pos[i]);
    }
    printf("\n");

    printf("Previous Mode: %d\n", motorSet->previous_mode);
    printf("Current Mode: %d\n", motorSet->mode);

    printf("States: ");
    for (int i = 0; i < MOTOR_NUM; i++) {
        printf("%d ", motorSet->state[i]);
    }
    printf("\n");
}

