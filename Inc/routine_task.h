#ifndef ROUTINE_TASK_H
#define ROUTINE_TASK_H

#ifdef __cplusplus
extern "C" {
#endif

void read_sensors(void);
void check_temperature_over_n_flick_led(void);

void routine_task(void const *arg);

#ifdef __cplusplus
}
#endif

#endif /* ROUTINE_TASK_H */
