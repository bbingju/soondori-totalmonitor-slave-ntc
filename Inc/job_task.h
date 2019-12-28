#ifndef JOB_TASK_H
#define JOB_TASK_H

#include <stdio.h>

typedef enum {
    JOB_TYPE_NONE,
    JOB_TYPE_TO_INTERNAL,
    JOB_TYPE_FROM_INTERNAL,
    JOB_TYPE_ROUTINE_MEASURE_TEMPERATURE,
    JOB_TYPE_ROUTINE_CHECK_TEMP_OVER,
    JOB_TYPE_MAX,
} JOB_TYPE_E;

#ifdef __cplusplus
extern "C" {
#endif

int post_job(JOB_TYPE_E type, void const *data, size_t datalen);
void job_task(void const *argument);

#ifdef __cplusplus
}
#endif

#endif /* JOB_TASK_H */
