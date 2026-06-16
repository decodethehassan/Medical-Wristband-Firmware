#ifndef AS6221_TASK_H_
#define AS6221_TASK_H_

#include <stdbool.h>
#include <stdint.h>

void as6221_task_start(void);

/* Returns the most recent AS6221 temperature sample and its age. */
bool as6221_get_latest_temp_c(float *temp_c_out, int64_t *age_ms_out);

#endif /* AS6221_TASK_H_ */
