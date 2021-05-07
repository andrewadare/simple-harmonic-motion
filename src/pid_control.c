#include "pid_control.h"

#include <stddef.h>

float clip(const float x, const float a, const float b) {
  return x < a ? a : x > b ? b : x;
}

void pid_update(const float input, const float ff, const float dt,
                pid_control_t* pid, float* derivative) {
  static float* prev_input = NULL;
  const float e = pid->setpoint - input;  // the control error

  // Contribution from error derivative de/dt
  float derivative_term = 0.;
  if (derivative) {
    derivative_term = pid->kd * (*derivative);
  } else if (prev_input) {
    // Omit d(setpoint)/dt from de/dt to keep output smooth
    derivative_term = pid->kd * (input - *prev_input) / dt;
  }

  // Update integrator with accumulated error and keep it bounded
  pid->error_sum += pid->ki * e * dt;
  pid->error_sum = clip(pid->error_sum, pid->min_output, pid->max_output);

  // Compute output and keep it bounded
  pid->output = ff + pid->kp * e + pid->error_sum - derivative_term;
  pid->output = clip(pid->output, pid->min_output, pid->max_output);

  *prev_input = input;
}
