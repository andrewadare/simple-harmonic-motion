#ifndef PID_CONTROL_H
#define PID_CONTROL_H

typedef struct {
  // Gain coefficients
  float kp;  // proportional
  float ki;  // integral
  float kd;  // derivative

  // Configuration and state variables
  float setpoint;
  float output;
  float min_output;
  float max_output;
  float error_sum;
} pid_control_t;

// Return a <= x <= b
float clip(const float x, const float a, const float b);

// Compute PID control output from input, a feed-forward value ff, and a
// timestep dt. The pid struct is modified with the results.
void pid_update(const float input, const float ff, const float dt,
                pid_control_t* pid);

#endif /* PID_CONTROL_H */
