#ifndef ACTUATOR_H
#define ACTUATOR_H

#include <stdbool.h>

typedef enum {
  DIRECTION_DOWN,
  DIRECTION_UP,
  DIRECTION_UNKNOWN
} actuator_direction_t;

typedef struct {
  int position;         // 0-4095
  int datum;            // start/rollover value for rotation counting
  float rotation_rate;  // signed ang. speed in 12-bit units per timestep
  int rotations;        // signed rotation count
  float radius;         // [cm] Half pitch diam. For rot. to lin. conversion
} pulley_t;

typedef struct {
  pulley_t pulley;
  float position;                  // cm above limit switch
  float speed;                     // cm/s, positive upward
  actuator_direction_t direction;  // linear up/down direction
  bool homed;                      // limit switch contacted
} actuator_t;

#endif /* ACTUATOR_H */