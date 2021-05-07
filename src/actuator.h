#ifndef ACTUATOR_H
#define ACTUATOR_H

#include <stdbool.h>

typedef enum {
  DIRECTION_DOWN,
  DIRECTION_UP,
  DIRECTION_UNKNOWN
} actuator_direction_t;

typedef struct {
  float position;                  // cm above limit switch
  float speed;                     // cm/s, positive upward
  actuator_direction_t direction;  // linear up/down direction
  float pulley_radius;             // [cm] Half pitch diam.
  int encoder_units;               // position in encoder ticks
  int datum;                       // encoder ticks at low point
  bool homed;                      // limit switch contacted
} actuator_t;

#endif /* ACTUATOR_H */
