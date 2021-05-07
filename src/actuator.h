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
  actuator_direction_t direction;  // up/down
  int encoder_units;               // encoder position
  int datum;                       // "" at low point
  bool homed;                      // limit switch contacted
} actuator_t;

#endif /* ACTUATOR_H */
