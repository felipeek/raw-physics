#ifndef RAW_PHYSICS_PHYSICS_PHYSICS_H
#define RAW_PHYSICS_PHYSICS_PHYSICS_H

#include <common.h>
#include "../entity.h"

void physics_simulate(Entity** entities, r64 delta_time);
#endif
