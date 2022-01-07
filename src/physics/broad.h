#ifndef RAW_PHYSICS_PHYSICS_BROAD_H
#define RAW_PHYSICS_PHYSICS_BROAD_H
#include "../render/graphics.h"

typedef struct {
	Entity* e1;
	Entity* e2;
} Broad_Collision_Pair;

Broad_Collision_Pair* broad_get_collision_pairs(Entity* entities);
Entity*** broad_collect_simulation_islands(Entity* entities, Broad_Collision_Pair* _collision_pairs);
void broad_simulation_islands_destroy(Entity*** simulation_islands);

#endif